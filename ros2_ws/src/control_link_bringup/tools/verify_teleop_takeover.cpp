#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/source_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using SourceStatus = control_link_interfaces::msg::SourceStatus;
	using Twist = geometry_msgs::msg::Twist;
	using SteadyTime = std::chrono::steady_clock::time_point;
	constexpr char kNav2SourceId[] = "nav2";
	constexpr char kTeleopSourceId[] = "teleop";

	struct ScenarioEvidence
	{
		bool publishing_started{false};
		bool publishing_stopped{false};
		bool saw_initial_nav2{false};
		bool saw_switch_to_teleop{false};
		bool saw_fallback_to_nav2{false};
		bool saw_canonical_teleop{false};
		bool saw_canonical_nav2_after_fallback{false};
		bool saw_teleop_lease_expire{false};
		bool saw_nav2_lease_during_fallback{false};
		std::uint64_t teleop_accepted_before{0U};
		std::uint64_t teleop_accepted_after{0U};
		std::uint64_t switch_sequence{0U};
		std::uint64_t fallback_sequence{0U};
		SteadyTime first_teleop_publish_at{};
		SteadyTime last_teleop_publish_at{};
		SteadyTime switch_observed_at{};
		SteadyTime fallback_observed_at{};
	};

	const control_link_contract::ProfileCommon & profile_common(
		const control_link_contract::ProfileConfig & profile)
	{
		return std::visit(
			[](const auto & concrete_profile)
				-> const control_link_contract::ProfileCommon &
			{
				return concrete_profile.common;
			},
			profile);
	}

	template<typename Predicate>
	bool spin_until(
		rclcpp::executors::SingleThreadedExecutor & executor,
		SteadyTime deadline,
		Predicate predicate)
	{
		while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
		{
			executor.spin_some();
			if (predicate())
			{
				return true;
			}
			std::this_thread::sleep_for(5ms);
		}

		executor.spin_some();
		return predicate();
	}

	std::int64_t elapsed_milliseconds(SteadyTime begin, SteadyTime end)
	{
		if (end < begin)
		{
			throw std::logic_error("steady observation time moved backwards");
		}
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			end - begin).count();
	}

	int run_scenario(const rclcpp::Node::SharedPtr & node)
	{
		const auto profile_path = std::filesystem::path{
			node->declare_parameter<std::string>("profile_path", "")};
		const auto config_root = std::filesystem::path{
			node->declare_parameter<std::string>("config_root", "")};
		const auto timeout_ms = node->declare_parameter<std::int64_t>(
			"timeout_ms", 45000);
		const auto teleop_publish_hz = node->declare_parameter<double>(
			"teleop_publish_hz", 20.0);
		const auto teleop_angular_velocity = node->declare_parameter<double>(
			"teleop_angular_velocity_radps", -0.35);
		if (profile_path.empty() || config_root.empty())
		{
			throw std::invalid_argument(
				"profile_path and config_root must be non-empty");
		}
		if (timeout_ms <= 0)
		{
			throw std::invalid_argument("timeout_ms must be positive");
		}
		if (!std::isfinite(teleop_publish_hz) || teleop_publish_hz < 20.0)
		{
			throw std::invalid_argument(
				"teleop_publish_hz must be finite and at least 20Hz");
		}

		const auto bundle = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto *robot_profile = std::get_if<control_link_contract::RobotProfile>(
			bundle->profile.get());
		if (robot_profile == nullptr)
		{
			throw std::invalid_argument(
				"teleop takeover scenario requires Robot Profile");
		}

		const auto &common = profile_common(*bundle->profile);
		const auto teleop_ingress = common.ingress.find(kTeleopSourceId);
		if (teleop_ingress == common.ingress.end())
		{
			throw std::invalid_argument(
				"Robot Profile does not define teleop ingress");
		}
		const auto teleop_policy = bundle->source_policy->sources.find(
			kTeleopSourceId);
		const auto nav2_policy = bundle->source_policy->sources.find(kNav2SourceId);
		if (teleop_policy == bundle->source_policy->sources.end() ||
			nav2_policy == bundle->source_policy->sources.end())
		{
			throw std::invalid_argument(
				"Robot takeover sources are absent from SourcePolicy");
		}
		if (teleop_policy->second.priority <= nav2_policy->second.priority)
		{
			throw std::invalid_argument(
				"teleop priority must be greater than nav2 priority");
		}
		if (!std::isfinite(teleop_angular_velocity) ||
			std::abs(teleop_angular_velocity) >
			bundle->gateway_contract->limits.max_abs_angular_velocity_radps)
		{
			throw std::invalid_argument(
				"teleop angular velocity must be finite and within the Gateway Contract limit");
		}
		for (const auto *source_id : {kNav2SourceId, kTeleopSourceId})
		{
			if (std::find(
					common.enabled_sources.begin(),
					common.enabled_sources.end(),
					source_id) == common.enabled_sources.end())
			{
				throw std::invalid_argument(
					"Robot takeover source is not enabled by Profile: " +
					std::string{source_id});
			}
		}

		control_link_contract::QosFactory qos_factory{
			bundle->gateway_contract};
		const auto &state_contract =
			bundle->gateway_contract->state_topics.at("gateway_state");
		const auto &source_status_contract =
			bundle->gateway_contract->state_topics.at("source_status");
		if (!state_contract.qos_profile.has_value() ||
			!source_status_contract.qos_profile.has_value())
		{
			throw std::logic_error(
				"Gateway state evidence Topics require Contract QoS profiles");
		}

		ScenarioEvidence evidence;
		auto gateway_state_subscription = node->create_subscription<GatewayState>(
			state_contract.topic,
			qos_factory.make(state_contract.qos_profile.value()),
			[&evidence](const GatewayState &state)
			{
				if (!evidence.publishing_started &&
					state.state == GatewayState::ACTIVE &&
					state.active_source_id == kNav2SourceId)
				{
					evidence.saw_initial_nav2 = true;
				}
				if (evidence.publishing_started &&
					!evidence.publishing_stopped &&
					state.state == GatewayState::ACTIVE &&
					state.active_source_id == kTeleopSourceId &&
					state.reason_code == GatewayState::REASON_SOURCE_SWITCH)
				{
					if (!evidence.saw_switch_to_teleop)
					{
						evidence.switch_observed_at = std::chrono::steady_clock::now();
					}
					evidence.saw_switch_to_teleop = true;
					evidence.switch_sequence = state.active_source_sequence;
				}
				if (evidence.publishing_stopped &&
					state.state == GatewayState::ACTIVE &&
					state.active_source_id == kNav2SourceId &&
					state.reason_code == GatewayState::REASON_SOURCE_FALLBACK)
				{
					if (!evidence.saw_fallback_to_nav2)
					{
						evidence.fallback_observed_at = std::chrono::steady_clock::now();
					}
					evidence.saw_fallback_to_nav2 = true;
					evidence.fallback_sequence = state.active_source_sequence;
				}
			});
		auto source_status_subscription = node->create_subscription<SourceStatus>(
			source_status_contract.topic,
			qos_factory.make(source_status_contract.qos_profile.value()),
			[&evidence](const SourceStatus &status)
			{
				if (status.source_id == kTeleopSourceId)
				{
					if (!evidence.publishing_started)
					{
						evidence.teleop_accepted_before = status.accepted_count;
					}
					evidence.teleop_accepted_after = std::max(
						evidence.teleop_accepted_after,
						status.accepted_count);
					if (evidence.publishing_stopped && status.command_valid &&
						!status.lease_valid)
					{
						evidence.saw_teleop_lease_expire = true;
					}
				}
				if (status.source_id == kNav2SourceId &&
					evidence.publishing_stopped && status.command_valid &&
					status.lease_valid)
				{
					evidence.saw_nav2_lease_during_fallback = true;
				}
			});
		auto canonical_subscription = node->create_subscription<ControlCommand>(
			bundle->gateway_contract->output.topic,
			qos_factory.make(bundle->gateway_contract->output.qos_profile),
			[&evidence](const ControlCommand &command)
			{
				if (evidence.publishing_started &&
					command.mode == ControlCommand::MODE_NORMAL &&
					command.source_id == kTeleopSourceId)
				{
					evidence.saw_canonical_teleop = true;
				}
				if (evidence.saw_fallback_to_nav2 &&
					command.mode == ControlCommand::MODE_NORMAL &&
					command.source_id == kNav2SourceId)
				{
					evidence.saw_canonical_nav2_after_fallback = true;
				}
			});

		const auto raw_input_qos = rclcpp::QoS{rclcpp::KeepLast{10}}
			.reliable()
			.durability_volatile();
		auto teleop_publisher = node->create_publisher<Twist>(
			teleop_ingress->second.input_topic,
			raw_input_qos);
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);

		const auto scenario_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};
		if (!spin_until(
				executor,
				scenario_deadline,
				[&evidence, &teleop_publisher]()
				{
					return evidence.saw_initial_nav2 &&
						teleop_publisher->get_subscription_count() > 0U;
				}))
		{
			throw std::runtime_error(
				"timed out waiting for ACTIVE/nav2 and teleop ingress discovery");
		}

		const auto publish_period = std::chrono::duration_cast<
			std::chrono::steady_clock::duration>(
			std::chrono::duration<double>{1.0 / teleop_publish_hz});
		Twist teleop_twist;
		teleop_twist.angular.z = teleop_angular_velocity;
		evidence.publishing_started = true;
		evidence.first_teleop_publish_at = std::chrono::steady_clock::now();
		auto next_publish = evidence.first_teleop_publish_at;
		const auto switch_deadline = evidence.first_teleop_publish_at +
			std::chrono::milliseconds{
				static_cast<std::int64_t>(
					bundle->gateway_contract->gateway.source_switch_hold_ms) + 1500};
		while (rclcpp::ok() && std::chrono::steady_clock::now() < switch_deadline &&
			!evidence.saw_switch_to_teleop)
		{
			const auto now = std::chrono::steady_clock::now();
			if (now >= next_publish)
			{
				teleop_publisher->publish(teleop_twist);
				evidence.last_teleop_publish_at = now;
				next_publish = now + publish_period;
			}
			executor.spin_some();
			std::this_thread::sleep_for(2ms);
		}
		if (!evidence.saw_switch_to_teleop)
		{
			throw std::runtime_error(
				"Gateway did not switch from nav2 to the higher-priority teleop source");
		}

		// 接管后继续发一段时间，证明不是单个样本造成的瞬时状态观察
		const auto post_switch_deadline = evidence.switch_observed_at + 300ms;
		while (rclcpp::ok() && std::chrono::steady_clock::now() < post_switch_deadline)
		{
			const auto now = std::chrono::steady_clock::now();
			if (now >= next_publish)
			{
				teleop_publisher->publish(teleop_twist);
				evidence.last_teleop_publish_at = now;
				next_publish = now + publish_period;
			}
			executor.spin_some();
			std::this_thread::sleep_for(2ms);
		}
		evidence.publishing_stopped = true;

		const auto fallback_deadline = evidence.last_teleop_publish_at +
			std::chrono::milliseconds{
				static_cast<std::int64_t>(teleop_policy->second.lease_timeout_ms) + 1000};
		if (!spin_until(
				executor,
				std::min(scenario_deadline, fallback_deadline),
				[&evidence]()
				{
					return evidence.saw_fallback_to_nav2 &&
						evidence.saw_canonical_nav2_after_fallback &&
						evidence.saw_teleop_lease_expire &&
						evidence.saw_nav2_lease_during_fallback;
				}))
		{
			throw std::runtime_error(
				"teleop lease expiry did not produce a complete ACTIVE/nav2 fallback evidence set");
		}

		const auto switch_elapsed_ms = elapsed_milliseconds(
			evidence.first_teleop_publish_at,
			evidence.switch_observed_at);
		const auto fallback_elapsed_ms = elapsed_milliseconds(
			evidence.last_teleop_publish_at,
			evidence.fallback_observed_at);
		const auto switch_hold_ms = static_cast<std::int64_t>(
			bundle->gateway_contract->gateway.source_switch_hold_ms);
		const auto lease_timeout_ms = static_cast<std::int64_t>(
			teleop_policy->second.lease_timeout_ms);
		if (switch_elapsed_ms < switch_hold_ms)
		{
			throw std::runtime_error(
				"teleop switched before Contract source_switch_hold_ms elapsed");
		}
		if (fallback_elapsed_ms > lease_timeout_ms + 500)
		{
			throw std::runtime_error(
				"nav2 fallback exceeded teleop lease plus observation allowance");
		}
		if (!evidence.saw_canonical_teleop ||
			evidence.teleop_accepted_after <= evidence.teleop_accepted_before)
		{
			throw std::runtime_error(
				"teleop source was selected without complete accepted/canonical evidence");
		}

		RCLCPP_INFO(
			node->get_logger(),
			"TELEOP_TAKEOVER_SUCCEEDED switch_ms=%ld fallback_ms=%ld "
			"teleop_sequence=%lu nav2_sequence=%lu accepted_delta=%lu",
			static_cast<long>(switch_elapsed_ms),
			static_cast<long>(fallback_elapsed_ms),
			static_cast<unsigned long>(evidence.switch_sequence),
			static_cast<unsigned long>(evidence.fallback_sequence),
			static_cast<unsigned long>(
				evidence.teleop_accepted_after - evidence.teleop_accepted_before));

		(void)gateway_state_subscription;
		(void)source_status_subscription;
		(void)canonical_subscription;
		return 0;
	}
}  // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	int result = 1;
	try
	{
		auto node = std::make_shared<rclcpp::Node>("verify_teleop_takeover");
		result = run_scenario(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("verify_teleop_takeover"),
			"Teleop takeover scenario failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return result;
}
