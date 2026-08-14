#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using ControlCommand = control_link_interfaces::msg::ControlCommand;

	std::string gateway_namespace(const std::string &gateway_fqn)
	{
		const auto separator = gateway_fqn.rfind('/');
		if (separator == std::string::npos || separator == 0U)
		{
			throw std::invalid_argument(
				"Gateway FQN must include a non-root namespace for replay isolation");
		}
		return gateway_fqn.substr(0U, separator);
	}

	std::string replay_topic(
		const std::string &live_topic,
		const std::string &live_namespace,
		const std::string &replay_namespace)
	{
		if (live_topic.size() <= live_namespace.size() ||
			live_topic.compare(0U, live_namespace.size(), live_namespace) != 0 ||
			live_topic[live_namespace.size()] != '/')
		{
			throw std::invalid_argument(
				"SourcePolicy topic is outside the Gateway namespace: " + live_topic);
		}
		return replay_namespace + live_topic.substr(live_namespace.size());
	}

	class ReplayInputBridge final : public rclcpp::Node
	{
	public:
		ReplayInputBridge()
			: rclcpp::Node("replay_input_bridge")
		{
			const auto profile_path = std::filesystem::path{
				declare_parameter<std::string>("profile_path", "")};
			const auto config_root = std::filesystem::path{
				declare_parameter<std::string>("config_root", "")};
			const auto requested_replay_namespace = declare_parameter<std::string>(
				"replay_namespace", "");

			bundle_ = control_link_contract::load_contract_bundle(
				profile_path,
				config_root);
			(void)control_link_contract::validate_fastdds_process_environment(
				*bundle_->profile,
				"ReplayInputBridge");
			const auto *adas_profile =
				std::get_if<control_link_contract::AdasProfile>(bundle_->profile.get());
			if (adas_profile == nullptr)
			{
				throw std::invalid_argument("ReplayInputBridge requires profile_id=adas");
			}
			if (requested_replay_namespace != adas_profile->replay.input_namespace)
			{
				throw std::invalid_argument(
					"replay_namespace must match ADAS Profile; expected=" +
					adas_profile->replay.input_namespace + "; actual=" +
					requested_replay_namespace);
			}

			const auto live_namespace = gateway_namespace(
				bundle_->gateway_contract->gateway.node_fqn);
			if (requested_replay_namespace == live_namespace)
			{
				throw std::invalid_argument(
					"replay_namespace must differ from the live Gateway namespace");
			}
			control_link_contract::QosFactory qos_factory{
				bundle_->gateway_contract};
			for (const auto &source_id : adas_profile->common.enabled_sources)
			{
				const auto source = bundle_->source_policy->sources.find(source_id);
				if (source == bundle_->source_policy->sources.end())
				{
					throw std::logic_error(
						"enabled replay source is missing from SourcePolicy: " + source_id);
				}

				auto channel = std::make_shared<SourceChannel>();
				channel->source_id = source_id;
				channel->live_topic = source->second.topic;
				channel->replay_topic = replay_topic(
					channel->live_topic,
					live_namespace,
					requested_replay_namespace);
				channel->publisher = create_publisher<ControlCommand>(
					channel->live_topic,
					qos_factory.make(bundle_->gateway_contract->input.qos_profile));
				const std::weak_ptr<SourceChannel> weak_channel{channel};
				channel->subscription = create_subscription<ControlCommand>(
					channel->replay_topic,
					qos_factory.make(bundle_->gateway_contract->input.qos_profile),
					[this, weak_channel](const ControlCommand &recorded)
					{
						const auto channel = weak_channel.lock();
						if (channel != nullptr)
						{
							republish(recorded, *channel);
						}
					});
				RCLCPP_INFO(
					get_logger(),
					"Replay source bridge ready: source=%s, replay=%s, live=%s",
					channel->source_id.c_str(),
					channel->replay_topic.c_str(),
					channel->live_topic.c_str());
				channels_.push_back(std::move(channel));
			}
		}

	private:
		struct SourceChannel final
		{
			std::string source_id;
			std::string replay_topic;
			std::string live_topic;
			std::uint64_t sequence{0U};
			rclcpp::Publisher<ControlCommand>::SharedPtr publisher;
			rclcpp::Subscription<ControlCommand>::SharedPtr subscription;
		};

		void republish(
			const ControlCommand &recorded,
			SourceChannel &channel)
		{
			if (channel.sequence == std::numeric_limits<std::uint64_t>::max())
			{
				RCLCPP_ERROR_ONCE(
					get_logger(),
					"Replay sequence exhausted: source=%s",
					channel.source_id.c_str());
				return;
			}

			const auto now = get_clock()->now();
			if (now.nanoseconds() <= 0)
			{
				RCLCPP_WARN_THROTTLE(
					get_logger(),
					*get_clock(),
					2000,
					"Replay bridge is waiting for a positive ROS clock");
				return;
			}

			ControlCommand command = recorded;
			command.source_stamp = static_cast<builtin_interfaces::msg::Time>(now);
			command.source_id = channel.source_id;
			command.source_sequence = ++channel.sequence;
			channel.publisher->publish(command);
		}

		control_link_contract::ContractBundlePtr bundle_;
		std::vector<std::shared_ptr<SourceChannel>> channels_;
	};
}  // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<ReplayInputBridge>();
		rclcpp::spin(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("replay_input_bridge"),
			"Replay input bridge failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
