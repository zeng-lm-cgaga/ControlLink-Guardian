#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "control_link_bringup/reconfiguration_transaction.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/reconfiguration.hpp"

namespace control_link_bringup
{
	namespace
	{
		using namespace std::chrono_literals;
		using ChangeState = lifecycle_msgs::srv::ChangeState;
		using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
		using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
		using GetState = lifecycle_msgs::srv::GetState;

		constexpr char kDefaultGatewayFqn[] = "/control_link/gateway";
		constexpr char kDefaultLockPath[] =
			"/tmp/control_link_guardian_gateway_reconfigure.lock";

		class OwnedFileDescriptor
		{
		public:
			explicit OwnedFileDescriptor(int descriptor = -1) noexcept
				: descriptor_(descriptor)
			{
			}

			~OwnedFileDescriptor()
			{
				reset();
			}

			OwnedFileDescriptor(const OwnedFileDescriptor &) = delete;
			OwnedFileDescriptor &operator=(const OwnedFileDescriptor &) = delete;

			OwnedFileDescriptor(OwnedFileDescriptor &&other) noexcept
				: descriptor_(std::exchange(other.descriptor_, -1))
			{
			}

			OwnedFileDescriptor &operator=(OwnedFileDescriptor &&other) noexcept
			{
				if (this != &other)
				{
					reset();
					descriptor_ = std::exchange(other.descriptor_, -1);
				}
				return *this;
			}

			[[nodiscard]] int get() const noexcept
			{
				return descriptor_;
			}

			void reset(int descriptor = -1) noexcept
			{
				if (descriptor_ >= 0)
				{
					(void)::close(descriptor_);
				}
				descriptor_ = descriptor;
			}

		private:
			int descriptor_;
		};

		[[noreturn]] void throw_errno(const std::string &operation)
		{
			throw std::runtime_error(
				operation + " failed: " + std::string{std::strerror(errno)});
		}

		void require_absolute_output_path(
			const std::filesystem::path &path,
			const char *name)
		{
			if (path.empty() || !path.is_absolute())
			{
				throw std::invalid_argument(
					std::string{name} + " must be an absolute path");
			}
			const auto parent = path.parent_path();
			if (parent.empty() || !std::filesystem::is_directory(parent))
			{
				throw std::invalid_argument(
					std::string{name} + " parent must be an existing directory");
			}
		}

		std::filesystem::path canonical_input_path(
			const std::filesystem::path &path,
			const char *name,
			bool require_directory)
		{
			if (path.empty() || !path.is_absolute())
			{
				throw std::invalid_argument(
					std::string{name} + " must be an absolute path");
			}
			const auto canonical = std::filesystem::canonical(path);
			if (require_directory != std::filesystem::is_directory(canonical))
			{
				throw std::invalid_argument(
					std::string{name} +
					(require_directory ? " must be a directory" : " must be a regular file"));
			}
			if (!require_directory && !std::filesystem::is_regular_file(canonical))
			{
				throw std::invalid_argument(
					std::string{name} + " must be a regular file");
			}
			return canonical;
		}

		std::filesystem::path normalized_output_path(
			const std::filesystem::path &path)
		{
			return std::filesystem::canonical(path.parent_path()) / path.filename();
		}

		void write_all(int descriptor, const std::string &content)
		{
			std::size_t offset = 0U;
			while (offset < content.size())
			{
				const auto written = ::write(
					descriptor,
					content.data() + offset,
					content.size() - offset);
				if (written < 0)
				{
					if (errno == EINTR)
					{
						continue;
					}
					throw_errno("write transaction file");
				}
				if (written == 0)
				{
					throw std::runtime_error(
						"write transaction file made no progress");
				}
				offset += static_cast<std::size_t>(written);
			}
		}

		// 同目录临时文件加 rename 保证观察者只会看到旧版或完整新版
		void atomic_write_text(
			const std::filesystem::path &path,
			const std::string &content)
		{
			require_absolute_output_path(path, "transaction output path");
			const auto temporary = std::filesystem::path{
				path.string() + ".tmp." + std::to_string(::getpid())};
			(void)::unlink(temporary.c_str());

			OwnedFileDescriptor output{::open(
				temporary.c_str(),
				O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
				S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)};
			if (output.get() < 0)
			{
				throw_errno("open transaction temporary file");
			}

			try
			{
				write_all(output.get(), content);
				if (::fsync(output.get()) != 0)
				{
					throw_errno("fsync transaction temporary file");
				}
				output.reset();
				if (::rename(temporary.c_str(), path.c_str()) != 0)
				{
					throw_errno("rename transaction file");
				}

				OwnedFileDescriptor directory{::open(
					path.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC)};
				if (directory.get() < 0)
				{
					throw_errno("open transaction output directory");
				}
				if (::fsync(directory.get()) != 0)
				{
					throw_errno("fsync transaction output directory");
				}
			}
			catch (...)
			{
				(void)::unlink(temporary.c_str());
				throw;
			}
		}

		class ExclusiveTransactionLock
		{
		public:
			explicit ExclusiveTransactionLock(const std::filesystem::path &path)
			{
				require_absolute_output_path(path, "transaction lock path");
				descriptor_.reset(::open(
					path.c_str(),
					O_RDWR | O_CREAT | O_CLOEXEC,
					S_IRUSR | S_IWUSR));
				if (descriptor_.get() < 0)
				{
					throw_errno("open transaction lock");
				}
				if (::flock(descriptor_.get(), LOCK_EX | LOCK_NB) != 0)
				{
					if (errno == EWOULDBLOCK)
					{
						throw std::runtime_error(
							"another Gateway reconfiguration transaction is active");
					}
					throw_errno("acquire transaction lock");
				}
			}

		private:
			OwnedFileDescriptor descriptor_;
		};

		struct CommandLine
		{
			std::filesystem::path candidate_profile_path;
			std::filesystem::path candidate_config_root;
			std::string expected_current_hash;
			std::filesystem::path result_path;
			std::filesystem::path last_known_good_path;
			std::filesystem::path lock_path{kDefaultLockPath};
			std::string gateway_fqn{kDefaultGatewayFqn};
			std::string transaction_id;
			std::chrono::milliseconds timeout{15s};
		};

		void print_usage(const char *program)
		{
			std::cout
				<< "Usage: " << program << " --candidate-profile PATH"
				<< " --candidate-config-root PATH --expected-current-hash HASH"
				<< " --result PATH --last-known-good PATH"
				<< " [--gateway-fqn FQN] [--transaction-id ID]"
				<< " [--lock-file PATH] [--timeout-ms N]\n";
		}

		std::uint64_t parse_positive_u64(
			const std::string &value,
			const char *name)
		{
			std::size_t consumed = 0U;
			std::uint64_t parsed = 0U;
			try
			{
				parsed = std::stoull(value, &consumed, 10);
			}
			catch (const std::exception &)
			{
				throw std::invalid_argument(
					std::string{name} + " must be a positive integer");
			}
			if (consumed != value.size() || parsed == 0U)
			{
				throw std::invalid_argument(
					std::string{name} + " must be a positive integer");
			}
			return parsed;
		}

		std::string make_transaction_id()
		{
			const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::system_clock::now().time_since_epoch()).count();
			return "tx-" + std::to_string(now) + "-" + std::to_string(::getpid());
		}

		CommandLine parse_command_line(const std::vector<std::string> &arguments)
		{
			if (arguments.empty())
			{
				throw std::invalid_argument("missing program name");
			}

			std::map<std::string, std::string> options;
			for (std::size_t index = 1U; index < arguments.size(); ++index)
			{
				const auto &name = arguments[index];
				if (name == "--help")
				{
					print_usage(arguments.front().c_str());
					throw std::runtime_error("help requested");
				}
				if (name.rfind("--", 0U) != 0U || index + 1U >= arguments.size())
				{
					throw std::invalid_argument(
						"expected --name value command-line pairs");
				}
				if (!options.emplace(name, arguments[++index]).second)
				{
					throw std::invalid_argument("duplicate command-line option: " + name);
				}
			}

			auto require = [&options](const char *name) -> std::string
			{
				const auto iterator = options.find(name);
				if (iterator == options.end() || iterator->second.empty())
				{
					throw std::invalid_argument(
						std::string{"missing required option: "} + name);
				}
				return iterator->second;
			};

			const std::vector<std::string> known{
				"--candidate-profile",
				"--candidate-config-root",
				"--expected-current-hash",
				"--result",
				"--last-known-good",
				"--gateway-fqn",
				"--transaction-id",
				"--lock-file",
				"--timeout-ms"};
			for (const auto &[name, value] : options)
			{
				(void)value;
				if (std::find(known.begin(), known.end(), name) == known.end())
				{
					throw std::invalid_argument("unknown command-line option: " + name);
				}
			}

			CommandLine result;
			result.candidate_profile_path = require("--candidate-profile");
			result.candidate_config_root = require("--candidate-config-root");
			result.expected_current_hash = require("--expected-current-hash");
			result.result_path = require("--result");
			result.last_known_good_path = require("--last-known-good");
			if (const auto iterator = options.find("--gateway-fqn");
				iterator != options.end())
			{
				result.gateway_fqn = iterator->second;
			}
			if (const auto iterator = options.find("--transaction-id");
				iterator != options.end())
			{
				result.transaction_id = iterator->second;
			}
			if (const auto iterator = options.find("--lock-file");
				iterator != options.end())
			{
				result.lock_path = iterator->second;
			}
			if (const auto iterator = options.find("--timeout-ms");
				iterator != options.end())
			{
				const auto value = parse_positive_u64(iterator->second, "--timeout-ms");
				if (value > static_cast<std::uint64_t>(
					std::chrono::milliseconds::max().count()))
				{
					throw std::invalid_argument("--timeout-ms is too large");
				}
				result.timeout = std::chrono::milliseconds{value};
			}
			if (result.transaction_id.empty())
			{
				result.transaction_id = make_transaction_id();
			}
			if (result.expected_current_hash.size() != 64U ||
				!std::all_of(
					result.expected_current_hash.begin(),
					result.expected_current_hash.end(),
					[](unsigned char character)
					{
						return std::isxdigit(character) != 0;
					}))
			{
				throw std::invalid_argument(
					"--expected-current-hash must be a 64-character hexadecimal SHA-256");
			}
			if (result.transaction_id.size() > 128U ||
				!std::all_of(
					result.transaction_id.begin(),
					result.transaction_id.end(),
					[](unsigned char character)
					{
						return std::isalnum(character) != 0 || character == '-' ||
							character == '_';
					}))
			{
				throw std::invalid_argument(
					"--transaction-id must contain only letters, digits, '-' or '_'");
			}
			if (result.gateway_fqn.empty() || result.gateway_fqn.front() != '/')
			{
				throw std::invalid_argument("--gateway-fqn must be an absolute ROS name");
			}
			return result;
		}

		std::optional<std::string> diagnostic_value(
			const DiagnosticStatus &status,
			const std::string &key)
		{
			for (const auto &value : status.values)
			{
				if (value.key == key)
				{
					return value.value;
				}
			}
			return std::nullopt;
		}

		struct GatewayDiagnosticSnapshot
		{
			std::uint64_t revision{0U};
			std::string decision_config_hash;
			std::string fastdds_profile_hash;
			std::optional<bool> endpoint_identity_healthy;
			std::optional<bool> endpoint_qos_compatible;
		};

		struct GatewayParameters
		{
			std::filesystem::path profile_path;
			std::filesystem::path config_root;
			std::filesystem::path decision_trace_path;
		};

		std::optional<bool> parse_diagnostic_bool(
			const DiagnosticStatus &status,
			const char *key)
		{
			const auto value = diagnostic_value(status, key);
			if (value == "true")
			{
				return true;
			}
			if (value == "false")
			{
				return false;
			}
			return std::nullopt;
		}

		class GatewayRuntimeClient
		{
		public:
			GatewayRuntimeClient(
				std::shared_ptr<rclcpp::Node> node,
				std::string gateway_fqn,
				std::chrono::milliseconds timeout)
				: node_(std::move(node)),
				  gateway_fqn_(std::move(gateway_fqn)),
				  timeout_(timeout)
			{
				executor_.add_node(node_);
				diagnostics_subscription_ = node_->create_subscription<DiagnosticArray>(
					"/diagnostics",
					rclcpp::QoS(rclcpp::KeepLast(20U)),
					[this](DiagnosticArray::ConstSharedPtr message)
					{
						GatewayDiagnosticSnapshot snapshot;
						bool gateway_status_observed = false;
						for (const auto &status : message->status)
						{
							if (status.hardware_id != "control_link/gateway")
							{
								continue;
							}
							if (status.name == "gateway/config")
							{
								gateway_status_observed = true;
								snapshot.decision_config_hash =
									diagnostic_value(status, "decision_config_hash")
										.value_or("");
								snapshot.fastdds_profile_hash =
									diagnostic_value(status, "fastdds_profile_hash")
										.value_or("");
							}
							else if (status.name == "gateway/endpoints")
							{
								gateway_status_observed = true;
								snapshot.endpoint_identity_healthy =
									parse_diagnostic_bool(status, "identity_healthy");
								snapshot.endpoint_qos_compatible =
									parse_diagnostic_bool(status, "qos_compatible");
							}
						}
						if (gateway_status_observed)
						{
							snapshot.revision = ++diagnostic_revision_;
							latest_diagnostics_ = std::move(snapshot);
						}
					});
				get_state_client_ = node_->create_client<GetState>(
					gateway_fqn_ + "/get_state");
				change_state_client_ = node_->create_client<ChangeState>(
					gateway_fqn_ + "/change_state");
				parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
					node_, gateway_fqn_);
			}

			void wait_for_services()
			{
				const auto deadline = std::chrono::steady_clock::now() + timeout_;
				while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
				{
					if (get_state_client_->service_is_ready() &&
						change_state_client_->service_is_ready() &&
						parameter_client_->service_is_ready())
					{
						return;
					}
					executor_.spin_once(50ms);
				}
				throw std::runtime_error("Gateway Lifecycle or parameter services are unavailable");
			}

			GatewayParameters read_parameters()
			{
				auto future = parameter_client_->get_parameters({
					"profile_path", "config_root", "decision_trace_path"});
				wait_for_future(future, "read Gateway parameters");
				const auto parameters = future.get();
				if (parameters.size() != 3U)
				{
					throw std::runtime_error("Gateway parameter response has an unexpected size");
				}
				GatewayParameters result{
					parameters[0].as_string(),
					parameters[1].as_string(),
					parameters[2].as_string()};
				if (result.profile_path.empty() || result.config_root.empty())
				{
					throw std::runtime_error("Gateway active configuration paths are empty");
				}
				return result;
			}

			void set_parameters(const GatewayParameters &parameters)
			{
				auto future = parameter_client_->set_parameters({
					rclcpp::Parameter("profile_path", parameters.profile_path.string()),
					rclcpp::Parameter("config_root", parameters.config_root.string()),
					rclcpp::Parameter(
						"decision_trace_path", parameters.decision_trace_path.string())});
				wait_for_future(future, "set Gateway parameters");
				const auto results = future.get();
				if (results.size() != 3U)
				{
					throw std::runtime_error("Gateway set-parameter response has an unexpected size");
				}
				for (std::size_t index = 0U; index < results.size(); ++index)
				{
					if (!results[index].successful)
					{
						throw std::runtime_error(
							"Gateway rejected parameter index " + std::to_string(index) +
							": " + results[index].reason);
					}
				}
			}

			[[nodiscard]] std::uint8_t read_state()
			{
				auto future = get_state_client_->async_send_request(
					std::make_shared<GetState::Request>());
				wait_for_future(future, "read Gateway Lifecycle state");
				return future.get()->current_state.id;
			}

			void require_state(std::uint8_t expected, const char *context)
			{
				const auto actual = read_state();
				if (actual != expected)
				{
					throw std::runtime_error(
						std::string{context} + " Lifecycle state mismatch: expected=" +
						std::to_string(expected) + ", actual=" + std::to_string(actual));
				}
			}

			void transition(
				std::uint8_t transition_id,
				std::uint8_t expected_state,
				const char *context)
			{
				auto request = std::make_shared<ChangeState::Request>();
				request->transition.id = transition_id;
				auto future = change_state_client_->async_send_request(request);
				wait_for_future(future, context);
				if (!future.get()->success)
				{
					throw std::runtime_error(
						std::string{context} + " Lifecycle transition was rejected");
				}
				require_state(expected_state, context);
			}

			void activate_with_retry(const char *context)
			{
				const auto deadline = std::chrono::steady_clock::now() + timeout_;
				while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
				{
					require_state(
						lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
						context);
					auto request = std::make_shared<ChangeState::Request>();
					request->transition.id =
						lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
					auto future = change_state_client_->async_send_request(request);
					const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
						deadline - std::chrono::steady_clock::now());
					if (remaining <= 0ms ||
						executor_.spin_until_future_complete(future, remaining) !=
							rclcpp::FutureReturnCode::SUCCESS)
					{
						throw std::runtime_error(
							std::string{context} + " Lifecycle response timed out");
					}
					if (future.get()->success)
					{
						require_state(
							lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
							context);
						return;
					}
					// Graph 稳定后 VehicleState 新订阅仍可能尚未收到首个合法样本
					std::this_thread::sleep_for(50ms);
				}
				throw std::runtime_error(
					std::string{context} + " Lifecycle activation timed out");
			}

			void mark_verification_floor() noexcept
			{
				verification_floor_ = diagnostic_revision_;
			}

			[[nodiscard]] std::string wait_for_any_config_hash()
			{
				wait_until(
					[this]()
					{
						return latest_diagnostics_.has_value() &&
							!latest_diagnostics_->decision_config_hash.empty() &&
							!latest_diagnostics_->fastdds_profile_hash.empty();
					},
					"Gateway diagnostics did not expose decision_config_hash");
				return latest_diagnostics_->decision_config_hash;
			}

			[[nodiscard]] std::string current_fastdds_profile_hash()
			{
				(void)wait_for_any_config_hash();
				return latest_diagnostics_->fastdds_profile_hash;
			}

			void wait_for_configuration(
				const std::string &expected_hash,
				const std::string &expected_fastdds_hash,
				const char *context)
			{
				wait_until(
					[this, &expected_hash, &expected_fastdds_hash]()
					{
						return latest_diagnostics_.has_value() &&
							latest_diagnostics_->revision > verification_floor_ &&
							latest_diagnostics_->decision_config_hash == expected_hash &&
							latest_diagnostics_->fastdds_profile_hash ==
								expected_fastdds_hash &&
							latest_diagnostics_->endpoint_identity_healthy == true &&
							latest_diagnostics_->endpoint_qos_compatible == true;
					},
					context);
			}

		private:
			template<typename Future>
			void wait_for_future(Future &future, const char *context)
			{
				if (executor_.spin_until_future_complete(future, timeout_) !=
					rclcpp::FutureReturnCode::SUCCESS)
				{
					throw std::runtime_error(std::string{context} + " timed out");
				}
			}

			template<typename Predicate>
			void wait_until(Predicate predicate, const char *context)
			{
				const auto deadline = std::chrono::steady_clock::now() + timeout_;
				while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
				{
					executor_.spin_some();
					if (predicate())
					{
						return;
					}
					executor_.spin_once(50ms);
				}

				std::string detail;
				if (latest_diagnostics_.has_value())
				{
					detail = "; latest_hash=" +
						latest_diagnostics_->decision_config_hash +
						", latest_fastdds_hash=" +
						latest_diagnostics_->fastdds_profile_hash +
						", identity_healthy=" +
						(latest_diagnostics_->endpoint_identity_healthy == true ?
							"true" : "false") +
						", qos_compatible=" +
						(latest_diagnostics_->endpoint_qos_compatible == true ?
							"true" : "false");
				}
				throw std::runtime_error(std::string{context} + " timed out" + detail);
			}

			std::shared_ptr<rclcpp::Node> node_;
			std::string gateway_fqn_;
			std::chrono::milliseconds timeout_;
			rclcpp::executors::SingleThreadedExecutor executor_;
			rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription_;
			rclcpp::Client<GetState>::SharedPtr get_state_client_;
			rclcpp::Client<ChangeState>::SharedPtr change_state_client_;
			std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
			std::uint64_t diagnostic_revision_{0U};
			std::uint64_t verification_floor_{0U};
			std::optional<GatewayDiagnosticSnapshot> latest_diagnostics_;
		};

		std::filesystem::path derived_trace_path(
			const std::filesystem::path &previous,
			const std::string &transaction_id,
			const char *phase)
		{
			if (previous.empty())
			{
				return {};
			}
			return previous.parent_path() /
				(previous.stem().string() + "." + transaction_id + "." + phase +
				 previous.extension().string());
		}

		class RosGatewayReconfigurationBackend final : public ReconfigurationBackend
		{
		public:
			RosGatewayReconfigurationBackend(
				GatewayRuntimeClient &runtime,
				GatewayParameters previous,
				GatewayParameters candidate,
				std::filesystem::path rollback_trace_path,
				control_link_contract::RuntimeConfigIdentity previous_identity,
				control_link_contract::RuntimeConfigIdentity candidate_identity,
				std::string previous_fastdds_hash,
				std::string candidate_fastdds_hash,
				std::filesystem::path last_known_good_path,
				std::string transaction_id)
				: runtime_(runtime),
				  previous_(std::move(previous)),
				  candidate_(std::move(candidate)),
				  rollback_trace_path_(std::move(rollback_trace_path)),
				  previous_identity_(std::move(previous_identity)),
				  candidate_identity_(std::move(candidate_identity)),
				  previous_fastdds_hash_(std::move(previous_fastdds_hash)),
				  candidate_fastdds_hash_(std::move(candidate_fastdds_hash)),
				  last_known_good_path_(std::move(last_known_good_path)),
				  transaction_id_(std::move(transaction_id))
			{
			}

			std::string current_decision_config_hash() override
			{
				return runtime_.wait_for_any_config_hash();
			}

			void deactivate_current() override
			{
				runtime_.require_state(
					lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
					"deactivate current precondition");
				runtime_.transition(
					lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
					lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
					"deactivate current");
			}

			void cleanup_current() override
			{
				runtime_.transition(
					lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
					lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
					"cleanup current");
			}

			void set_candidate_config() override
			{
				runtime_.set_parameters(candidate_);
			}

			void configure_candidate() override
			{
				runtime_.mark_verification_floor();
				runtime_.transition(
					lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
					lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
					"configure candidate");
			}

			void verify_candidate() override
			{
				runtime_.wait_for_configuration(
					candidate_identity_.decision_config.decision_config_hash,
					candidate_fastdds_hash_,
					"verify candidate config and Graph");
			}

			void activate_candidate() override
			{
				runtime_.activate_with_retry("activate candidate");
			}

			void commit_candidate() override
			{
				const nlohmann::json descriptor{
					{"schema_version", 1U},
					{"transaction_id", transaction_id_},
					{"profile_path", candidate_.profile_path.string()},
					{"config_root", candidate_.config_root.string()},
					{"contract_id", candidate_identity_.contract.contract_id},
					{"contract_version", candidate_identity_.contract.contract_version},
					{"contract_hash", candidate_identity_.contract.contract_hash},
					{"decision_config_hash",
					 candidate_identity_.decision_config.decision_config_hash},
					{"fastdds_profile_hash", candidate_fastdds_hash_}};
				atomic_write_text(last_known_good_path_, descriptor.dump(2) + "\n");
			}

			void cleanup_candidate() override
			{
				const auto state = runtime_.read_state();
				if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
				{
					runtime_.transition(
						lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
						lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
						"deactivate partial candidate");
				}
				const auto quiesced_state = runtime_.read_state();
				if (quiesced_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
				{
					runtime_.transition(
						lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
						lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
						"cleanup partial candidate");
				}
				else if (quiesced_state !=
					lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
				{
					throw std::runtime_error(
						"partial candidate is not cleanup-capable: state=" +
						std::to_string(quiesced_state));
				}
			}

			void restore_previous_config() override
			{
				auto rollback = previous_;
				rollback.decision_trace_path = rollback_trace_path_;
				runtime_.set_parameters(rollback);
			}

			void configure_previous() override
			{
				runtime_.mark_verification_floor();
				runtime_.transition(
					lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
					lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
					"configure previous");
			}

			void verify_previous() override
			{
				runtime_.wait_for_configuration(
					previous_identity_.decision_config.decision_config_hash,
					previous_fastdds_hash_,
					"verify previous config and Graph");
			}

			void activate_previous() override
			{
				runtime_.activate_with_retry("activate previous");
			}

		private:
			GatewayRuntimeClient &runtime_;
			GatewayParameters previous_;
			GatewayParameters candidate_;
			std::filesystem::path rollback_trace_path_;
			control_link_contract::RuntimeConfigIdentity previous_identity_;
			control_link_contract::RuntimeConfigIdentity candidate_identity_;
			std::string previous_fastdds_hash_;
			std::string candidate_fastdds_hash_;
			std::filesystem::path last_known_good_path_;
			std::string transaction_id_;
		};

		int status_exit_code(ReconfigurationStatus status) noexcept
		{
			switch (status)
			{
			case ReconfigurationStatus::kNoOp:
			case ReconfigurationStatus::kCommitted:
				return 0;
			case ReconfigurationStatus::kRejected:
				return 2;
			case ReconfigurationStatus::kRolledBack:
				return 3;
			case ReconfigurationStatus::kRollbackFailed:
				return 4;
			}
			return 4;
		}

		std::string invalid_manifest(
			const std::string &transaction_id,
			const std::string &message)
		{
			return nlohmann::json{
				{"schema_version", 1U},
				{"transaction_id", transaction_id},
				{"status", "INVALID"},
				{"message", message}}.dump(2) + "\n";
		}
	}  // namespace
}  // namespace control_link_bringup

int main(int argc, char **argv)
{
	using namespace control_link_bringup;

	std::optional<CommandLine> command_line;
	std::unique_ptr<ExclusiveTransactionLock> transaction_lock;
	bool rclcpp_initialized = false;
	try
	{
		rclcpp::init(argc, argv);
		rclcpp_initialized = true;
		command_line = parse_command_line(rclcpp::remove_ros_arguments(argc, argv));
		command_line->candidate_profile_path = canonical_input_path(
			command_line->candidate_profile_path, "--candidate-profile", false);
		command_line->candidate_config_root = canonical_input_path(
			command_line->candidate_config_root, "--candidate-config-root", true);
		require_absolute_output_path(command_line->result_path, "--result");
		require_absolute_output_path(
			command_line->last_known_good_path, "--last-known-good");
		require_absolute_output_path(command_line->lock_path, "--lock-file");
		const auto result_output = normalized_output_path(command_line->result_path);
		const auto last_known_good_output = normalized_output_path(
			command_line->last_known_good_path);
		const auto lock_output = normalized_output_path(command_line->lock_path);
		if (result_output == last_known_good_output || result_output == lock_output ||
			last_known_good_output == lock_output)
		{
			throw std::invalid_argument(
				"--result, --last-known-good and --lock-file must be distinct paths");
		}
		transaction_lock = std::make_unique<ExclusiveTransactionLock>(
			command_line->lock_path);
		atomic_write_text(
			command_line->result_path,
			invalid_manifest(
				command_line->transaction_id,
				"transaction started but no final result was committed"));

		auto node = std::make_shared<rclcpp::Node>("gateway_reconfiguration_cli");
		GatewayRuntimeClient runtime{
			node,
			command_line->gateway_fqn,
			command_line->timeout};
		runtime.wait_for_services();
		runtime.require_state(
			lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
			"transaction precondition");

		const auto previous_parameters = runtime.read_parameters();
		auto current = control_link_contract::load_contract_bundle(
			previous_parameters.profile_path,
			previous_parameters.config_root);
		auto candidate = control_link_contract::load_contract_bundle(
			command_line->candidate_profile_path,
			command_line->candidate_config_root);

		const auto runtime_hash = runtime.wait_for_any_config_hash();
		if (runtime_hash != current->identity.decision_config.decision_config_hash)
		{
			throw std::runtime_error(
				"runtime diagnostics hash differs from the current parameter Bundle: "
				"runtime=" + runtime_hash + ", bundle=" +
				current->identity.decision_config.decision_config_hash);
		}
		const auto runtime_fastdds_hash = runtime.current_fastdds_profile_hash();
		if (runtime_fastdds_hash != current->fastdds_profile_hash)
		{
			throw std::runtime_error(
				"runtime FastDDS hash differs from the current parameter Bundle: "
				"runtime=" + runtime_fastdds_hash + ", bundle=" +
				current->fastdds_profile_hash);
		}

		auto plan = control_link_contract::build_reconfiguration_plan(
			current,
			candidate,
			command_line->expected_current_hash);
		GatewayParameters candidate_parameters{
			command_line->candidate_profile_path,
			command_line->candidate_config_root,
			derived_trace_path(
				previous_parameters.decision_trace_path,
				command_line->transaction_id,
				"candidate")};
		const auto rollback_trace_path = derived_trace_path(
			previous_parameters.decision_trace_path,
			command_line->transaction_id,
			"rollback");
		RosGatewayReconfigurationBackend backend{
			runtime,
			previous_parameters,
			std::move(candidate_parameters),
			rollback_trace_path,
			plan.diff.current_identity,
			plan.diff.candidate_identity,
			plan.current->fastdds_profile_hash,
			plan.candidate->fastdds_profile_hash,
			command_line->last_known_good_path,
			command_line->transaction_id};

		const auto result = execute_reconfiguration(
			plan,
			command_line->transaction_id,
			backend);
		atomic_write_text(
			command_line->result_path,
			serialize_reconfiguration_result(result) + "\n");
		std::cout << serialize_reconfiguration_result(result) << '\n';
		const auto exit_code = status_exit_code(result.status);
		rclcpp::shutdown();
		return exit_code;
	}
	catch (const std::exception &exception)
	{
		if (command_line.has_value())
		{
			try
			{
				ReconfigurationResult rejected;
				rejected.transaction_id = command_line->transaction_id;
				rejected.message = exception.what();
				atomic_write_text(
					command_line->result_path,
					serialize_reconfiguration_result(rejected) + "\n");
			}
			catch (const std::exception &write_error)
			{
				std::cerr << "failed to write rejection result: "
					<< write_error.what() << '\n';
			}
		}
		std::cerr << "Gateway reconfiguration failed: " << exception.what() << '\n';
		if (rclcpp_initialized && rclcpp::ok())
		{
			rclcpp::shutdown();
		}
		return 2;
	}
	catch (...)
	{
		std::cerr << "Gateway reconfiguration failed with an unknown exception\n";
		if (rclcpp_initialized && rclcpp::ok())
		{
			rclcpp::shutdown();
		}
		return 2;
	}
}
