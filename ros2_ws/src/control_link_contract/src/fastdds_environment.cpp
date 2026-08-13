#include "control_link_contract/fastdds_environment.hpp"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <variant>

#include "rmw/rmw.h"

namespace control_link_contract
{
	namespace
	{
		const std::filesystem::path &profile_fastdds_path(
			const ProfileConfig &profile)
		{
			return std::visit(
				[](const auto &typed_profile) -> const std::filesystem::path &
				{
					return typed_profile.common.fastdds_profile_path;
				},
				profile);
		}
	} // namespace

	std::string validate_fastdds_process_environment(
		const ProfileConfig &profile,
		std::string_view component_name)
	{
		const std::string component = component_name.empty() ?
			"ControlLink component" : std::string(component_name);
		const char *rmw_identifier = rmw_get_implementation_identifier();
		if (rmw_identifier == nullptr || rmw_identifier[0] == '\0')
		{
			throw std::runtime_error(
				component + " cannot determine the active RMW implementation");
		}

		const std::string rmw_implementation{rmw_identifier};
		if (rmw_implementation != "rmw_fastrtps_cpp")
		{
			throw std::runtime_error(
				component + " Profile requires rmw_fastrtps_cpp; actual=" +
				rmw_implementation);
		}

		const char *qos_from_xml = std::getenv("RMW_FASTRTPS_USE_QOS_FROM_XML");
		if (qos_from_xml == nullptr || std::string(qos_from_xml) != "0")
		{
			throw std::runtime_error(
				"RMW_FASTRTPS_USE_QOS_FROM_XML must be 0 so Contract remains the Topic QoS owner");
		}

		const char *profile_environment =
			std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE");
		if (profile_environment == nullptr || profile_environment[0] == '\0')
		{
			throw std::runtime_error(
				"FASTRTPS_DEFAULT_PROFILES_FILE is not set before " +
				component + " startup");
		}

		const std::filesystem::path environment_path{profile_environment};
		if (!environment_path.is_absolute())
		{
			throw std::runtime_error(
				"FASTRTPS_DEFAULT_PROFILES_FILE must be an absolute path; actual=" +
				environment_path.string());
		}

		std::error_code error;
		const auto canonical_environment_path =
			std::filesystem::canonical(environment_path, error);
		if (error)
		{
			throw std::runtime_error(
				"FASTRTPS_DEFAULT_PROFILES_FILE cannot be canonicalized; actual=" +
				environment_path.string() + " (" + error.message() + ")");
		}

		const auto &expected_path = profile_fastdds_path(profile);
		if (canonical_environment_path != expected_path)
		{
			throw std::runtime_error(
				"FastDDS participant profile mismatch: expected=" +
				expected_path.string() + ", actual=" +
				canonical_environment_path.string());
		}

		return rmw_implementation;
	}
} // namespace control_link_contract
