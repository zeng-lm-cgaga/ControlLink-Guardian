#include "control_link_gateway/source_binding.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace control_link_gateway
{
	PublisherGenerationKey
	publisher_generation_from_message_info(
		const rclcpp::MessageInfo &message_info)
	{
		const auto rmw_gid = message_info.get_rmw_message_info().publisher_gid;
		if (!rmw_gid.implementation_identifier)
		{
			throw std::invalid_argument("publisher GID is missing RMW implementation identifier");
		}

		PublisherGenerationKey result;
		result.rmw_implementation = rmw_gid.implementation_identifier;

		std::copy_n(
			rmw_gid.data,
			RMW_GID_STORAGE_SIZE,
			result.publisher_gid.begin());

		return result;
	}

	PublisherGenerationUpdate confirm_publisher_generation(
		SourceRuntimeSlot &slot,
		PublisherGenerationKey generation)
	{
		if (!slot.confirmed_publisher_generation.has_value() && !slot.last_accepted_sequence.has_value() && !slot.latest_valid_snapshot.has_value())
		{
			slot.confirmed_publisher_generation = std::move(generation);
			return PublisherGenerationUpdate::kFirstConfirmation;
		}

		if (slot.confirmed_publisher_generation.has_value())
		{
			if (slot.confirmed_publisher_generation == generation)
			{
				return PublisherGenerationUpdate::kUnchanged;
			}
			else
			{
				slot.confirmed_publisher_generation = std::move(generation);
				slot.last_accepted_sequence.reset();
				slot.latest_valid_snapshot.reset();
				return PublisherGenerationUpdate::kChanged;
			}
		}

		throw std::logic_error(
			"source slot has sequence or snapshot without a confirmed publisher generation");
	}

	void invalidate_source_endpoint_snapshot(SourceRuntimeSlot &slot) noexcept
	{
		slot.latest_valid_snapshot.reset();
	}

	bool message_matches_confirmed_generation(
		const SourceRuntimeSlot &slot,
		const PublisherGenerationKey &actual_generation) noexcept
	{
		if(!slot.confirmed_publisher_generation.has_value())
		{
			return false;
		}

		return slot.confirmed_publisher_generation == actual_generation;
	}
} // namespace control_link_gateway
