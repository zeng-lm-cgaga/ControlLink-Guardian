#include "control_link_gateway/source_runtime.hpp"

#include <stdexcept>
#include <utility>

namespace control_link_gateway
{
	void commit_validation_result(
		SourceRuntimeSlot &slot,
		const CommandValidationResult &result)
	{
		const bool accepted_reason = result.reason == RejectReason::kNone;
		const bool has_snapshot = result.snapshot.has_value();
		if (accepted_reason != has_snapshot)
		{
			throw std::logic_error(
				"CommandValidationResult reason and snapshot are inconsistent");
		}

		if (!accepted_reason)
		{
			slot.last_reject_reason = result.reason;
			slot.rejected_count += 1;
			return;
		}

		if (!slot.confirmed_publisher_generation.has_value())
		{
			throw std::logic_error(
				"accepted command has no confirmed publisher generation");
		}

		const auto &result_snapshot = result.snapshot.value();
		if (!(slot.confirmed_publisher_generation.value() ==
			result_snapshot.publisher_generation))
		{
			throw std::logic_error(
				"accepted command generation does not match the confirmed publisher generation");
		}

		if (slot.last_accepted_sequence.has_value() &&
			result_snapshot.sequence <= slot.last_accepted_sequence.value())
		{
			throw std::logic_error(
				"accepted command sequence does not increase the source baseline");
		}

		// 先完成可能抛异常的复制，确保失败时 Slot 不会处于半提交状态
		auto next_snapshot = result_snapshot;
		const auto next_sequence = next_snapshot.sequence;

		slot.latest_valid_snapshot = std::move(next_snapshot);
		slot.last_accepted_sequence = next_sequence;
		slot.accepted_count += 1;
	}

	std::map<std::string, SourceSnapshot>
	collect_latest_valid_snapshots(
		const std::map<std::string, SourceRuntimeSlot> &slots)
	{
		std::map<std::string, SourceSnapshot> snapshots;

		for (const auto &[source_id, slot] : slots)
		{
			if (!slot.latest_valid_snapshot.has_value())
			{
				continue;
			}

			const auto &snapshot = slot.latest_valid_snapshot.value();
			if (snapshot.source_id != source_id)
			{
				throw std::logic_error(
					"source slot key does not match latest-valid snapshot source_id: key=" +
					source_id + ", snapshot=" + snapshot.source_id);
			}

			if (!slot.confirmed_publisher_generation.has_value())
			{
				throw std::logic_error(
					"source slot has a latest-valid snapshot without a confirmed publisher generation: " +
					source_id);
			}

			if (!(snapshot.publisher_generation ==
				slot.confirmed_publisher_generation.value()))
			{
				throw std::logic_error(
					"source slot latest-valid snapshot generation does not match the confirmed generation: " +
					source_id);
			}

			if (!slot.last_accepted_sequence.has_value())
			{
				throw std::logic_error(
					"source slot has a latest-valid snapshot without a sequence baseline: " +
					source_id);
			}

			if (slot.last_accepted_sequence.value() != snapshot.sequence)
			{
				throw std::logic_error(
					"source slot sequence baseline does not match the latest-valid snapshot: " +
					source_id);
			}

			const auto [iterator, inserted] = snapshots.emplace(source_id, snapshot);
			(void)iterator;
			if (!inserted)
			{
				throw std::logic_error(
					"duplicate source reached the latest-valid snapshot collection: " +
					source_id);
			}
		}

		return snapshots;
	}
} // namespace control_link_gateway
