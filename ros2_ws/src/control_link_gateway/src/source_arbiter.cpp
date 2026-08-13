#include "control_link_gateway/source_arbiter.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace control_link_gateway
{
	namespace
	{
		constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;

		bool exceeds_milliseconds(
			std::uint64_t duration_ns,
			std::uint64_t limit_ms) noexcept
		{
			const auto max_value = std::numeric_limits<std::uint64_t>::max();
			if (limit_ms > max_value / kNanosecondsPerMillisecond)
			{
				// 阈值大于 duration_ns 的可表示范围，任何输入 duration 都无法超过它
				return false;
			}

			const auto limit_ns = limit_ms * kNanosecondsPerMillisecond;
			return duration_ns > limit_ns;
		}

		const control_link_contract::SourcePolicyEntry &require_policy_entry(
			const control_link_contract::SourcePolicy &policy,
			const std::string &source_id)
		{
			const auto iterator = policy.sources.find(source_id);
			if (iterator == policy.sources.end())
			{
				throw std::logic_error(
					"SourceArbiter snapshot source is absent from SourcePolicy: " + source_id);
			}

			return iterator->second;
		}

		bool command_is_fresh(
			const SourceSnapshot &snapshot,
			std::int64_t now_ros_ns,
			std::uint64_t command_timeout_ms) noexcept
		{
			if (now_ros_ns < 0 || snapshot.source_stamp_ns < 0)
			{
				return false;
			}

			if (snapshot.source_stamp_ns > now_ros_ns)
			{
				// future skew 已由 Validator 限制，Arbiter 只等待 ROS time 追上合法快照
				return true;
			}

			const auto age_ns = static_cast<std::uint64_t>(
				now_ros_ns - snapshot.source_stamp_ns);
			return !exceeds_milliseconds(age_ns, command_timeout_ms);
		}

		bool lease_is_valid(
			const SourceSnapshot &snapshot,
			std::chrono::steady_clock::time_point now_steady,
			std::uint64_t lease_timeout_ms)
		{
			if (now_steady < snapshot.received_at)
			{
				throw std::logic_error(
					"SourceArbiter steady clock precedes snapshot receive time");
			}

			using Milliseconds = std::chrono::milliseconds;
			const auto lease_timeout = Milliseconds{
				static_cast<Milliseconds::rep>(lease_timeout_ms)};
			return now_steady - snapshot.received_at <= lease_timeout;
		}

		const control_link_contract::SourcePolicyEntry &require_snapshot_policy(
			const std::string &map_source_id,
			const SourceSnapshot &snapshot,
			const control_link_contract::SourcePolicy &policy)
		{
			// 快照身份与优先级必须和配置 owner 一致，不一致属于内部不变量错误
			if (map_source_id != snapshot.source_id)
			{
				throw std::logic_error(
					"SourceArbiter snapshot key does not match source_id: key=" +
					map_source_id + ", snapshot=" + snapshot.source_id);
			}

			const auto &entry = require_policy_entry(policy, snapshot.source_id);
			if (snapshot.priority != entry.priority)
			{
				throw std::logic_error(
					"SourceArbiter snapshot priority does not match SourcePolicy: source=" +
					snapshot.source_id + ", expected=" + std::to_string(entry.priority) +
					", actual=" + std::to_string(snapshot.priority));
			}

			return entry;
		}

		std::int64_t command_age_ns(
			const SourceSnapshot &snapshot,
			std::int64_t now_ros_ns) noexcept
		{
			return now_ros_ns - snapshot.source_stamp_ns;
		}

		bool candidate_is_better(
			const SourceSnapshot &left,
			const SourceSnapshot &right,
			std::int64_t now_ros_ns) noexcept
		{
			if (left.priority != right.priority)
			{
				return left.priority > right.priority;
			}

			const auto left_age = command_age_ns(left, now_ros_ns);
			const auto right_age = command_age_ns(right, now_ros_ns);
			if (left_age != right_age)
			{
				return left_age < right_age;
			}

			return left.source_id < right.source_id;
		}

		using CandidateList = std::vector<const SourceSnapshot *>;

		CandidateList collect_qualified_candidates(
			const std::map<std::string, SourceSnapshot> &snapshots,
			const control_link_contract::GatewayContract &contract,
			const control_link_contract::SourcePolicy &policy,
			std::int64_t now_ros_ns,
			std::chrono::steady_clock::time_point now_steady)
		{
			// 指针只在本次 evaluate 内观察输入 map，决策返回前会复制最终快照
			CandidateList candidates;
			candidates.reserve(snapshots.size());

			for (const auto &[source_id, snapshot] : snapshots)
			{
				if (assess_source_snapshot(
						source_id,
						snapshot,
						contract,
						policy,
						now_ros_ns,
						now_steady).qualified())
				{
					candidates.push_back(&snapshot);
				}
			}

			std::sort(
				candidates.begin(),
				candidates.end(),
				[now_ros_ns](const SourceSnapshot *left, const SourceSnapshot *right)
				{
					return candidate_is_better(*left, *right, now_ros_ns);
				});

			return candidates;
		}

		const SourceSnapshot *find_candidate(
			const CandidateList &candidates,
			const std::string &source_id) noexcept
		{
			const auto iterator = std::find_if(
				candidates.begin(),
				candidates.end(),
				[&source_id](const SourceSnapshot *candidate)
				{
					return candidate->source_id == source_id;
				});

			return iterator == candidates.end() ? nullptr : *iterator;
		}

		bool reaches_milliseconds(
			std::chrono::steady_clock::duration elapsed,
			std::uint64_t limit_ms) noexcept
		{
			using Milliseconds = std::chrono::milliseconds;
			const auto max_milliseconds =
				std::numeric_limits<Milliseconds::rep>::max();
			if (limit_ms > static_cast<std::uint64_t>(max_milliseconds))
			{
				// elapsed 的 duration 表示范围不可能达到该无符号阈值
				return false;
			}

			return elapsed >= Milliseconds{
				static_cast<Milliseconds::rep>(limit_ms)};
		}
		} // namespace

	SourceSnapshotAssessment assess_source_snapshot(
		const std::string &map_source_id,
		const SourceSnapshot &snapshot,
		const control_link_contract::GatewayContract &contract,
		const control_link_contract::SourcePolicy &policy,
		std::int64_t now_ros_ns,
		std::chrono::steady_clock::time_point now_steady)
	{
		const auto &entry = require_snapshot_policy(
			map_source_id,
			snapshot,
			policy);
		const bool fresh = command_is_fresh(
			snapshot,
			now_ros_ns,
			contract.gateway.command_timeout_ms);
		const bool lease_valid = lease_is_valid(
			snapshot,
			now_steady,
			entry.lease_timeout_ms);
		const auto age_ns = command_age_ns(snapshot, now_ros_ns);

		return SourceSnapshotAssessment{
			fresh,
			lease_valid,
			age_ns > 0 ? age_ns : 0};
	}

	SourceArbiter::SourceArbiter(
		control_link_contract::GatewayContractPtr contract,
		control_link_contract::SourcePolicyPtr policy)
		: contract_(std::move(contract)),
		  policy_(std::move(policy))
	{
		if (!contract_)
		{
			throw std::invalid_argument(
				"SourceArbiter requires a non-null GatewayContract");
		}

		if (!policy_)
		{
			throw std::invalid_argument(
				"SourceArbiter requires a non-null SourcePolicy");
		}
	}

	ArbitrationDecision SourceArbiter::evaluate(const ArbitrationInput &input)
	{
		if (input.snapshots == nullptr)
		{
			throw std::invalid_argument(
				"SourceArbiter requires a non-null snapshot map");
		}

		auto candidates = collect_qualified_candidates(
			*input.snapshots,
			*contract_,
			*policy_,
			input.now_ros_ns,
			input.now_steady);

		if (candidates.empty())
		{
			const bool lost_active = active_source_id_.has_value();
			active_source_id_.reset();
			challenger_.reset();

			return ArbitrationDecision{
				std::nullopt,
				lost_active ? ArbitrationEvent::kNoQualifiedSource
							: ArbitrationEvent::kNoChange};
		}

		// candidates 包含本 tick 的全部合格来源，排序后的首项才是理论最优来源
		const SourceSnapshot *best_candidate = candidates.front();
		if (!active_source_id_.has_value())
		{
			active_source_id_ = best_candidate->source_id;
			challenger_.reset();
			return ArbitrationDecision{
				*best_candidate,
				ArbitrationEvent::kFirstSelection};
		}

		// active_source_id_ 来自上一次决策，在本 tick 候选中查找是为了重验其资格
		const SourceSnapshot *current_active_candidate = find_candidate(
			candidates, active_source_id_.value());
		if (current_active_candidate == nullptr)
		{
			// active 一旦失效必须立即 fallback，不能继续等待 switch hold
			active_source_id_ = best_candidate->source_id;
			challenger_.reset();
			return ArbitrationDecision{
				*best_candidate,
				ArbitrationEvent::kFallback};
		}

		if (best_candidate->source_id == current_active_candidate->source_id)
		{
			challenger_.reset();
			return ArbitrationDecision{
				*current_active_candidate,
				ArbitrationEvent::kNoChange};
		}

		if (
			!challenger_.has_value() ||
			challenger_->source_id != best_candidate->source_id)
		{
			// challenger 发生变化意味着“持续更优”重新计时
			challenger_ = ChallengerTracking{
				best_candidate->source_id,
				input.now_steady};
		}

		if (input.now_steady < challenger_->better_since)
		{
			throw std::logic_error(
				"SourceArbiter steady clock moved before challenger start time");
		}

		const auto challenger_elapsed =
			input.now_steady - challenger_->better_since;
		if (!reaches_milliseconds(
				challenger_elapsed,
				contract_->gateway.source_switch_hold_ms))
		{
			// active 仍合格时，在 hold 达标前继续输出 active，而不是提前使用 best
			return ArbitrationDecision{
				*current_active_candidate,
				ArbitrationEvent::kNoChange};
		}

		active_source_id_ = best_candidate->source_id;
		challenger_.reset();
		return ArbitrationDecision{
			*best_candidate,
			ArbitrationEvent::kSwitch};
	}
} // namespace control_link_gateway
