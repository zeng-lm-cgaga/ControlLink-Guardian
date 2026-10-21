#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

namespace control_link_timing
{
	// 记录 CLOCK_MONOTONIC 纳秒，与 Linux bpf_ktime_get_ns 使用相同时基。
	std::uint64_t monotonic_ns() noexcept;
	std::uint32_t thread_id() noexcept;

	enum class Phase : std::uint8_t
	{
		SourceReceive, SourceResult, SourceSelect, CanonicalPublish,
		MailboxWrite, MailboxConsume, CanWrite, CanReceive,
		FeedbackValidate, FeedbackPublish, GatewayFeedback, PeriodicTick,
		StopRequested, WorkerExit, WorkerJoined, ResourcesClosed, SourceSnapshot,
		WorkCpu, WorkSleep, WorkLock, WorkHold
	};

	// GID 是本次实际 ROS publisher 实例，不冒充认证身份或网络 session。
	// canonical 不携带上游 GID；分析时必须经 Gateway 的选择记录关联。
	struct Identity final
	{
		std::array<char, 96> source{};
		std::array<std::uint8_t, 24> publisher_gid{};
		std::uint64_t sequence{0};
		std::int64_t source_stamp_ns{0};
		bool valid{true};
		bool set_source(std::string_view value) noexcept;
	};

	struct Event final
	{
		Phase phase{Phase::SourceReceive};
		Identity identity{};
		std::uint64_t begin_ns{0};
		std::uint64_t end_ns{0};
		std::uint64_t scheduled_ns{0};
		// 1=CLOCK_MONOTONIC；2=RCL_STEADY_TIME。后者可能是 RAW，严禁与 begin_ns 直接相减。
		std::uint8_t schedule_clock{1};
		std::uint64_t schedule_now_ns{0};
		// 同一 worker 的 frame_id 标识一次生成/发送尝试，CAN counter 只作局部关联。
		std::uint64_t frame_id{0};
		std::int64_t result{0};
		std::int64_t value{0};
		std::uint32_t tid{0};
	};

	struct RecorderStats final
	{
		std::uint64_t queued, written, full, contended, invalid_identity, closed, io_errors;
	};

	// 数据面只 try_lock + 复制定长事件，不分配、不等待文件系统；后台每 20ms 批量写。
	// owner 必须先停止所有生产者，再 close/drain/join。文件独占创建，禁止覆盖证据。
	class Recorder final
	{
	public:
		static constexpr std::size_t capacity = 4096;
		explicit Recorder(const std::string &absolute_path);
		~Recorder();
		Recorder(const Recorder &) = delete;
		Recorder &operator=(const Recorder &) = delete;
		bool record(Event event) noexcept;
		void close() noexcept;
		RecorderStats stats() const noexcept;
	private:
		void run() noexcept;
		bool write_bytes(const std::string &bytes) noexcept;
		void finalize() noexcept;
		int fd_{-1};
		int status_fd_{-1};
		std::string path_;
		bool finalized_{false};
		std::uint32_t pid_{0};
		std::unique_ptr<std::array<Event, capacity>> queue_;
		std::mutex queue_mutex_, close_mutex_;
		std::condition_variable wake_;
		std::size_t head_{0}, size_{0};
		std::atomic<bool> closing_{false};
		std::thread writer_;
		std::atomic<std::uint64_t> queued_{0}, written_{0}, full_{0}, contended_{0};
		std::atomic<std::uint64_t> invalid_{0}, closed_{0}, io_errors_{0};
	};

	// 空路径默认关闭；开启失败显式抛错，不能悄悄声称已采样。
	std::shared_ptr<Recorder> open_optional(const std::string &absolute_path);
}
