#include "control_link_timing/recorder.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <fcntl.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace control_link_timing
{
	std::uint64_t monotonic_ns() noexcept
	{
		timespec now{};
		if (::clock_gettime(CLOCK_MONOTONIC, &now) != 0) { return 0; }
		return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
	}

	std::uint32_t thread_id() noexcept
	{
		thread_local const auto tid = static_cast<std::uint32_t>(::syscall(SYS_gettid));
		return tid;
	}

	bool Identity::set_source(std::string_view value) noexcept
	{
		if (value.size() >= source.size() || value.find('\0') != std::string_view::npos)
		{
			valid = false;
			return false;
		}
		source.fill('\0');
		std::copy(value.begin(), value.end(), source.begin());
		return true;
	}

	namespace
	{
		const char *phase_name(Phase phase) noexcept
		{
			constexpr const char *names[]{"source_receive", "source_result", "source_select",
				"canonical_publish", "mailbox_write", "mailbox_consume", "can_write", "can_receive",
				"feedback_validate", "feedback_publish", "gateway_feedback", "periodic_tick",
				"stop_requested", "worker_exit", "worker_joined", "resources_closed", "source_snapshot",
				"work_cpu", "work_sleep", "work_lock", "work_hold"};
			const auto index = static_cast<std::size_t>(phase);
			return index < std::size(names) ? names[index] : "unknown";
		}
		std::string hex(const void *data, std::size_t size)
		{
			constexpr char digits[] = "0123456789abcdef";
			const auto *bytes = static_cast<const unsigned char *>(data);
			std::string output(size * 2, '0');
			for (std::size_t i = 0; i < size; ++i)
			{
				output[2 * i] = digits[bytes[i] >> 4];
				output[2 * i + 1] = digits[bytes[i] & 15];
			}
			return output;
		}
	}

	Recorder::Recorder(const std::string &path)
		: path_(path), pid_(static_cast<std::uint32_t>(::getpid())),
		  queue_(std::make_unique<std::array<Event, capacity>>())
	{
		if (!std::filesystem::path(path).is_absolute())
		{
			throw std::invalid_argument("runtime_timing_path must be absolute");
		}
		fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
		if (fd_ < 0) { throw std::system_error(errno, std::generic_category(), "timing open"); }
		try
		{
			status_fd_ = ::open((path + ".status.partial").c_str(),
				O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
			if (status_fd_ < 0 || std::filesystem::exists(path + ".status.json"))
			{
				throw std::runtime_error("timing status path already exists or cannot be opened");
			}
			if (!write_bytes("# control_link_timing_v2 clock=CLOCK_MONOTONIC schedule_clock=per_event capacity=4096 flush_ms=20 pid=" +
				std::to_string(pid_) + "\nphase,pid,tid,begin_ns,end_ns,scheduled_ns,schedule_clock,schedule_now_ns,source_hex,gid,sequence,source_stamp_ns,frame_id,result,value\n"))
			{
				throw std::runtime_error("timing header write failed");
			}
			writer_ = std::thread(&Recorder::run, this);
		}
		catch (...)
		{
			(void)::close(fd_); fd_ = -1;
			if (status_fd_ >= 0) { (void)::close(status_fd_); status_fd_ = -1; }
			throw;
		}
	}

	Recorder::~Recorder() { close(); }

	bool Recorder::record(Event event) noexcept
	{
		if (!event.identity.valid)
		{
			invalid_.fetch_add(1, std::memory_order_relaxed); return false;
		}
		if (closing_.load(std::memory_order_acquire))
		{
			closed_.fetch_add(1, std::memory_order_relaxed); return false;
		}
		event.tid = thread_id();
		std::unique_lock<std::mutex> lock(queue_mutex_, std::try_to_lock);
		if (!lock.owns_lock()) { contended_.fetch_add(1, std::memory_order_relaxed); return false; }
		if (closing_.load(std::memory_order_acquire))
		{
			closed_.fetch_add(1, std::memory_order_relaxed); return false;
		}
		if (size_ == capacity) { full_.fetch_add(1, std::memory_order_relaxed); return false; }
		(*queue_)[(head_ + size_) % capacity] = event;
		++size_;
		queued_.fetch_add(1, std::memory_order_relaxed);
		return true;
	}

	RecorderStats Recorder::stats() const noexcept
	{
		return {queued_.load(), written_.load(), full_.load(), contended_.load(),
			invalid_.load(), closed_.load(), io_errors_.load()};
	}

	bool Recorder::write_bytes(const std::string &bytes) noexcept
	{
		std::size_t offset = 0;
		while (offset < bytes.size())
		{
			const auto count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
			if (count > 0) { offset += static_cast<std::size_t>(count); }
			else if (count < 0 && errno == EINTR) { continue; }
			else { io_errors_.fetch_add(1); return false; }
		}
		return true;
	}

	void Recorder::run() noexcept
	{
		try
		{
			std::array<Event, 256> batch{};
			while (true)
			{
				std::size_t count;
				{
					std::unique_lock<std::mutex> lock(queue_mutex_);
					if (size_ == 0 && !closing_.load())
					{
						wake_.wait_for(lock, std::chrono::milliseconds(20), [this] { return closing_.load(); });
					}
					count = std::min(size_, batch.size());
					for (std::size_t i = 0; i < count; ++i) { batch[i] = (*queue_)[(head_ + i) % capacity]; }
					head_ = (head_ + count) % capacity;
					size_ -= count;
					if (count == 0 && closing_.load()) { break; }
				}
				std::ostringstream output;
				for (std::size_t i = 0; i < count; ++i)
				{
					const auto &event = batch[i];
					output << phase_name(event.phase) << ',' << pid_ << ',' << event.tid << ','
						<< event.begin_ns << ',' << event.end_ns << ',' << event.scheduled_ns << ','
						<< static_cast<unsigned int>(event.schedule_clock) << ',' << event.schedule_now_ns << ','
						<< hex(event.identity.source.data(), strnlen(event.identity.source.data(), event.identity.source.size())) << ','
						<< hex(event.identity.publisher_gid.data(), event.identity.publisher_gid.size()) << ','
						<< event.identity.sequence << ',' << event.identity.source_stamp_ns << ',' << event.frame_id << ','
						<< event.result << ',' << event.value << '\n';
				}
				if (!write_bytes(output.str())) { break; }
				written_.fetch_add(count);
			}
			const auto state = stats();
			std::ostringstream footer;
			footer << "# drain queued=" << state.queued << " written=" << state.written
				<< " full=" << state.full << " contended=" << state.contended << " invalid_identity="
				<< state.invalid_identity << " closed=" << state.closed << " io_errors=" << state.io_errors << '\n';
			(void)write_bytes(footer.str());
		}
		catch (...) { io_errors_.fetch_add(1); }
	}

	void Recorder::finalize() noexcept
	{
		if (finalized_) { return; }
		finalized_ = true;
		if (::fsync(fd_) != 0) { io_errors_.fetch_add(1); }
		if (::close(fd_) != 0) { io_errors_.fetch_add(1); }
		fd_ = -1;
		// 独立状态只在数据文件 fsync/close 之后生成；自身关闭失败则不发布完成标记。
		try
		{
			const auto state = stats();
			std::ostringstream output;
			output << "{\"schema\":1,\"data_closed\":true,\"queued\":" << state.queued
				<< ",\"written\":" << state.written << ",\"full\":" << state.full
				<< ",\"contended\":" << state.contended << ",\"invalid_identity\":"
				<< state.invalid_identity << ",\"closed\":" << state.closed
				<< ",\"io_errors\":" << state.io_errors << "}\n";
			const auto bytes = output.str();
			std::size_t offset = 0;
			bool complete = true;
			while (offset < bytes.size())
			{
				const auto count = ::write(status_fd_, bytes.data() + offset, bytes.size() - offset);
				if (count > 0) { offset += static_cast<std::size_t>(count); }
				else if (count < 0 && errno == EINTR) { continue; }
				else { complete = false; break; }
			}
			if (::fsync(status_fd_) != 0) { complete = false; }
			if (::close(status_fd_) != 0) { complete = false; }
			status_fd_ = -1;
			if (complete && ::link((path_ + ".status.partial").c_str(),
				(path_ + ".status.json").c_str()) == 0)
			{
				(void)::unlink((path_ + ".status.partial").c_str());
			}
			else { io_errors_.fetch_add(1); }
		}
		catch (...) { io_errors_.fetch_add(1); }
		if (status_fd_ >= 0) { (void)::close(status_fd_); status_fd_ = -1; }
		if (io_errors_.load() != 0)
		{
			constexpr char message[] = "timing finalization I/O error\n";
			const auto reported = ::write(STDERR_FILENO, message, sizeof(message) - 1);
			(void)reported;
		}
	}

	void Recorder::close() noexcept
	{
		std::lock_guard<std::mutex> close_lock(close_mutex_);
		{ std::lock_guard<std::mutex> lock(queue_mutex_); closing_.store(true, std::memory_order_release); }
		wake_.notify_one();
		if (writer_.joinable()) { writer_.join(); }
		finalize();
	}

	std::shared_ptr<Recorder> open_optional(const std::string &path)
	{
		return path.empty() ? nullptr : std::make_shared<Recorder>(path);
	}
}
