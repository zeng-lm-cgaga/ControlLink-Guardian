#pragma once

#include <cstdint>
#include <deque>
#include <filesystem>
#include <string>

namespace control_link_gateway {
struct FaultCapturePolicy {
  bool enabled{false};
  std::uint64_t pre_events{0}, post_events{0}, post_window_ms{0}, cooldown_ms{0};
  std::uint64_t max_record_bytes{0}, segment_bytes{0}, total_bytes{0}, max_files{0};
};
struct FaultCaptureStatus {
  std::uint64_t written{0}, dropped{0}, budget_exhausted{0}, oversized{0};
  std::uint64_t triggers{0}, merged{0}, cooled{0}, rotated_files{0}, io_errors{0}, truncated{0};
};
FaultCapturePolicy load_fault_capture_policy(const std::filesystem::path &path);
void validate_fault_capture_policy(const FaultCapturePolicy &policy);

// 只在 DecisionTraceRecorder 既有writer线程调用；独占新目录，不扫描/清理其他日志。
// pre环与post窗均有界；同窗触发合并但不延长截止；固定单个并发快照。
class FaultCaptureWriter final {
 public:
  FaultCaptureWriter(std::filesystem::path directory, std::string trace_header, FaultCapturePolicy policy);
  ~FaultCaptureWriter();
  FaultCaptureWriter(const FaultCaptureWriter &) = delete;
  FaultCaptureWriter &operator=(const FaultCaptureWriter &) = delete;
  void append(std::string record, std::uint64_t monotonic_ns, bool trigger);
  // 仅显式成功收束才发布closed marker；异常析构只关fd，避免把未排空队列误记为完整关闭。
  void finish();
  FaultCaptureStatus status() const { return status_; }
 private:
  struct File { int fd{-1}; std::string name; std::uint64_t bytes{0}; };
  struct Closed { std::string name; std::uint64_t bytes; };
  bool make_room(std::uint64_t bytes);
  void open(File &file, const std::string &kind, std::uint64_t snapshot, std::uint64_t now);
  bool write(File &file, const std::string &bytes, bool reserve_footer = true);
  void close(File &file, bool truncated);
  void store(File &file, const std::string &record, const std::string &kind, std::uint64_t now);
  std::filesystem::path directory_;
  std::string header_;
  FaultCapturePolicy policy_;
  FaultCaptureStatus status_;
  int directory_fd_{-1};
  File continuous_, snapshot_;
  std::deque<Closed> closed_;
  std::deque<std::string> pre_;
  std::uint64_t bytes_{0}, next_file_{0}, snapshot_id_{0}, last_trigger_{0};
  std::uint64_t snapshot_end_{0}, post_count_{0}, last_event_{0};
  std::uint64_t process_starttime_{0};
  bool finished_{false};
};
}
