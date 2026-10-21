#include "control_link_gateway/fault_capture.hpp"

#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <set>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace control_link_gateway {
namespace {
constexpr std::uint64_t footer_reserve = 512;
void io_failure(const char *operation) { throw std::runtime_error(std::string("fault capture ") + operation + " failed errno=" + std::to_string(errno)); }
}
FaultCapturePolicy load_fault_capture_policy(const std::filesystem::path &path) {
  if (!path.is_absolute() || !std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > 4096)
    throw std::invalid_argument("fault capture config must be an absolute regular file <=4KiB");
  const auto node = YAML::LoadFile(path.string());
  const std::set<std::string> fields{"schema_version", "pre_events", "post_events", "post_window_ms", "cooldown_ms",
      "max_record_bytes", "segment_bytes", "total_bytes", "max_files"};
  if (!node.IsMap() || node.size() != fields.size()) throw std::invalid_argument("fault capture exact schema required");
  std::set<std::string> seen;
  for (const auto &entry : node) {
    const auto key = entry.first.as<std::string>();
    if (!fields.count(key) || !seen.insert(key).second) throw std::invalid_argument("fault capture unknown/duplicate key");
    if (!entry.second.IsScalar()) throw std::invalid_argument("fault capture positive decimal required");
    const auto value = entry.second.Scalar();
    if (value.empty() || value.size() > 10 || value.find_first_not_of("0123456789") != std::string::npos)
      throw std::invalid_argument("fault capture bounded decimal required");
  }
  const auto read = [&](const char *key) { return node[key].as<std::uint64_t>(); };
  FaultCapturePolicy policy{true, read("pre_events"), read("post_events"), read("post_window_ms"), read("cooldown_ms"),
      read("max_record_bytes"), read("segment_bytes"), read("total_bytes"), read("max_files")};
  if (read("schema_version") != 1) throw std::invalid_argument("fault capture unsupported schema");
  validate_fault_capture_policy(policy);
  return policy;
}
void validate_fault_capture_policy(const FaultCapturePolicy &policy) {
  if (!policy.enabled || !policy.pre_events || policy.pre_events > 4096 ||
      !policy.post_events || policy.post_events > 4096 || !policy.post_window_ms || policy.post_window_ms > 60000 ||
      !policy.cooldown_ms || policy.cooldown_ms > 3600000 || policy.max_record_bytes < 1024 || policy.max_record_bytes > 1048576 ||
      policy.segment_bytes < policy.max_record_bytes * 2 + footer_reserve * 2 || policy.segment_bytes > 16777216 ||
      policy.total_bytes < policy.segment_bytes * 2 || policy.total_bytes > 1073741824 || policy.max_files < 4 || policy.max_files > 1024)
    throw std::invalid_argument("fault capture invalid finite memory/disk/window budget");
}
FaultCaptureWriter::FaultCaptureWriter(std::filesystem::path directory, std::string trace_header, FaultCapturePolicy policy)
    : directory_(std::move(directory)), header_(std::move(trace_header)), policy_(policy) {
  validate_fault_capture_policy(policy_);
  std::ifstream stat("/proc/self/stat");
  std::string record;
  std::getline(stat, record);
  std::istringstream fields(record.substr(record.rfind(')') + 2));
  std::string value;
  for (unsigned index = 0; index <= 19; ++index) {
    if (!(fields >> value)) throw std::runtime_error("cannot bind fault capture process starttime");
  }
  process_starttime_ = std::stoull(value);
  if (!directory_.is_absolute() || !std::filesystem::create_directory(directory_))
    throw std::invalid_argument("fault capture requires a new owned absolute directory");
  std::filesystem::permissions(directory_, std::filesystem::perms::owner_all);
  directory_fd_ = ::open(directory_.c_str(), O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  if (directory_fd_ < 0) io_failure("open directory");
  if (header_.size() > policy_.max_record_bytes / 2) {
    ::close(directory_fd_); directory_fd_ = -1;
    throw std::invalid_argument("fault capture header exceeds record budget");
  }
}
FaultCaptureWriter::~FaultCaptureWriter() {
  if (continuous_.fd >= 0) ::close(continuous_.fd);
  if (snapshot_.fd >= 0) ::close(snapshot_.fd);
  if (directory_fd_ >= 0) ::close(directory_fd_);
}
bool FaultCaptureWriter::make_room(std::uint64_t needed) {
  while ((!closed_.empty()) &&
      (bytes_ + needed + 4096 > policy_.total_bytes ||
       closed_.size() + (continuous_.fd >= 0) + (snapshot_.fd >= 0) >= policy_.max_files)) {
    const auto oldest = closed_.front();
    if (::unlinkat(directory_fd_, oldest.name.c_str(), 0) != 0) { ++status_.io_errors; io_failure("rotate unlink"); }
    bytes_ -= oldest.bytes; closed_.pop_front(); ++status_.rotated_files;
  }
  if (bytes_ + needed + 4096 > policy_.total_bytes) {
    ++status_.budget_exhausted; ++status_.dropped; return false;
  }
  return true;
}
bool FaultCaptureWriter::write(File &file, const std::string &record, bool reserve_footer) {
  if (file.fd < 0) return false;
  if (reserve_footer && !make_room(record.size())) return false;
  std::size_t offset = 0;
  while (offset < record.size()) {
    const auto count = ::write(file.fd, record.data() + offset, record.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) { ++status_.io_errors; io_failure("write"); }
    offset += static_cast<std::size_t>(count); file.bytes += count; bytes_ += count;
  }
  return true;
}
void FaultCaptureWriter::open(File &file, const std::string &kind, std::uint64_t snapshot, std::uint64_t now) {
  const auto metadata = nlohmann::json{{"record_type", "capture_header"}, {"schema_version", 1}, {"kind", kind},
      {"snapshot_id", snapshot}, {"monotonic_ns", now}, {"clock", "CLOCK_MONOTONIC"}, {"pid", ::getpid()},
      {"starttime", process_starttime_},
      {"trace_header", nlohmann::json::parse(header_)}, {"replay_complete", false},
      {"prehistory_may_be_truncated", true}, {"rotation_index", next_file_}, {"rotated_files", status_.rotated_files}}.dump() + '\n';
  if (!make_room(metadata.size())) return;
  file.name = kind + "-" + std::to_string(++next_file_) + ".jsonl";
  file.bytes = 0;
  file.fd = ::openat(directory_fd_, file.name.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (file.fd < 0) { ++status_.io_errors; io_failure("open segment"); }
  write(file, metadata);
}
void FaultCaptureWriter::close(File &file, bool truncated) {
  if (file.fd < 0) return;
  const auto footer = nlohmann::json{{"record_type", "capture_footer"}, {"truncated", truncated},
      {"replay_complete", false}, {"last_monotonic_ns", last_event_}, {"dropped", status_.dropped},
      {"io_errors", status_.io_errors}, {"rotated_files", status_.rotated_files}}.dump() + '\n';
  if (footer.size() > footer_reserve) throw std::runtime_error("fault footer exceeds reserved budget");
  write(file, footer, false);
  const int descriptor = file.fd;
  const bool synced = ::fsync(descriptor) == 0;
  const bool closed = ::close(descriptor) == 0;
  file.fd = -1;
  if (!synced || !closed) { ++status_.io_errors; io_failure("fsync/close"); }
  closed_.push_back({file.name, file.bytes});
  if (truncated) ++status_.truncated;
}
void FaultCaptureWriter::store(File &file, const std::string &record, const std::string &kind, std::uint64_t now) {
  if (file.fd >= 0 && file.bytes + record.size() + footer_reserve > policy_.segment_bytes) close(file, true);
  if (file.fd < 0) open(file, kind, snapshot_id_, now);
  if (write(file, record)) ++status_.written;
}
void FaultCaptureWriter::append(std::string record, std::uint64_t now, bool trigger) {
  if (finished_) return;
  last_event_ = now;
  if (record.size() > policy_.max_record_bytes) { ++status_.oversized; ++status_.dropped; return; }
  store(continuous_, record, "continuous", now);
  if (snapshot_.fd >= 0 && (now > snapshot_end_ || post_count_ >= policy_.post_events)) close(snapshot_, post_count_ >= policy_.post_events);
  if (trigger) {
    ++status_.triggers;
    if (snapshot_.fd >= 0) ++status_.merged;
    else if (last_trigger_ && now - last_trigger_ < policy_.cooldown_ms * 1000000) ++status_.cooled;
    else {
      last_trigger_ = now; ++snapshot_id_; post_count_ = 0;
      snapshot_end_ = now + policy_.post_window_ms * 1000000;
      open(snapshot_, "fault", snapshot_id_, now);
      for (const auto &previous : pre_) store(snapshot_, previous, "fault", now);
    }
  }
  if (snapshot_.fd >= 0) { store(snapshot_, record, "fault", now); ++post_count_; }
  pre_.push_back(std::move(record));
  if (pre_.size() > policy_.pre_events) pre_.pop_front();
}
void FaultCaptureWriter::finish() {
  if (finished_) return;
  close(snapshot_, last_event_ < snapshot_end_);
  close(continuous_, false);
  constexpr std::uint64_t status_reserve = 2048;
  if (!make_room(status_reserve)) throw std::runtime_error("fault capture final status budget exhausted");
  const auto status = nlohmann::json{{"schema_version", 1}, {"closed", true}, {"replay_complete", false},
      {"written", status_.written}, {"dropped", status_.dropped}, {"budget_exhausted", status_.budget_exhausted},
      {"oversized", status_.oversized}, {"triggers", status_.triggers}, {"merged", status_.merged},
      {"cooled", status_.cooled}, {"rotated_files", status_.rotated_files}, {"io_errors", status_.io_errors},
      {"truncated", status_.truncated}, {"pid", ::getpid()}, {"starttime", process_starttime_},
      {"marker_directory_durability", "not_claimed"}, {"last_monotonic_ns", last_event_}}.dump() + '\n';
  if (status.size() > status_reserve) throw std::runtime_error("fault capture final status exceeds reserve");
  File summary;
  summary.name = "capture-status.pending";
  summary.fd = ::openat(directory_fd_, summary.name.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (summary.fd < 0) { ++status_.io_errors; io_failure("open status"); }
  try {
    write(summary, status, false);
    const bool synced = ::fsync(summary.fd) == 0;
    const bool closed = ::close(summary.fd) == 0;
    summary.fd = -1;
    if (!synced || !closed) { ++status_.io_errors; io_failure("status fsync/close"); }
  } catch (...) {
    if (summary.fd >= 0) ::close(summary.fd);
    throw;
  }
  if (::fsync(directory_fd_) != 0) { ++status_.io_errors; io_failure("directory fsync"); }
  if (::renameat(directory_fd_, summary.name.c_str(), directory_fd_, "capture-status.json") != 0) {
    ++status_.io_errors; io_failure("publish closed status");
  }
  finished_ = true;
}
}
