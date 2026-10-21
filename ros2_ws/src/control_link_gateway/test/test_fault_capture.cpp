#include "control_link_gateway/fault_capture.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <csignal>

using namespace control_link_gateway;
namespace {
std::filesystem::path new_directory() {
  std::filesystem::create_directories(CONTROL_LINK_FAULT_TEST_ROOT);
  std::string pattern = std::string(CONTROL_LINK_FAULT_TEST_ROOT) + "/case-XXXXXX";
  if (!::mkdtemp(pattern.data())) throw std::runtime_error("cannot create test artifact owner");
  return std::filesystem::path(pattern) / "capture";
}
FaultCapturePolicy policy() { return {true, 2, 3, 100, 1000, 1024, 4096, 16384, 4}; }
std::string record(int sequence, std::size_t padding = 0) {
  return nlohmann::json{{"record_type", "synthetic_test_event"}, {"sequence", sequence},
                        {"padding", std::string(padding, 'x')}}.dump() + '\n';
}
TEST(FaultCapture, PrePostRingMergeAndPartialReplayBoundary) {
  const auto directory = new_directory();
  FaultCaptureWriter writer(directory, "{\"synthetic_header\":true}", policy());
  for (int sequence = 1; sequence <= 6; ++sequence)
    writer.append(record(sequence), 1000000000ULL + sequence * 1000000, sequence == 3 || sequence == 4);
  writer.finish();
  EXPECT_EQ(writer.status().triggers, 2U);
  EXPECT_EQ(writer.status().merged, 1U);
  std::vector<int> captured;
  for (const auto &file : std::filesystem::directory_iterator(directory)) {
    if (file.path().filename().string().find("fault-") != 0) continue;
    std::ifstream source(file.path());
    std::string line;
    while (std::getline(source, line)) {
      const auto row = nlohmann::json::parse(line);
      if (row.contains("sequence")) captured.push_back(row["sequence"]);
      if (row.contains("replay_complete")) EXPECT_FALSE(row["replay_complete"].get<bool>());
    }
  }
  EXPECT_EQ(captured, (std::vector<int>{1, 2, 3, 4, 5}));
  EXPECT_TRUE(std::filesystem::is_regular_file(directory / "capture-status.json"));
}
TEST(FaultCapture, RotationAndTotalBytesStayInsideOwnedDirectory) {
  const auto directory = new_directory();
  FaultCaptureWriter writer(directory, "{}", policy());
  for (int sequence = 0; sequence < 100; ++sequence) writer.append(record(sequence, 600), 1000000000ULL + sequence, false);
  writer.finish();
  std::uint64_t bytes = 0, count = 0;
  for (const auto &file : std::filesystem::directory_iterator(directory)) { bytes += file.file_size(); ++count; }
  EXPECT_LE(bytes, policy().total_bytes);
  EXPECT_LE(count, policy().max_files);
  EXPECT_GT(writer.status().rotated_files, 0U);
  std::ifstream status_file(directory / "capture-status.json");
  const auto status = nlohmann::json::parse(status_file);
  EXPECT_EQ(status.at("rotated_files"), writer.status().rotated_files);
}
TEST(FaultCapture, OversizeAndCooldownAreCountedWithoutUnboundedSnapshots) {
  const auto directory = new_directory();
  FaultCaptureWriter writer(directory, "{}", policy());
  writer.append(record(0, 2000), 1000000000, false);
  writer.append(record(1), 1100000000, true);
  writer.append(record(2), 1300000000, true);
  writer.finish();
  EXPECT_EQ(writer.status().oversized, 1U);
  EXPECT_EQ(writer.status().dropped, 1U);
  EXPECT_EQ(writer.status().cooled, 1U);
}
TEST(FaultCapture, StopTruncatesPostWindowAndRejectsExistingDirectory) {
  const auto directory = new_directory();
  FaultCaptureWriter writer(directory, "{}", policy());
  writer.append(record(1), 1000000000, true);
  writer.finish();
  EXPECT_GE(writer.status().truncated, 1U);
  EXPECT_THROW((FaultCaptureWriter{directory, "{}", policy()}), std::invalid_argument);
}
TEST(FaultCapture, InvalidBudgetAndOpenFailureAreVisible) {
  auto invalid = policy(); invalid.total_bytes = 1024;
  EXPECT_THROW((FaultCaptureWriter{new_directory(), "{}", invalid}), std::invalid_argument);
  const auto directory = new_directory();
  FaultCaptureWriter writer(directory, "{}", policy());
  ASSERT_TRUE(std::filesystem::remove(directory));
  EXPECT_THROW(writer.append(record(1), 1000000000, false), std::runtime_error);
  EXPECT_GT(writer.status().io_errors, 0U);
}
TEST(FaultCapture, WriteFailureCannotPublishClosedMarker) {
  const auto directory = new_directory();
  const auto child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::signal(SIGXFSZ, SIG_IGN);
    const rlimit limit{128, 128};
    if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) ::_exit(2);
    try {
      FaultCaptureWriter writer(directory, "{}", policy());
      try { writer.append(record(1, 600), 1000000000, true); }
      catch (const std::exception &) { ::_exit(writer.status().io_errors > 0 ? 0 : 3); }
      ::_exit(4);
    } catch (...) { ::_exit(5); }
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_FALSE(std::filesystem::exists(directory / "capture-status.json"));
}
TEST(FaultCapture, ExceptionUnwindDoesNotClaimExplicitQueueDrain) {
  const auto directory = new_directory();
  {
    FaultCaptureWriter writer(directory, "{}", policy());
    writer.append(record(1), 1000000000, true);
  }
  EXPECT_FALSE(std::filesystem::exists(directory / "capture-status.json"));
}
}
