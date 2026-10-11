#include "../src/application_runtime.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

#include <gtest/gtest.h>

namespace control_link::someip {
namespace {

class RuntimeEnvironment final {
 public:
  RuntimeEnvironment() {
    for (const auto &name : names_) {
      const auto *value = std::getenv(name.c_str());
      original_.push_back(value ? std::optional<std::string>{value} : std::nullopt);
      ::unsetenv(name.c_str());
    }
    std::array<char, 48> path{};
    const std::string pattern = "/tmp/control-link-runtime-XXXXXX";
    std::copy(pattern.begin(), pattern.end(), path.begin());
    const auto descriptor = ::mkstemp(path.data());
    if (descriptor < 0) throw std::runtime_error("mkstemp failed");
    ::close(descriptor);
    path_ = path.data();
  }
  ~RuntimeEnvironment() {
    for (std::size_t index = 0; index < names_.size(); ++index) {
      if (original_[index]) ::setenv(names_[index].c_str(), original_[index]->c_str(), 1);
      else ::unsetenv(names_[index].c_str());
    }
    std::filesystem::remove(path_);
  }
  const std::string &path() const { return path_; }
 private:
  const std::array<std::string, 2> names_{"VSOMEIP_CONFIGURATION", "VSOMEIP_CONFIGURATION_runtime_test"};
  std::vector<std::optional<std::string>> original_;
  std::string path_;
};

TEST(SomeipApplicationRuntime, BindsExplicitConfiguration) {
  RuntimeEnvironment environment;
  detail::bind_application_configuration("runtime_test", environment.path());
  EXPECT_EQ(std::string{std::getenv("VSOMEIP_CONFIGURATION")}, environment.path());
}

TEST(SomeipApplicationRuntime, RejectsApplicationSpecificConfigurationOverride) {
  RuntimeEnvironment environment;
  ::setenv("VSOMEIP_CONFIGURATION_runtime_test", "/tmp/another-unvalidated.json", 1);
  EXPECT_THROW(detail::bind_application_configuration("runtime_test", environment.path()),
               std::invalid_argument);
  EXPECT_EQ(std::getenv("VSOMEIP_CONFIGURATION"), nullptr);
}

TEST(SomeipApplicationRuntime, AcceptsSameApplicationSpecificConfiguration) {
  RuntimeEnvironment environment;
  ::setenv("VSOMEIP_CONFIGURATION_runtime_test", environment.path().c_str(), 1);
  EXPECT_NO_THROW(detail::bind_application_configuration("runtime_test", environment.path()));
}

TEST(SomeipApplicationRuntime, NormalLoopExitRetainsCooperativeOwnerCleanup) {
  int starts = 0;
  int failures = 0;
  int exits = 0;
  detail::run_application_loop([&] { ++starts; }, [&](const char *) { ++failures; }, [&] { ++exits; });
  EXPECT_EQ(starts, 1);
  EXPECT_EQ(failures, 0);
  EXPECT_EQ(exits, 1);
}

TEST(SomeipApplicationRuntime, StartExceptionRevokesBeforeProcessExitWithoutExternalStopWait) {
  EXPECT_EXIT(detail::run_application_loop(
      [] { throw std::runtime_error("injected partial startup"); },
      [](const char *) { std::fprintf(stderr, "revoked-before-exit; "); },
      [] { std::_Exit(99); }), ::testing::ExitedWithCode(EXIT_FAILURE),
      "revoked-before-exit; ControlLink SOME/IP application loop failed");
}

TEST(SomeipApplicationRuntime, UnknownStartExceptionCannotBecomeAReusableStoppedLoop) {
  EXPECT_EXIT(detail::run_application_loop([] { throw 42; }, [](const char *) {},
      [] { std::_Exit(99); }), ::testing::ExitedWithCode(EXIT_FAILURE), "Unknown exception");
}

TEST(SomeipApplicationRuntime, FailureDiagnosticExceptionStillFailsClosed) {
  EXPECT_EXIT(detail::run_application_loop([] { throw std::runtime_error("start failure"); },
      [](const char *) { throw std::runtime_error("revoke allocation failure"); },
      [] { std::_Exit(99); }), ::testing::ExitedWithCode(EXIT_FAILURE), "process fail-closed");
}

}
}
