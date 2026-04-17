#include "robot/support/logger.hpp"

#include <atomic>
#include <cstdio>
#include <optional>
#include <string>

namespace robot::support {

namespace {

std::atomic<int> g_log_level{static_cast<int>(LogLevel::kInfo)};
std::atomic_flag g_sink_lock = ATOMIC_FLAG_INIT;
TickProvider g_tick_provider = nullptr;
LogSink g_log_sink = nullptr;

class SpinGuard {
 public:
  explicit SpinGuard(std::atomic_flag& lock) noexcept : lock_(lock) {
    while (lock_.test_and_set(std::memory_order_acquire)) {
    }
  }

  ~SpinGuard() { lock_.clear(std::memory_order_release); }

 private:
  std::atomic_flag& lock_;
};

[[nodiscard]] const char* LevelName(LogLevel level) noexcept {
  switch (level) {
    case LogLevel::kError:
      return "E";
    case LogLevel::kWarn:
      return "W";
    case LogLevel::kInfo:
      return "I";
    case LogLevel::kDebug:
      return "D";
    case LogLevel::kVerbose:
      return "V";
  }
  return "I";
}

[[nodiscard]] uint32_t TicksMs() noexcept {
  if (g_tick_provider == nullptr) {
    return 0;
  }
  return g_tick_provider();
}

void DefaultSink(const char* line) noexcept {
  if (line == nullptr) {
    return;
  }
  std::fputs(line, stdout);
  std::fflush(stdout);
}

[[nodiscard]] LogSink ActiveSink() noexcept {
  return g_log_sink == nullptr ? &DefaultSink : g_log_sink;
}

void WriteLine(const std::string& line) noexcept {
  SpinGuard guard(g_sink_lock);
  ActiveSink()(line.c_str());
}

}  // namespace

bool DistinctFilter::ShouldEmit(std::string_view value) noexcept {
  if (!last_value_.has_value() || *last_value_ != value) {
    last_value_.emplace(value);
    return true;
  }
  return false;
}

void DistinctFilter::Reset() noexcept {
  last_value_.reset();
}

RateLimitFilter::RateLimitFilter(uint32_t period_ms) noexcept : period_ms_(period_ms) {}

bool RateLimitFilter::ShouldEmit(uint32_t now_ms) noexcept {
  if (!last_emit_ms_.has_value() || static_cast<uint32_t>(now_ms - *last_emit_ms_) >= period_ms_) {
    last_emit_ms_ = now_ms;
    return true;
  }
  return false;
}

void RateLimitFilter::Reset() noexcept {
  last_emit_ms_.reset();
}

void SetGlobalLogLevel(LogLevel level) noexcept {
  g_log_level.store(static_cast<int>(level), std::memory_order_relaxed);
}

LogLevel global_log_level() noexcept {
  return static_cast<LogLevel>(g_log_level.load(std::memory_order_relaxed));
}

void SetTickProvider(TickProvider provider) noexcept {
  SpinGuard guard(g_sink_lock);
  g_tick_provider = provider;
}

void ResetTickProvider() noexcept {
  SetTickProvider(nullptr);
}

void SetLogSink(LogSink sink) noexcept {
  SpinGuard guard(g_sink_lock);
  g_log_sink = sink;
}

void ResetLogSink() noexcept {
  SetLogSink(nullptr);
}

void Log(LogLevel level, std::string_view tag, std::string_view message) noexcept {
  if (static_cast<int>(level) > g_log_level.load(std::memory_order_relaxed)) {
    return;
  }

  std::string line;
  line.reserve(tag.size() + message.size() + 24U);
  line += LevelName(level);
  line += " (";
  line += std::to_string(TicksMs());
  line += ")";
  if (!tag.empty()) {
    line.push_back(' ');
    line += tag;
    line += ": ";
  } else {
    line.push_back(' ');
  }
  line += message;
  line.push_back('\n');
  WriteLine(line);
}

void LogE(std::string_view tag, std::string_view message) noexcept {
  Log(LogLevel::kError, tag, message);
}

void LogW(std::string_view tag, std::string_view message) noexcept {
  Log(LogLevel::kWarn, tag, message);
}

void LogI(std::string_view tag, std::string_view message) noexcept {
  Log(LogLevel::kInfo, tag, message);
}

void LogD(std::string_view tag, std::string_view message) noexcept {
  Log(LogLevel::kDebug, tag, message);
}

void LogV(std::string_view tag, std::string_view message) noexcept {
  Log(LogLevel::kVerbose, tag, message);
}

}  // namespace robot::support
