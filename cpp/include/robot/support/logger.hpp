#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace robot::support {

enum class LogLevel : int {
  kError = 1,
  kWarn = 2,
  kInfo = 3,
  kDebug = 4,
  kVerbose = 5,
};

class DistinctFilter {
 public:
  [[nodiscard]] bool ShouldEmit(std::string_view value) noexcept;
  void Reset() noexcept;

 private:
  std::optional<std::string> last_value_;
};

class RateLimitFilter {
 public:
  explicit RateLimitFilter(uint32_t period_ms) noexcept;

  [[nodiscard]] bool ShouldEmit(uint32_t now_ms) noexcept;
  void Reset() noexcept;

 private:
  uint32_t period_ms_;
  std::optional<uint32_t> last_emit_ms_;
};

using TickProvider = uint32_t (*)();
using LogSink = void (*)(const char* line);

void SetGlobalLogLevel(LogLevel level) noexcept;
[[nodiscard]] LogLevel global_log_level() noexcept;

void SetTickProvider(TickProvider provider) noexcept;
void ResetTickProvider() noexcept;

void SetLogSink(LogSink sink) noexcept;
void ResetLogSink() noexcept;

void Log(LogLevel level, std::string_view tag, std::string_view message) noexcept;
void LogE(std::string_view tag, std::string_view message) noexcept;
void LogW(std::string_view tag, std::string_view message) noexcept;
void LogI(std::string_view tag, std::string_view message) noexcept;
void LogD(std::string_view tag, std::string_view message) noexcept;
void LogV(std::string_view tag, std::string_view message) noexcept;

}  // namespace robot::support
