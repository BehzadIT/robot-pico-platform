#include "robot/platform/controller_uart_runtime.hpp"

#include <algorithm>

namespace robot::platform {

ControllerUartRuntime::ControllerUartRuntime(const robot::config::ControllerUartConfig& config)
    : enabled_(config.enabled),
      uart_id_(config.uart_id),
      baud_(config.baud),
      tx_pin_(config.tx_pin),
      rx_pin_(config.rx_pin),
      read_chunk_(std::max(1, config.read_chunk)),
      poll_ms_(std::max(1, config.poll_ms)),
      diag_sample_ms_(std::max(0, config.diag_sample_ms)) {}

bool ControllerUartRuntime::IsHardwareConfigValid() const noexcept {
  return enabled_ && (uart_id_ == 0 || uart_id_ == 1) && baud_ > 0 && tx_pin_ >= 0 &&
         rx_pin_ >= 0;
}

bool ControllerUartRuntime::ShouldPoll(uint32_t now_ms) noexcept {
  if (!IsHardwareConfigValid()) {
    return false;
  }
  if (!poll_initialized_) {
    poll_initialized_ = true;
    last_poll_ms_ = now_ms;
    return true;
  }
  if (now_ms - last_poll_ms_ >= static_cast<uint32_t>(poll_ms_)) {
    last_poll_ms_ = now_ms;
    return true;
  }
  return false;
}

bool ControllerUartRuntime::ShouldSample(uint32_t now_ms) noexcept {
  if (!IsHardwareConfigValid() || diag_sample_ms_ <= 0) {
    return false;
  }
  if (!sample_initialized_) {
    sample_initialized_ = true;
    last_sample_ms_ = now_ms;
    return true;
  }
  if (now_ms - last_sample_ms_ >= static_cast<uint32_t>(diag_sample_ms_)) {
    last_sample_ms_ = now_ms;
    return true;
  }
  return false;
}

void ControllerUartRuntime::MarkPolled(uint32_t now_ms) noexcept {
  poll_initialized_ = true;
  last_poll_ms_ = now_ms;
}

void ControllerUartRuntime::MarkSampled(uint32_t now_ms) noexcept {
  sample_initialized_ = true;
  last_sample_ms_ = now_ms;
}

}  // namespace robot::platform
