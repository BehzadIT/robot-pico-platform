#pragma once

#include <cstdint>

#include "robot/config/config.hpp"

namespace robot::platform {

/**
 * Preserve the controller-UART runtime configuration and cadence policy.
 *
 * This helper keeps the firmware/app boundary explicit: the Pico app owns the
 * actual UART hardware, while this object owns the enabled flag, pin/baud
 * parameters, and the poll/sample cadence derived from config.
 */
class ControllerUartRuntime {
 public:
  explicit ControllerUartRuntime(const robot::config::ControllerUartConfig& config);

  [[nodiscard]] bool enabled() const noexcept { return enabled_; }
  [[nodiscard]] bool IsHardwareConfigValid() const noexcept;

  [[nodiscard]] int uart_id() const noexcept { return uart_id_; }
  [[nodiscard]] int baud() const noexcept { return baud_; }
  [[nodiscard]] int tx_pin() const noexcept { return tx_pin_; }
  [[nodiscard]] int rx_pin() const noexcept { return rx_pin_; }
  [[nodiscard]] int read_chunk() const noexcept { return read_chunk_; }
  [[nodiscard]] int poll_ms() const noexcept { return poll_ms_; }
  [[nodiscard]] int diag_sample_ms() const noexcept { return diag_sample_ms_; }

  [[nodiscard]] bool ShouldPoll(uint32_t now_ms) noexcept;
  [[nodiscard]] bool ShouldSample(uint32_t now_ms) noexcept;
  void MarkPolled(uint32_t now_ms) noexcept;
  void MarkSampled(uint32_t now_ms) noexcept;

 private:
  bool enabled_{false};
  int uart_id_{1};
  int baud_{115200};
  int tx_pin_{4};
  int rx_pin_{5};
  int read_chunk_{64};
  int poll_ms_{10};
  int diag_sample_ms_{500};
  bool poll_initialized_{false};
  bool sample_initialized_{false};
  uint32_t last_poll_ms_{0};
  uint32_t last_sample_ms_{0};
};

}  // namespace robot::platform
