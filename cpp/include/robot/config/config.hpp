#pragma once

#include <cstdint>

namespace robot::config {

struct WifiConfig {
  int connect_timeout_s{20};
  int max_attempts{3};
  int retry_backoff_ms{1000};
  int pre_reset_delay_ms{300};
  int post_activate_delay_ms{300};
  int status_poll_ms{250};
};

struct DrivetrainConfig {
  bool enable_watchdog{true};
  // Keep staged watchdog rollout semantics from the MicroPython runtime so the
  // first watchdog-enabled sessions are not cold-start paths.
  bool watchdog_arm_after_first_stop{true};
  int watchdog_timeout_ms{4000};
};

struct ControllerUartConfig {
  bool enabled{true};
  int uart_id{1};
  int baud{115200};
  int tx_pin{4};
  int rx_pin{5};
  int read_chunk{64};
  int poll_ms{10};
  int diag_sample_ms{500};
};

struct ControllerDriveConfig {
  bool enabled{true};
  int publish_period_ms{50};
  int snapshot_stale_ms{250};
  // 30/255 from XInput scales to about 118 on the project's 0..1000 trigger
  // normalization, so retain the current conservative 120 threshold.
  int trigger_press_threshold{120};
  // Lower release threshold preserves hysteresis and prevents chatter near
  // trigger neutral.
  int trigger_release_threshold{80};
  int turn_deadzone{240};
};

inline constexpr WifiConfig kWifiConfig{};
inline constexpr DrivetrainConfig kDrivetrainConfig{};
inline constexpr ControllerUartConfig kControllerUartConfig{};
inline constexpr ControllerDriveConfig kControllerDriveConfig{};

}  // namespace robot::config
