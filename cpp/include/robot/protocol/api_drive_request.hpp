#pragma once

#include <optional>
#include <string>

namespace robot::protocol {

/**
 * Normalized drive request at the transport/control boundary.
 *
 * Transport code may package operator intent into this shape, but command
 * freshness, source ownership, and motion acceptance stay in control code.
 */
struct ApiDriveRequest {
  std::optional<int> seq;
  std::optional<int> sent_at_ms;
  int throttle_percent{0};
  int turn_percent{0};
  int target_rpm{0};
  int target_direction{0};
  int target_angle{0};

  static ApiDriveRequest FromIntent(int throttle_percent,
                                    int turn_percent,
                                    std::optional<int> seq = std::nullopt,
                                    std::optional<int> sent_at_ms = std::nullopt);
};

}  // namespace robot::protocol
