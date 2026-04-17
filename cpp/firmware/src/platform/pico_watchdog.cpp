#include "robot/platform/pico_watchdog.hpp"

#include "hardware/watchdog.h"

namespace robot::platform {

bool PicoWatchdog::Arm(int timeout_ms) {
  watchdog_enable(static_cast<uint32_t>(timeout_ms), true);
  armed_ = true;
  return true;
}

void PicoWatchdog::Feed() {
  if (armed_) {
    watchdog_update();
  }
}

}  // namespace robot::platform
