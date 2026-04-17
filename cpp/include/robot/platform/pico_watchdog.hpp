#pragma once

#include "robot/hardware/watchdog.hpp"

namespace robot::platform {

class PicoWatchdog final : public robot::hardware::Watchdog {
 public:
  bool Arm(int timeout_ms) override;
  void Feed() override;
  [[nodiscard]] bool armed() const override { return armed_; }

 private:
  bool armed_{false};
};

}  // namespace robot::platform
