#pragma once

namespace robot::hardware {

/**
 * Watchdog boundary for the deterministic control owner.
 *
 * The control domain feeds the watchdog only after real progress. Service-side
 * transport work must not be allowed to hide a wedged drivetrain loop.
 */
class Watchdog {
 public:
  virtual ~Watchdog() = default;

  virtual bool Arm(int timeout_ms) = 0;
  virtual void Feed() = 0;
  [[nodiscard]] virtual bool armed() const = 0;
};

class NullWatchdog final : public Watchdog {
 public:
  bool Arm(int /*timeout_ms*/) override {
    armed_ = true;
    return true;
  }

  void Feed() override {}

  [[nodiscard]] bool armed() const override { return armed_; }

 private:
  bool armed_{false};
};

}  // namespace robot::hardware
