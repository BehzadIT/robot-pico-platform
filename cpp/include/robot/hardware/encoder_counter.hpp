#pragma once

#include <cstdint>
#include <functional>
#include <memory>

namespace robot::hardware {

struct EncoderCounterConfig {
  int state_machine_id{0};
  int pin_a{0};
  int pin_b{0};
};

/**
 * Minimal quadrature-counter interface.
 *
 * The Pico runtime backs this with a persistent PIO owner. Host tests can use
 * fake counters to preserve the same subsystem boundary without hardware.
 */
class EncoderCounter {
 public:
  virtual ~EncoderCounter() = default;

  [[nodiscard]] virtual int32_t Read() const = 0;
  virtual void SetZero() = 0;
  virtual void Deinit() = 0;
};

using EncoderCounterFactory =
    std::function<std::unique_ptr<EncoderCounter>(const EncoderCounterConfig&)>;

}  // namespace robot::hardware
