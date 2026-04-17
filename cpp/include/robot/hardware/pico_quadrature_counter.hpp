#pragma once

#include "robot/hardware/encoder_counter.hpp"

namespace robot::hardware {

class PicoQuadratureCounter final : public EncoderCounter {
 public:
  PicoQuadratureCounter(int state_machine_id, int pin_a, int pin_b);
  ~PicoQuadratureCounter() override;

  [[nodiscard]] int32_t Read() const override;
  void SetZero() override;
  void Deinit() override;

 private:
  int state_machine_id_;
  int pin_a_;
  int pin_b_;
};

}  // namespace robot::hardware
