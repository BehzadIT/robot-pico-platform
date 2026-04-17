#include "robot/hardware/pico_quadrature_counter.hpp"

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/assert.h"
#include "pico/stdlib.h"
#include "quadrature_counter.pio.h"

namespace robot::hardware {

namespace {

PIO kEncoderPio = pio0;

uint ProgramOffset() {
  static const uint offset = pio_add_program(kEncoderPio, &quadrature_counter_program);
  return offset;
}

}  // namespace

PicoQuadratureCounter::PicoQuadratureCounter(int state_machine_id, int pin_a, int pin_b)
    : state_machine_id_(state_machine_id), pin_a_(pin_a), pin_b_(pin_b) {
  hard_assert(pin_b_ == pin_a_ + 1);
  const float clkdiv =
      static_cast<float>(clock_get_hz(clk_sys)) / 125000000.0f;
  quadrature_counter_program_init(kEncoderPio, static_cast<uint>(state_machine_id_),
                                  ProgramOffset(), static_cast<uint>(pin_a_), clkdiv);
  SetZero();
}

PicoQuadratureCounter::~PicoQuadratureCounter() {
  Deinit();
}

int32_t PicoQuadratureCounter::Read() const {
  pio_sm_put_blocking(kEncoderPio, static_cast<uint>(state_machine_id_), 1);
  return static_cast<int32_t>(
      pio_sm_get_blocking(kEncoderPio, static_cast<uint>(state_machine_id_)));
}

void PicoQuadratureCounter::SetZero() {
  pio_sm_exec(kEncoderPio, static_cast<uint>(state_machine_id_), pio_encode_set(pio_y, 0));
}

void PicoQuadratureCounter::Deinit() {
  pio_sm_set_enabled(kEncoderPio, static_cast<uint>(state_machine_id_), false);
}

}  // namespace robot::hardware
