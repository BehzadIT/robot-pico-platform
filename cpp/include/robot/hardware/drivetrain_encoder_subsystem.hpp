#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "robot/control/constants.hpp"
#include "robot/hardware/encoder_counter.hpp"

namespace robot::hardware {

struct EncoderFault {
  std::string code;
  std::string message;
};

struct EncoderSnapshot {
  uint64_t t_us{0};
  int32_t right_count{0};
  int32_t left_count{0};
};

struct EncoderVelocitySample {
  bool valid{false};
  bool primed{false};
  uint64_t t_us{0};
  uint32_t dt_us{0};
  int32_t right_count{0};
  int32_t left_count{0};
  int32_t right_delta{0};
  int32_t left_delta{0};
  double right_rpm{0.0};
  double left_rpm{0.0};
  int command_age_ms{0};
};

/**
 * Persistent owner for both drivetrain encoders.
 *
 * The safety rule from the MicroPython runtime stays intact here: one
 * subsystem owns both counters across start/stop cycles, and recovery tears
 * down that whole owner boundary instead of patching one state machine in
 * place.
 */
class DrivetrainEncoderSubsystem {
 public:
  explicit DrivetrainEncoderSubsystem(EncoderCounterFactory counter_factory);

  [[nodiscard]] bool Initialize();
  [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
  [[nodiscard]] bool is_faulted() const noexcept { return fault_.has_value(); }
  [[nodiscard]] std::optional<EncoderFault> fault_info() const { return fault_; }
  [[nodiscard]] std::optional<EncoderFault> last_fault_info() const {
    return last_fault_;
  }

  [[nodiscard]] bool ResetCounts();
  [[nodiscard]] EncoderSnapshot Snapshot(uint64_t now_us) const;
  [[nodiscard]] EncoderVelocitySample PrimeVelocityEstimator(uint64_t now_us);
  [[nodiscard]] EncoderVelocitySample SampleVelocity(uint64_t now_us);
  void Shutdown() noexcept;
  [[nodiscard]] bool Recover();

 private:
  void ResetVelocityEstimator() noexcept;
  [[nodiscard]] EncoderSnapshot ReadCounts(uint64_t now_us) const;
  void SetFault(std::string code, std::string message);

  EncoderCounterFactory counter_factory_;
  std::unique_ptr<EncoderCounter> right_counter_;
  std::unique_ptr<EncoderCounter> left_counter_;
  bool initialized_{false};
  std::optional<EncoderFault> fault_;
  std::optional<EncoderFault> last_fault_;
  std::optional<uint64_t> last_sample_t_us_;
  std::optional<int32_t> last_right_count_;
  std::optional<int32_t> last_left_count_;
  EncoderVelocitySample velocity_sample_{};
};

}  // namespace robot::hardware
