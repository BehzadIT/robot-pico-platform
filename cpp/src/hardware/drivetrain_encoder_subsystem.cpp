#include "robot/hardware/drivetrain_encoder_subsystem.hpp"

#include <stdexcept>
#include <string>

#include "robot/support/logger.hpp"

namespace robot::hardware {

namespace {

constexpr double kRevScaleNumerator = 60'000'000.0;
constexpr const char* kEncoderTag = "encoder";

}  // namespace

DrivetrainEncoderSubsystem::DrivetrainEncoderSubsystem(EncoderCounterFactory counter_factory)
    : counter_factory_(std::move(counter_factory)) {}

bool DrivetrainEncoderSubsystem::Initialize() {
  if (fault_.has_value()) {
    return false;
  }
  if (initialized_) {
    return true;
  }
  try {
    right_counter_ = counter_factory_({
        .state_machine_id = robot::control::EncoderPins::kRightSmId,
        .pin_a = robot::control::EncoderPins::kRightPinA,
        .pin_b = robot::control::EncoderPins::kRightPinB,
    });
    left_counter_ = counter_factory_({
        .state_machine_id = robot::control::EncoderPins::kLeftSmId,
        .pin_a = robot::control::EncoderPins::kLeftPinA,
        .pin_b = robot::control::EncoderPins::kLeftPinB,
    });
    if (!right_counter_ || !left_counter_) {
      throw std::runtime_error("encoder counter factory returned null");
    }
    initialized_ = true;
    ResetVelocityEstimator();
    robot::support::LogI(kEncoderTag,
                         std::string("encoder subsystem initialized right_sm=") +
                             std::to_string(robot::control::EncoderPins::kRightSmId) +
                             " left_sm=" +
                             std::to_string(robot::control::EncoderPins::kLeftSmId));
    return true;
  } catch (const std::exception& exc) {
    Shutdown();
    SetFault("init_failed", exc.what());
    return false;
  }
}

bool DrivetrainEncoderSubsystem::ResetCounts() {
  if (!initialized_) {
    return false;
  }
  try {
    right_counter_->SetZero();
    left_counter_->SetZero();
    ResetVelocityEstimator();
    robot::support::LogI(kEncoderTag, "encoder counts reset");
    return true;
  } catch (const std::exception& exc) {
    SetFault("reset_failed", exc.what());
    return false;
  }
}

EncoderSnapshot DrivetrainEncoderSubsystem::Snapshot(uint64_t now_us) const {
  if (!initialized_) {
    throw std::runtime_error("encoder subsystem not initialized");
  }
  return ReadCounts(now_us);
}

EncoderVelocitySample DrivetrainEncoderSubsystem::PrimeVelocityEstimator(uint64_t now_us) {
  if (!initialized_) {
    throw std::runtime_error("encoder subsystem not initialized");
  }
  const auto counts = ReadCounts(now_us);
  last_sample_t_us_ = counts.t_us;
  last_right_count_ = counts.right_count;
  last_left_count_ = counts.left_count;
  velocity_sample_ = {
      .valid = false,
      .primed = true,
      .t_us = counts.t_us,
      .dt_us = 0,
      .right_count = counts.right_count,
      .left_count = counts.left_count,
      .right_delta = 0,
      .left_delta = 0,
      .right_rpm = 0.0,
      .left_rpm = 0.0,
      .command_age_ms = 0,
  };
  return velocity_sample_;
}

EncoderVelocitySample DrivetrainEncoderSubsystem::SampleVelocity(uint64_t now_us) {
  if (!initialized_) {
    throw std::runtime_error("encoder subsystem not initialized");
  }
  try {
    const auto counts = ReadCounts(now_us);
    if (!last_sample_t_us_.has_value()) {
      return PrimeVelocityEstimator(now_us);
    }

    const uint64_t dt_us = counts.t_us - *last_sample_t_us_;
    if (dt_us == 0) {
      throw std::runtime_error("encoder dt_us must be positive");
    }

    const int32_t right_delta = counts.right_count - *last_right_count_;
    const int32_t left_delta = counts.left_count - *last_left_count_;
    const double rev_scale =
        kRevScaleNumerator /
        (robot::control::EncoderGeometry::kEncoderCpr *
         robot::control::EncoderGeometry::kGearRatio * static_cast<double>(dt_us));

    last_sample_t_us_ = counts.t_us;
    last_right_count_ = counts.right_count;
    last_left_count_ = counts.left_count;
    velocity_sample_ = {
        .valid = true,
        .primed = true,
        .t_us = counts.t_us,
        .dt_us = static_cast<uint32_t>(dt_us),
        .right_count = counts.right_count,
        .left_count = counts.left_count,
        .right_delta = right_delta,
        .left_delta = left_delta,
        .right_rpm = right_delta * rev_scale,
        .left_rpm = left_delta * rev_scale,
        .command_age_ms = 0,
    };
    return velocity_sample_;
  } catch (const std::exception& exc) {
    SetFault("read_failed", exc.what());
    throw;
  }
}

void DrivetrainEncoderSubsystem::Shutdown() noexcept {
  for (auto* counter : {right_counter_.get(), left_counter_.get()}) {
    if (counter == nullptr) {
      continue;
    }
    try {
      counter->Deinit();
    } catch (...) {
      robot::support::LogW(kEncoderTag, "ignoring encoder shutdown error");
    }
  }
  right_counter_.reset();
  left_counter_.reset();
  initialized_ = false;
  ResetVelocityEstimator();
}

bool DrivetrainEncoderSubsystem::Recover() {
  robot::support::LogI(kEncoderTag, "encoder recovery requested");
  if (fault_.has_value()) {
    robot::support::LogI(kEncoderTag, std::string("encoder recovery from fault code=") +
                                           fault_->code + " detail=" + fault_->message);
  }
  Shutdown();
  fault_.reset();
  const bool recovered = Initialize();
  if (recovered) {
    robot::support::LogI(kEncoderTag, "encoder recovery completed");
    if (last_fault_.has_value()) {
      robot::support::LogI(kEncoderTag, std::string("encoder recovered code=") + last_fault_->code +
                                               " detail=" + last_fault_->message);
    }
  }
  return recovered;
}

void DrivetrainEncoderSubsystem::ResetVelocityEstimator() noexcept {
  last_sample_t_us_.reset();
  last_right_count_.reset();
  last_left_count_.reset();
  velocity_sample_ = {};
}

EncoderSnapshot DrivetrainEncoderSubsystem::ReadCounts(uint64_t now_us) const {
  return {
      .t_us = now_us,
      .right_count = right_counter_->Read(),
      .left_count = left_counter_->Read(),
  };
}

void DrivetrainEncoderSubsystem::SetFault(std::string code, std::string message) {
  fault_ = {
      .code = std::move(code),
      .message = std::move(message),
  };
  last_fault_ = fault_;
  robot::support::LogE(kEncoderTag, std::string("encoder fault code=") + fault_->code +
                                       " detail=" + fault_->message);
}

}  // namespace robot::hardware
