#include "robot/control/controller_uart_pipeline.hpp"

#include <array>
#include <cstdlib>
#include <string>
#include <utility>

#include "robot/support/logger.hpp"

namespace robot::control {

namespace {

constexpr const char* kControllerTag = "controller_drive";
constexpr std::array<std::pair<uint16_t, const char*>, 10> kButtonNames = {{
    {0x0001, "A"},
    {0x0002, "B"},
    {0x0004, "X"},
    {0x0008, "Y"},
    {0x0010, "Left Bumper"},
    {0x0020, "Right Bumper"},
    {0x0040, "Left Trigger Button"},
    {0x0080, "Right Trigger Button"},
    {0x0100, "Left Stick Press"},
    {0x0200, "Right Stick Press"},
}};

constexpr std::array<std::pair<uint8_t, const char*>, 4> kMiscButtonNames = {{
    {0x01, "Xbox"},
    {0x02, "View"},
    {0x04, "Menu"},
    {0x08, "Capture"},
}};

constexpr std::array<std::pair<uint8_t, const char*>, 4> kDpadNames = {{
    {0x01, "D-Pad Up"},
    {0x02, "D-Pad Down"},
    {0x04, "D-Pad Right"},
    {0x08, "D-Pad Left"},
}};

bool ChangedBeyondThreshold(int16_t previous_value, int16_t current_value, int threshold) {
  if (previous_value == current_value) {
    return false;
  }
  if (std::abs(static_cast<int>(current_value) - static_cast<int>(previous_value)) >= threshold) {
    return true;
  }
  const bool previous_active = std::abs(static_cast<int>(previous_value)) >= threshold;
  const bool current_active = std::abs(static_cast<int>(current_value)) >= threshold;
  return previous_active != current_active;
}

bool TriggerChanged(uint16_t previous_value, uint16_t current_value, int threshold) {
  if (previous_value == current_value) {
    return false;
  }
  if (std::abs(static_cast<int>(current_value) - static_cast<int>(previous_value)) >= threshold) {
    return true;
  }
  const bool previous_active = previous_value >= threshold;
  const bool current_active = current_value >= threshold;
  return previous_active != current_active;
}

template <typename MaskT, std::size_t N>
std::string MaskedNames(MaskT previous_mask,
                        MaskT current_mask,
                        const std::array<std::pair<MaskT, const char*>, N>& names,
                        bool pressed) {
  std::string out = "[";
  bool first = true;
  for (const auto& [mask, name] : names) {
    const bool was_set = (previous_mask & mask) != 0U;
    const bool is_set = (current_mask & mask) != 0U;
    const bool include = pressed ? (!was_set && is_set) : (was_set && !is_set);
    if (!include) {
      continue;
    }
    if (!first) {
      out.push_back(',');
    }
    first = false;
    out += name;
  }
  out.push_back(']');
  return out;
}

std::string BuildInputChangeMessage(const std::optional<robot::protocol::JoystickSnapshot>& previous,
                                    const robot::protocol::JoystickSnapshot& current) {
  std::string out;
  if (!previous.has_value()) {
    if (!current.connected) {
      out += " connected=False";
    }
  } else {
    if (previous->connected != current.connected) {
      out += " connected=";
      out += current.connected ? "True" : "False";
    }
  }

  const auto& prev = previous.has_value() ? *previous : robot::protocol::JoystickSnapshot{};
  if (!previous.has_value() || prev.buttons != current.buttons) {
    const auto pressed = MaskedNames(prev.buttons, current.buttons, kButtonNames, true);
    const auto released = MaskedNames(prev.buttons, current.buttons, kButtonNames, false);
    if (pressed != "[]" || released != "[]") {
      out += " pressed=";
      out += pressed;
      out += " released=";
      out += released;
    }
  }
  if (!previous.has_value() || prev.misc_buttons != current.misc_buttons) {
    const auto pressed = MaskedNames(prev.misc_buttons, current.misc_buttons, kMiscButtonNames, true);
    const auto released =
        MaskedNames(prev.misc_buttons, current.misc_buttons, kMiscButtonNames, false);
    if (pressed != "[]" || released != "[]") {
      out += " misc_pressed=";
      out += pressed;
      out += " misc_released=";
      out += released;
    }
  }
  if (!previous.has_value() || prev.dpad != current.dpad) {
    const auto pressed = MaskedNames(prev.dpad, current.dpad, kDpadNames, true);
    const auto released = MaskedNames(prev.dpad, current.dpad, kDpadNames, false);
    if (pressed != "[]" || released != "[]") {
      out += " dpad_pressed=";
      out += pressed;
      out += " dpad_released=";
      out += released;
    }
  }
  if (!previous.has_value() || ChangedBeyondThreshold(prev.left_stick_x, current.left_stick_x, 20) ||
      ChangedBeyondThreshold(prev.left_stick_y, current.left_stick_y, 20)) {
    if (current.left_stick_x != 0 || current.left_stick_y != 0 || !previous.has_value()) {
      out += " left_stick(x=";
      out += std::to_string(current.left_stick_x);
      out += " y=";
      out += std::to_string(current.left_stick_y);
      out += ")";
    }
  }
  if (!previous.has_value() || ChangedBeyondThreshold(prev.right_stick_x, current.right_stick_x, 20) ||
      ChangedBeyondThreshold(prev.right_stick_y, current.right_stick_y, 20)) {
    if (current.right_stick_x != 0 || current.right_stick_y != 0 || !previous.has_value()) {
      out += " right_stick(x=";
      out += std::to_string(current.right_stick_x);
      out += " y=";
      out += std::to_string(current.right_stick_y);
      out += ")";
    }
  }
  if (!previous.has_value() || TriggerChanged(prev.left_trigger, current.left_trigger, 8)) {
    if (current.left_trigger != 0 || !previous.has_value()) {
      out += " left_trigger=";
      out += std::to_string(current.left_trigger);
    }
  }
  if (!previous.has_value() || TriggerChanged(prev.right_trigger, current.right_trigger, 8)) {
    if (current.right_trigger != 0 || !previous.has_value()) {
      out += " right_trigger=";
      out += std::to_string(current.right_trigger);
    }
  }
  return out;
}

void LogSnapshotChange(const std::optional<robot::protocol::JoystickSnapshot>& previous,
                       const robot::protocol::JoystickSnapshot& current,
                       uint32_t now_ms) {
  const std::string change = BuildInputChangeMessage(previous, current);
  if (change.empty()) {
    return;
  }
  const int age_ms = static_cast<int>(now_ms - current.received_at_ms);
  robot::support::LogI(kControllerTag,
                       std::string("joystick input seq=") + std::to_string(current.seq) + change +
                           " age_ms=" + std::to_string(age_ms));
}

}  // namespace

ControllerUartPipeline::ControllerUartPipeline(ControllerDriveConfig config)
    : config_(config), mapper_(config) {}

void ControllerUartPipeline::FeedBytes(const uint8_t* chunk,
                                       std::size_t length,
                                       uint32_t now_ms) {
  const auto snapshots = parser_.Feed(chunk, length, now_ms);
  for (const auto& snapshot : snapshots) {
    LogSnapshotChange(latest_snapshot_, snapshot, now_ms);
    latest_snapshot_ = snapshot;
    ++frames_ok_;
  }
}

std::optional<ControllerCommand> ControllerUartPipeline::Tick(uint32_t now_ms) {
  const auto mapped =
      mapper_.MapSnapshot(latest_snapshot_ ? &(*latest_snapshot_) : nullptr, now_ms);

  last_mode_ = mapped.mode;
  last_age_ms_ = mapped.age_ms;
  last_throttle_percent_ = mapped.throttle_percent;
  last_turn_percent_ = mapped.turn_percent;

  if (mapped.mode == "driving") {
    const bool should_publish =
        !driving_ || (now_ms - last_publish_ms_ >= static_cast<uint32_t>(config_.publish_period_ms));
    driving_ = true;
    if (!should_publish) {
      return std::nullopt;
    }

    last_publish_ms_ = now_ms;
    ControllerCommand command;
    command.kind = ControllerCommandKind::kDrive;
    command.mode = mapped.mode;
    command.snapshot_seq = mapped.snapshot_seq;
    command.age_ms = mapped.age_ms;
    command.reason = driving_ ? "controller_drive" : "controller_start";
    command.drive_request = robot::protocol::ApiDriveRequest::FromIntent(
        mapped.throttle_percent,
        mapped.turn_percent,
        NextDriveSeq(mapped.snapshot_seq, now_ms),
        static_cast<int>(now_ms));
    return command;
  }

  if (driving_) {
    driving_ = false;
    last_stop_reason_ = mapped.reason;
    ControllerCommand command;
    command.kind = ControllerCommandKind::kStop;
    command.mode = mapped.mode;
    command.snapshot_seq = mapped.snapshot_seq;
    command.age_ms = mapped.age_ms;
    command.reason = mapped.reason.empty() ? "controller_stop" : mapped.reason;
    return command;
  }

  if (!mapped.reason.empty()) {
    last_stop_reason_ = mapped.reason;
  }
  return std::nullopt;
}

ControllerPipelineStatus ControllerUartPipeline::status() const {
  ControllerPipelineStatus snapshot;
  snapshot.has_snapshot = latest_snapshot_.has_value();
  snapshot.driving = driving_;
  snapshot.mode = last_mode_;
  snapshot.last_stop_reason = last_stop_reason_;
  snapshot.last_snapshot_seq = latest_snapshot_.has_value() ? latest_snapshot_->seq : 0;
  snapshot.last_source_uptime_ms =
      latest_snapshot_.has_value() ? latest_snapshot_->source_uptime_ms : 0;
  snapshot.last_receive_ms = latest_snapshot_.has_value() ? latest_snapshot_->received_at_ms : 0;
  snapshot.connected = latest_snapshot_.has_value() ? latest_snapshot_->connected : false;
  snapshot.dpad = latest_snapshot_.has_value() ? latest_snapshot_->dpad : 0;
  snapshot.buttons = latest_snapshot_.has_value() ? latest_snapshot_->buttons : 0;
  snapshot.misc_buttons = latest_snapshot_.has_value() ? latest_snapshot_->misc_buttons : 0;
  snapshot.left_stick_x = latest_snapshot_.has_value() ? latest_snapshot_->left_stick_x : 0;
  snapshot.left_stick_y = latest_snapshot_.has_value() ? latest_snapshot_->left_stick_y : 0;
  snapshot.right_stick_x = latest_snapshot_.has_value() ? latest_snapshot_->right_stick_x : 0;
  snapshot.right_stick_y = latest_snapshot_.has_value() ? latest_snapshot_->right_stick_y : 0;
  snapshot.left_trigger = latest_snapshot_.has_value() ? latest_snapshot_->left_trigger : 0;
  snapshot.right_trigger = latest_snapshot_.has_value() ? latest_snapshot_->right_trigger : 0;
  snapshot.last_throttle_percent = last_throttle_percent_;
  snapshot.last_turn_percent = last_turn_percent_;
  snapshot.last_age_ms = last_age_ms_;
  snapshot.frames_ok = frames_ok_;
  return snapshot;
}

int ControllerUartPipeline::NextDriveSeq(uint32_t snapshot_seq,
                                         uint32_t now_ms) noexcept {
  const int candidate = static_cast<int>((snapshot_seq > now_ms ? snapshot_seq : now_ms) + 1U);
  if (candidate <= last_emitted_seq_) {
    ++last_emitted_seq_;
  } else {
    last_emitted_seq_ = candidate;
  }
  return last_emitted_seq_;
}

}  // namespace robot::control
