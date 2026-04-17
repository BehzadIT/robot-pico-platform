#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "robot/control/controller_drive_mapper.hpp"
#include "robot/control/controller_uart_pipeline.hpp"
#include "robot/control/drivetrain_controller.hpp"
#include "robot/control/drive_command_manager.hpp"
#include "robot/control/pid.hpp"
#include "robot/hardware/drivetrain_encoder_subsystem.hpp"
#include "robot/hardware/encoder_counter.hpp"
#include "robot/hardware/motor_output_driver.hpp"
#include "robot/hardware/watchdog.hpp"
#include "robot/protocol/api_drive_request.hpp"
#include "robot/protocol/joystick_transport.hpp"
#include "robot/platform/controller_uart_runtime.hpp"
#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/control_core_runtime.hpp"
#include "robot/runtime/service_core_runtime.hpp"
#include "robot/runtime/shared_runtime_state.hpp"
#include "robot/support/logger.hpp"

namespace {

using robot::control::ControllerDriveConfig;
using robot::control::ControllerDriveMapper;
using robot::control::ControllerCommandKind;
using robot::control::ControllerUartPipeline;
using robot::control::DrivetrainController;
using robot::control::DriveCommandManager;
using robot::control::Pid;
using robot::control::PidLimits;
using robot::hardware::DrivetrainEncoderSubsystem;
using robot::hardware::EncoderCounter;
using robot::hardware::EncoderCounterConfig;
using robot::hardware::NullMotorOutputDriver;
using robot::hardware::NullWatchdog;
using robot::protocol::ApiDriveRequest;
using robot::protocol::JoystickFrameParser;
using robot::protocol::JoystickSnapshot;
using robot::platform::ControllerUartRuntime;
using robot::runtime::Mailbox;
using robot::runtime::SharedRuntimeState;
using robot::runtime::ServiceCoreRuntime;
using robot::runtime::ControlCoreRuntime;
using robot::support::DistinctFilter;
using robot::support::LogD;
using robot::support::LogI;
using robot::support::LogLevel;
using robot::support::RateLimitFilter;
using robot::support::SetGlobalLogLevel;
using robot::support::SetLogSink;
using robot::support::SetTickProvider;
using robot::support::ResetLogSink;
using robot::support::ResetTickProvider;

std::string g_captured_logs;

void CaptureSink(const char* line) {
  g_captured_logs += line == nullptr ? "" : line;
}

uint32_t TestTicks() {
  return 42;
}

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    std::exit(1);
  }
}

void RequireNear(double actual, double expected, double epsilon, const char* message) {
  if (std::fabs(actual - expected) > epsilon) {
    std::cerr << "FAIL: " << message << " expected=" << expected
              << " actual=" << actual << "\n";
    std::exit(1);
  }
}

JoystickSnapshot MakeSnapshot(bool connected = true,
                              uint16_t left_trigger = 0,
                              uint16_t right_trigger = 0,
                              int16_t left_stick_x = 0,
                              uint32_t seq = 1,
                              uint32_t received_at_ms = 0) {
  JoystickSnapshot snapshot;
  snapshot.seq = seq;
  snapshot.source_uptime_ms = received_at_ms;
  snapshot.connected = connected;
  snapshot.left_trigger = left_trigger;
  snapshot.right_trigger = right_trigger;
  snapshot.left_stick_x = left_stick_x;
  snapshot.received_at_ms = received_at_ms;
  return snapshot;
}

std::vector<uint8_t> BuildFrame(const JoystickSnapshot& snapshot) {
  std::vector<uint8_t> frame(robot::protocol::kFrameLen, 0);
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = robot::protocol::kFrameVersion;
  frame[3] = robot::protocol::kFrameTypeSnapshot;
  frame[4] = static_cast<uint8_t>(robot::protocol::kSnapshotPayloadLen & 0xFF);
  frame[5] =
      static_cast<uint8_t>((robot::protocol::kSnapshotPayloadLen >> 8) & 0xFF);

  auto write_u16 = [&](std::size_t offset, uint16_t value) {
    frame[offset] = static_cast<uint8_t>(value & 0xFF);
    frame[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  };
  auto write_u32 = [&](std::size_t offset, uint32_t value) {
    frame[offset] = static_cast<uint8_t>(value & 0xFF);
    frame[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
  };

  constexpr std::size_t payload = robot::protocol::kFrameHeaderLen;
  write_u32(payload + 0, snapshot.seq);
  write_u32(payload + 4, snapshot.source_uptime_ms);
  frame[payload + 8] = snapshot.connected ? 0x01 : 0x00;
  frame[payload + 9] = snapshot.dpad;
  write_u16(payload + 10, snapshot.buttons);
  frame[payload + 12] = snapshot.misc_buttons;
  frame[payload + 13] = 0;
  write_u16(payload + 14, static_cast<uint16_t>(snapshot.left_stick_x));
  write_u16(payload + 16, static_cast<uint16_t>(snapshot.left_stick_y));
  write_u16(payload + 18, static_cast<uint16_t>(snapshot.right_stick_x));
  write_u16(payload + 20, static_cast<uint16_t>(snapshot.right_stick_y));
  write_u16(payload + 22, snapshot.left_trigger);
  write_u16(payload + 24, snapshot.right_trigger);

  const uint16_t crc =
      robot::protocol::Crc16Ccitt(frame.data() + 2, robot::protocol::kFrameLen - 4);
  write_u16(robot::protocol::kFrameLen - 2, crc);
  return frame;
}

class TestEncoderCounter final : public EncoderCounter {
 public:
  [[nodiscard]] int32_t Read() const override { return count_; }
  void SetZero() override { count_ = 0; }
  void Deinit() override {}
  void SetCount(int32_t count) { count_ = count; }

 private:
  int32_t count_{0};
};

void TestControllerDriveMapper() {
  ControllerDriveMapper mapper(ControllerDriveConfig{});

  auto forward_snapshot = MakeSnapshot(true, 0, 560);
  auto forward = mapper.MapSnapshot(&forward_snapshot, 0);
  Require(forward.mode == "driving", "right trigger should drive forward");
  Require(forward.throttle_percent > 0, "forward throttle should be positive");

  auto reverse_snapshot = MakeSnapshot(true, 560, 0);
  auto reverse = mapper.MapSnapshot(&reverse_snapshot, 0);
  Require(reverse.mode == "driving", "left trigger should drive reverse");
  Require(reverse.throttle_percent < 0, "reverse throttle should be negative");

  auto equal_snapshot = MakeSnapshot(true, 400, 400);
  auto equal = mapper.MapSnapshot(&equal_snapshot, 0);
  Require(equal.mode == "armed_neutral", "equal triggers should be neutral");

  auto active_snapshot = MakeSnapshot(true, 0, 140);
  auto hold_snapshot = MakeSnapshot(true, 0, 100, 0, 2);
  auto release_snapshot = MakeSnapshot(true, 0, 70, 0, 3);
  auto active = mapper.MapSnapshot(&active_snapshot, 0);
  auto hold = mapper.MapSnapshot(&hold_snapshot, 10);
  auto release = mapper.MapSnapshot(&release_snapshot, 20);
  Require(active.mode == "driving", "press threshold should activate");
  Require(hold.mode == "driving", "release threshold should hold");
  Require(release.mode == "armed_neutral", "release threshold should stop");

  auto deadzone_snapshot = MakeSnapshot(true, 0, 300, 180);
  auto deadzone = mapper.MapSnapshot(&deadzone_snapshot, 0);
  Require(deadzone.turn_percent == 0, "small stick noise should deadzone");

  auto stale_snapshot = MakeSnapshot(true, 0, 500, 0, 1, 0);
  auto stale = mapper.MapSnapshot(&stale_snapshot, 300);
  Require(stale.mode == "inactive", "stale snapshot should go inactive");
  Require(stale.reason == "snapshot_stale", "stale reason should match");
}

void TestApiDriveRequest() {
  auto request = ApiDriveRequest::FromIntent(50, -50, 7, 80);
  Require(request.target_rpm > 0, "intent should map to rpm");
  Require(request.target_direction == 0, "positive throttle should be forward");
  Require(request.target_angle < 0, "negative turn should map left");
  Require(request.seq == std::optional<int>(7), "seq should be preserved");

  auto stop = ApiDriveRequest::FromIntent(1, 0);
  Require(stop.target_rpm == 0, "low rpm intent should normalize to stop");
}

void TestPid() {
  Pid pid_us(0.0, 2.0, 0.0, 1.0, {std::nullopt, std::nullopt},
             {std::nullopt, std::nullopt});
  RequireNear(pid_us.Update(0.0, 0.05), 0.1, 1e-9,
              "explicit dt should scale integral in seconds");

  Pid pid_limited(0.0, 1.0, 0.0, 1.0, {-1.0, 1.0}, {-0.2, 0.2});
  const auto ignored_output = pid_limited.Update(0.0, 1.0);
  (void)ignored_output;
  const auto output = pid_limited.Update(0.0, 1.0);
  RequireNear(output, 0.2, 1e-9, "integral limits should be independent");

  Pid pid_seeded(0.0, 1.0, 0.0, 0.0, {-1.0, 1.0}, {-0.25, 0.25});
  pid_seeded.SetAutoMode(false);
  pid_seeded.SetAutoMode(true, 0.9);
  Require(pid_seeded.auto_mode(), "pid should re-enter auto mode");
  RequireNear(std::get<1>(pid_seeded.components()), 0.25, 1e-9,
              "auto mode seed should clamp to integral limits");
}

void TestDriveCommandManager() {
  DriveCommandManager manager(150);

  auto drive_a = ApiDriveRequest::FromIntent(50, 0, 10, 100);
  auto result_a = manager.AcceptDriveRequest(drive_a, 100, std::string("ws-a"));
  Require(result_a.ok, "initial drive request should be accepted");
  Require(manager.GetDriveTarget(100).target_rpm > 0,
          "accepted drive should update target store");

  auto stale = ApiDriveRequest::FromIntent(60, 0, 10, 120);
  auto stale_result = manager.AcceptDriveRequest(stale, 120, std::string("ws-a"));
  Require(!stale_result.ok && stale_result.code == "st",
          "duplicate sequence should be rejected as stale");

  auto foreign = ApiDriveRequest::FromIntent(70, 0, 11, 130);
  auto foreign_result =
      manager.AcceptDriveRequest(foreign, 130, std::string("ws-b"));
  Require(!foreign_result.ok && foreign_result.code == "ic",
          "foreign command source should be rejected");

  const auto timeout_snapshot = manager.TimeoutStop(701, 500);
  Require(timeout_snapshot.has_value(), "timeout should stop active drive");
  Require(timeout_snapshot->last_seq == 10, "timeout snapshot should retain seq");
  Require(!manager.IsFreshSequence(10), "timeout fence should reject old seq");

  auto guarded = ApiDriveRequest::FromIntent(40, 0, 12, 710);
  auto guarded_result =
      manager.AcceptDriveRequest(guarded, 710, std::string("ws-a"));
  Require(!guarded_result.ok && guarded_result.code == "si",
          "timeout guard window should reject immediate restart");

  auto resumed_result =
      manager.AcceptDriveRequest(guarded, 900, std::string("ws-a"));
  Require(resumed_result.ok, "drive should resume after timeout guard window");

  Require(manager.Stop(901, std::string("ws-a")),
          "active source stop should clear accepted motion intent");
  Require(manager.GetDriveTarget(901).target_rpm == 0,
          "stop should clear accepted target at the admission layer");
  Require(!manager.ClearCommandSourceIfActive("ws-a", 901),
          "stop should already clear the active command source");

  DriveCommandManager disconnect_manager(150);
  auto disconnect_drive = ApiDriveRequest::FromIntent(35, 0, 13, 950);
  auto disconnect_result =
      disconnect_manager.AcceptDriveRequest(disconnect_drive, 950, std::string("ws-a"));
  Require(disconnect_result.ok, "fresh manager should accept disconnect test drive");
  Require(disconnect_manager.ClearCommandSourceIfActive("ws-a", 951),
          "active command source should be clearable on disconnect-style events");
  Require(!disconnect_manager.ClearCommandSourceIfActive("ws-a", 952),
          "clearing inactive source should report false");
}

void TestControllerUartPipeline() {
  ControllerUartPipeline pipeline(ControllerDriveConfig{});
  JoystickSnapshot snapshot = MakeSnapshot(true, 0, 560, 700, 20, 0);
  auto frame = BuildFrame(snapshot);
  pipeline.FeedBytes(frame.data(), frame.size(), 0);

  const auto first = pipeline.Tick(0);
  Require(first.has_value(), "first driving snapshot should publish");
  Require(first->kind == ControllerCommandKind::kDrive,
          "driving snapshot should emit drive command");
  Require(first->drive_request.throttle_percent > 0,
          "drive command should carry positive throttle");

  const auto suppressed = pipeline.Tick(10);
  Require(!suppressed.has_value(), "publish cadence should suppress early republish");

  const auto republished = pipeline.Tick(60);
  Require(republished.has_value(), "publish cadence should republish while driving");
  Require(republished->kind == ControllerCommandKind::kDrive,
          "republish should stay a drive command");

  const auto stale_stop = pipeline.Tick(400);
  Require(stale_stop.has_value(), "stale input should trigger stop command");
  Require(stale_stop->kind == ControllerCommandKind::kStop,
          "stale input should emit stop command");
  Require(stale_stop->reason == "snapshot_stale",
          "stale stop reason should be preserved");
}

void TestControllerUartRuntime() {
  ControllerUartRuntime runtime(robot::config::ControllerUartConfig{});
  Require(runtime.enabled(), "controller UART runtime should preserve enabled flag");
  Require(runtime.IsHardwareConfigValid(), "default controller UART config should be valid");
  Require(runtime.uart_id() == 1, "default controller UART id should be preserved");
  Require(runtime.baud() == 115200, "default controller UART baud should be preserved");
  Require(runtime.tx_pin() == 4 && runtime.rx_pin() == 5,
          "default controller UART pins should be preserved");

  Require(runtime.ShouldPoll(0), "first controller UART poll should emit immediately");
  Require(!runtime.ShouldPoll(5), "controller UART poll should respect cadence");
  Require(runtime.ShouldPoll(10), "controller UART poll should reopen after cadence");

  Require(runtime.ShouldSample(0), "first controller UART sample should emit immediately");
  Require(!runtime.ShouldSample(250), "controller UART sample should respect diag cadence");
  Require(runtime.ShouldSample(500), "controller UART sample should reopen after cadence");

  robot::config::ControllerUartConfig invalid{};
  invalid.uart_id = 2;
  ControllerUartRuntime invalid_runtime(invalid);
  Require(!invalid_runtime.IsHardwareConfigValid(),
          "unsupported controller UART id should be marked invalid");
  Require(!invalid_runtime.ShouldPoll(0),
          "invalid controller UART config should suppress polling");
}

void TestMailbox() {
  Mailbox<int, 2> mailbox;
  Require(mailbox.Push(1), "first mailbox push should succeed");
  Require(mailbox.Push(2), "second mailbox push should succeed");
  Require(!mailbox.Push(3), "mailbox should reject overflow");
  Require(mailbox.ReplaceLatest(5), "mailbox should replace latest entry");
  const auto first = mailbox.Pop();
  const auto second = mailbox.Pop();
  Require(first.has_value() && *first == 1, "mailbox first pop should preserve fifo");
  Require(second.has_value() && *second == 5,
          "mailbox replace-latest should update newest element");
  Require(!mailbox.Pop().has_value(), "mailbox should become empty");
}

void TestLogger() {
  g_captured_logs.clear();
  SetLogSink(&CaptureSink);
  SetTickProvider(&TestTicks);
  SetGlobalLogLevel(LogLevel::kInfo);

  LogI("boot", "stdio initialized");
  Require(g_captured_logs == "I (42) boot: stdio initialized\n",
          "logger should format info lines with tag and ticks");

  g_captured_logs.clear();
  LogI("", "plain message");
  Require(g_captured_logs == "I (42) plain message\n",
          "logger should omit empty tag segment");

  g_captured_logs.clear();
  LogD("boot", "hidden debug");
  Require(g_captured_logs.empty(), "logger should filter by global log level");

  g_captured_logs.clear();
  DistinctFilter distinct;
  Require(distinct.ShouldEmit("idle"), "distinct filter should emit first value");
  Require(!distinct.ShouldEmit("idle"), "distinct filter should suppress unchanged value");
  Require(distinct.ShouldEmit("running"), "distinct filter should emit changed value");

  RateLimitFilter rate_limit(100);
  Require(rate_limit.ShouldEmit(0), "rate limit filter should emit first value");
  Require(!rate_limit.ShouldEmit(50), "rate limit filter should suppress too-soon repeat");
  Require(rate_limit.ShouldEmit(100), "rate limit filter should emit after cooldown");

  ResetLogSink();
  ResetTickProvider();
  SetGlobalLogLevel(LogLevel::kInfo);
}

void TestServiceControlRuntime() {
  SharedRuntimeState shared_state;
  ServiceCoreRuntime service_core(shared_state);
  ControlCoreRuntime control_core(shared_state);

  const auto drive = ApiDriveRequest::FromIntent(45, 10, 41, 100);
  Require(service_core.QueueDrive(drive, "ws-a"),
          "service core should enqueue drive command");
  control_core.Tick(100);
  const auto drive_completion = service_core.PollCompletion();
  Require(drive_completion.has_value() && drive_completion->ok,
          "control core should accept queued drive command");
  Require(control_core.status(100).drive_target.target_rpm > 0,
          "accepted transport drive should update control target");

  Require(service_core.QueueStop(42, "ws-a"),
          "service core should enqueue stop command");
  control_core.Tick(110);
  const auto stop_completion = service_core.PollCompletion();
  Require(stop_completion.has_value() && stop_completion->ok,
          "control core should complete stop command");

  const auto disconnect_source = std::string("ws-7");
  const auto disconnect_drive = ApiDriveRequest::FromIntent(55, 0, 43, 120);
  Require(service_core.QueueDrive(disconnect_drive, disconnect_source),
          "service core should enqueue disconnect test drive");
  control_core.Tick(120);
  const auto disconnect_drive_completion = service_core.PollCompletion();
  Require(disconnect_drive_completion.has_value() && disconnect_drive_completion->ok,
          "disconnect test drive should be accepted before cleanup");

  Require(service_core.QueueDisconnect(disconnect_source),
          "service core should enqueue disconnect cleanup");
  control_core.Tick(130);
  const auto disconnect_stop_completion = service_core.PollCompletion();
  Require(disconnect_stop_completion.has_value() && disconnect_stop_completion->ok,
          "disconnect cleanup should stop active motion");
  Require(disconnect_stop_completion->code == "s",
          "disconnect cleanup should flow through the normal stop code");

  const auto reconnect_drive = ApiDriveRequest::FromIntent(45, 0, 44, 140);
  Require(service_core.QueueDrive(reconnect_drive, disconnect_source),
          "service core should accept a new drive after disconnect cleanup");
  control_core.Tick(140);
  const auto reconnect_completion = service_core.PollCompletion();
  Require(reconnect_completion.has_value() && reconnect_completion->ok,
          "drive should be accepted after disconnect cleanup");

  JoystickSnapshot snapshot = MakeSnapshot(true, 0, 560, 500, 90, 120);
  auto frame = BuildFrame(snapshot);
  control_core.FeedControllerBytes(frame.data(), frame.size(), 120);
  control_core.Tick(120);
  Require(control_core.status(120).controller.driving,
          "control core should own live controller UART path");
  Require(control_core.status(120).drive_target.target_rpm > 0,
          "controller-owned UART path should be able to re-arm motion");
}

void TestDrivetrainController() {
  TestEncoderCounter* right_counter = nullptr;
  TestEncoderCounter* left_counter = nullptr;
  DrivetrainEncoderSubsystem encoders(
      [&](const EncoderCounterConfig& config) -> std::unique_ptr<EncoderCounter> {
        auto counter = std::make_unique<TestEncoderCounter>();
        if (config.state_machine_id == 0) {
          right_counter = counter.get();
        } else {
          left_counter = counter.get();
        }
        return counter;
      });
  NullMotorOutputDriver motor_driver;
  NullWatchdog watchdog;
  DrivetrainController controller(encoders, motor_driver, watchdog);
  DriveCommandManager manager;

  auto drive = ApiDriveRequest::FromIntent(50, 0, 88, 0);
  Require(manager.AcceptDriveRequest(drive, 0, std::string("bench")).ok,
          "bench drive request should be accepted");
  Require(controller.OnDriveAccepted().ok,
          "accepted drive should arm the drivetrain state machine");
  Require(!controller.Tick(manager, 0).has_value(),
          "first drivetrain tick should prepare running state");

  right_counter->SetCount(64);
  left_counter->SetCount(64);
  Require(!controller.Tick(manager, 20).has_value(),
          "running tick should apply control without generating completion");
  const auto running_status = controller.status(manager.GetDriveTarget(20));
  Require(running_status.current_pwm_left > 0 && running_status.current_pwm_right > 0,
          "valid encoder sample should drive non-zero pwm");

  const auto stop_request = controller.RequestStop(90, "bench_stop");
  Require(stop_request.ok && !stop_request.completed,
          "active drivetrain stop should complete in the controller tick");
  const auto stop_event = controller.Tick(manager, 40);
  Require(stop_event.has_value(), "stop cycle should emit a completion event");
  Require(stop_event->kind == robot::control::DrivetrainEvent::Kind::kStopCompleted,
          "stop cycle should emit stop completion");
  Require(stop_event->seq == std::optional<int>(90),
          "stop completion should preserve sequence");

  const auto recover_request = controller.RequestRecover(91);
  Require(recover_request.ok, "idle drivetrain should accept recovery");
  const auto recover_event = controller.Tick(manager, 60);
  Require(recover_event.has_value(), "recover cycle should emit completion");
  Require(recover_event->kind == robot::control::DrivetrainEvent::Kind::kRecoverCompleted,
          "recover cycle should complete successfully");
}

void TestJoystickParser() {
  JoystickFrameParser parser;
  JoystickSnapshot snapshot = MakeSnapshot(true, 100, 400, 320, 9, 44);
  auto frame = BuildFrame(snapshot);

  std::vector<uint8_t> prefixed = {0x00, 0x11};
  prefixed.insert(prefixed.end(), frame.begin(), frame.end());
  const auto parsed = parser.Feed(prefixed.data(), prefixed.size(), 77);

  Require(parsed.size() == 1, "parser should resync and decode one frame");
  Require(parsed[0].seq == 9, "decoded seq should match");
  Require(parsed[0].right_trigger == 400, "decoded trigger should match");
  Require(parsed[0].received_at_ms == 77, "receive timestamp should be injected");
}

}  // namespace

int main() {
  TestControllerDriveMapper();
  TestApiDriveRequest();
  TestPid();
  TestDriveCommandManager();
  TestControllerUartPipeline();
  TestControllerUartRuntime();
  TestMailbox();
  TestLogger();
  TestServiceControlRuntime();
  TestDrivetrainController();
  TestJoystickParser();
  std::cout << "robot_core_tests: ok\n";
  return 0;
}
