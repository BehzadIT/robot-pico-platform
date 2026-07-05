#include "robot/app/pico_firmware_app.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <string>

#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "task.h"
#include "robot/config/config.hpp"
#include "robot/hardware/pico_motor_output_driver.hpp"
#include "robot/hardware/pico_quadrature_counter.hpp"
#include "robot/platform/pico_watchdog.hpp"
#include "robot/support/logger.hpp"

namespace robot::app {

namespace {

constexpr const char* kBootTag = "boot";
constexpr const char* kControllerTag = "controller_drive";
constexpr const char* kControlTag = "control";
constexpr uint32_t kBootStageInit = 0;
constexpr uint32_t kBootStageStdioReady = 1;
constexpr uint32_t kBootStageControllerUartReady = 2;
constexpr uint32_t kBootStageServiceStartRequested = 3;
constexpr uint32_t kBootStageControlStartRequested = 4;
constexpr uint32_t kBootStageSchedulerStarting = 5;
constexpr std::size_t kMaxControllerReadChunk = 256;
constexpr TickType_t kControlLoopPeriod = pdMS_TO_TICKS(5);
constexpr UBaseType_t kControlTaskPriority = 4;
constexpr UBaseType_t kControlCoreAffinityMask = 1U << 1;
constexpr configSTACK_DEPTH_TYPE kControlTaskStackWords = 1024;

robot::hardware::EncoderCounterFactory MakePicoCounterFactory() {
  return [](const robot::hardware::EncoderCounterConfig& config) {
    return std::make_unique<robot::hardware::PicoQuadratureCounter>(
        config.state_machine_id, config.pin_a, config.pin_b);
  };
}

uint32_t CurrentTicksMs() {
  return to_ms_since_boot(get_absolute_time());
}

std::string Hex32(uint32_t value) {
  char buffer[11];
  std::snprintf(buffer, sizeof(buffer), "0x%08lx", static_cast<unsigned long>(value));
  return std::string(buffer);
}

std::string StageName(uint32_t stage) {
  switch (stage) {
    case kBootStageInit:
      return "init";
    case kBootStageStdioReady:
      return "stdio_ready";
    case kBootStageControllerUartReady:
      return "controller_uart_ready";
    case kBootStageServiceStartRequested:
      return "service_start_requested";
    case kBootStageControlStartRequested:
      return "control_start_requested";
    case kBootStageSchedulerStarting:
      return "scheduler_starting";
    default:
      return "unknown";
  }
}

void RecordBootStage(uint32_t stage, const char* label) {
  watchdog_hw->scratch[0] = stage;
  robot::support::LogI(kBootTag,
                       std::string("boot stage=") + label + " code=" + std::to_string(stage));
}

void LogResetContext() {
  const bool watchdog_reboot = watchdog_caused_reboot();
  const bool watchdog_enable_reboot = watchdog_enable_caused_reboot();
  const uint32_t watchdog_reason = watchdog_hw->reason;
  const uint32_t last_stage = watchdog_hw->scratch[0];
  const uint32_t last_wifi_stage = watchdog_hw->scratch[1];
  robot::support::LogI(
      kBootTag,
      std::string("reset context watchdog_caused=") + (watchdog_reboot ? "true" : "false") +
          " watchdog_enable_caused=" + (watchdog_enable_reboot ? "true" : "false") +
          " reason=" + Hex32(watchdog_reason) + " last_stage=" +
          std::to_string(last_stage) + " (" + StageName(last_stage) + ")" +
          " wifi_stage=" + std::to_string(last_wifi_stage));
}

}  // namespace

PicoFirmwareApp::PicoFirmwareApp()
    : control_core_runtime_(shared_state_, MakePicoCounterFactory(),
                            std::make_unique<robot::hardware::PicoMotorOutputDriver>(),
                            std::make_unique<robot::platform::PicoWatchdog>()),
      service_core_(service_core_runtime_) {}

void PicoFirmwareApp::Run() {
  stdio_init_all();
  robot::support::SetTickProvider(&CurrentTicksMs);
  LogResetContext();
  robot::support::LogI(kBootTag, "stdio initialized");
  RecordBootStage(kBootStageStdioReady, "stdio_initialized");
  InitializeControllerUart();
  RecordBootStage(kBootStageControllerUartReady, "controller_uart_initialized");
  robot::support::LogI(kBootTag, "launching service task on core 0");
  RecordBootStage(kBootStageServiceStartRequested, "service_start_requested");
  if (!service_core_.Start()) {
    robot::support::LogE(kBootTag, "service task failed to start");
    configASSERT(false);
  }
  robot::support::LogI(kBootTag, "launching control task on core 1");
  RecordBootStage(kBootStageControlStartRequested, "control_start_requested");
  const BaseType_t control_task_result = xTaskCreateAffinitySet(
      &PicoFirmwareApp::ControlTaskEntry, "ctrl-core1", kControlTaskStackWords, this,
      kControlTaskPriority, kControlCoreAffinityMask, nullptr);
  if (control_task_result != pdPASS) {
    robot::support::LogE(kBootTag, "control task failed to start");
    configASSERT(false);
  }
  robot::support::LogI(kBootTag, "starting scheduler");
  RecordBootStage(kBootStageSchedulerStarting, "scheduler_starting");
  vTaskStartScheduler();
  robot::support::LogE(kBootTag, "scheduler returned unexpectedly");
}

void PicoFirmwareApp::ControlTaskEntry(void* context) {
  auto* app = static_cast<PicoFirmwareApp*>(context);
  configASSERT(app != nullptr);
  app->ControlTask();
}

void PicoFirmwareApp::ControlTask() {
  robot::support::LogI(kControlTag,
                       std::string("control task started core=") +
                           std::to_string(get_core_num()));
  TickType_t last_wake_time = xTaskGetTickCount();
  while (true) {
    const uint32_t now_ms = CurrentTicksMs();
    if (controller_uart_runtime_.ShouldPoll(now_ms)) {
      PollControllerUart(now_ms);
    }
    control_core_runtime_.Tick(now_ms);
    MaybeLogControllerUartSample(now_ms);
    vTaskDelayUntil(&last_wake_time, kControlLoopPeriod);
  }
}

void PicoFirmwareApp::InitializeControllerUart() {
  if (!controller_uart_runtime_.enabled()) {
    robot::support::LogW(kBootTag, "controller UART ingress disabled by config");
    controller_uart_active_ = false;
    controller_uart_ = nullptr;
    return;
  }

  if (!controller_uart_runtime_.IsHardwareConfigValid()) {
    robot::support::LogE(
        kBootTag,
        std::string("controller UART ingress invalid config uart_id=") +
            std::to_string(controller_uart_runtime_.uart_id()) + " baud=" +
            std::to_string(controller_uart_runtime_.baud()) + " tx=" +
            std::to_string(controller_uart_runtime_.tx_pin()) + " rx=" +
            std::to_string(controller_uart_runtime_.rx_pin()));
    controller_uart_active_ = false;
    controller_uart_ = nullptr;
    return;
  }

  controller_uart_ = ControllerUartFromId(controller_uart_runtime_.uart_id());
  if (controller_uart_ == nullptr) {
    robot::support::LogE(kBootTag,
                         std::string("controller UART ingress unsupported uart_id=") +
                             std::to_string(controller_uart_runtime_.uart_id()));
    controller_uart_active_ = false;
    return;
  }

  uart_init(controller_uart_, static_cast<uint32_t>(controller_uart_runtime_.baud()));
  gpio_set_function(static_cast<uint>(controller_uart_runtime_.tx_pin()), GPIO_FUNC_UART);
  gpio_set_function(static_cast<uint>(controller_uart_runtime_.rx_pin()), GPIO_FUNC_UART);
  uart_set_hw_flow(controller_uart_, false, false);
  uart_set_format(controller_uart_, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(controller_uart_, true);

  controller_uart_active_ = true;
  robot::support::LogI(
      kBootTag,
      std::string("controller UART ingress started uart=") +
          std::to_string(controller_uart_runtime_.uart_id()) + " tx=" +
          std::to_string(controller_uart_runtime_.tx_pin()) + " rx=" +
          std::to_string(controller_uart_runtime_.rx_pin()) + " baud=" +
          std::to_string(controller_uart_runtime_.baud()) + " read_chunk=" +
          std::to_string(controller_uart_runtime_.read_chunk()) + " poll_ms=" +
          std::to_string(controller_uart_runtime_.poll_ms()) + " diag_sample_ms=" +
          std::to_string(controller_uart_runtime_.diag_sample_ms()));
  robot::support::LogI(
      kControllerTag,
      std::string("controller pipeline active poll_ms=") +
          std::to_string(controller_uart_runtime_.poll_ms()) +
          " publish_period_ms=" +
          std::to_string(robot::config::kControllerDriveConfig.publish_period_ms) +
          " stale_ms=" +
          std::to_string(robot::config::kControllerDriveConfig.snapshot_stale_ms));
}

void PicoFirmwareApp::PollControllerUart(uint32_t now_ms) {
  if (!controller_uart_active_ || controller_uart_ == nullptr) {
    return;
  }

  std::array<uint8_t, kMaxControllerReadChunk> buffer{};
  const std::size_t read_limit = std::min<std::size_t>(
      static_cast<std::size_t>(controller_uart_runtime_.read_chunk()), buffer.size());
  std::size_t read = 0;
  while (uart_is_readable(controller_uart_) && read < read_limit) {
    buffer[read++] = static_cast<uint8_t>(uart_getc(controller_uart_));
  }

  if (read > 0) {
    control_core_runtime_.FeedControllerBytes(buffer.data(), read, now_ms);
  }
}

void PicoFirmwareApp::MaybeLogControllerUartSample(uint32_t now_ms) {
  if (!controller_uart_runtime_.ShouldSample(now_ms)) {
    return;
  }
}

uart_inst_t* PicoFirmwareApp::ControllerUartFromId(int uart_id) {
  switch (uart_id) {
    case 0:
      return uart0;
    case 1:
      return uart1;
    default:
      return nullptr;
  }
}

}  // namespace robot::app
