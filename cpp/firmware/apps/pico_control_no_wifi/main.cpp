#include <cstdio>

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"

namespace {
// Give the USB CDC device time to enumerate before the first boot banner.
// This is only for observability in the clean app; it is not a control-loop
// timing dependency. Boot-log capture should still start on the host before
// flash/reboot when the exact first lines matter.
constexpr uint32_t kUsbSettleMs = 1500;

// Slow service-side proof of life. Core 0 is currently only a service skeleton,
// so a low-rate heartbeat is enough to prove task scheduling without noisy logs.
constexpr TickType_t kServiceHeartbeatTicks = pdMS_TO_TICKS(5000);

// Placeholder cadence for the future deterministic control tick on core 1.
// Step 3 intentionally does no command admission, encoder reads, or motor
// writes; this period only proves that vTaskDelayUntil-based scheduling works.
constexpr TickType_t kControlLoopPeriodTicks = pdMS_TO_TICKS(10);

// Log every 100 control ticks. With the 10 ms skeleton tick this yields a
// 1 Hz control heartbeat, keeping USB logs readable while proving cadence.
constexpr uint32_t kControlHeartbeatEveryTicks = 100;

constexpr UBaseType_t kServiceTaskPriority = 2;
constexpr UBaseType_t kControlTaskPriority = 3;
constexpr UBaseType_t kServiceCoreAffinityMask = 1U << 0;
constexpr UBaseType_t kControlCoreAffinityMask = 1U << 1;
constexpr configSTACK_DEPTH_TYPE kTaskStackWords = 1024;

SemaphoreHandle_t g_log_mutex = nullptr;

uint64_t UptimeMs() {
  return to_ms_since_boot(get_absolute_time());
}

void LogLine(const char* role, const char* event, uint32_t count = 0) {
  const bool can_lock =
      g_log_mutex != nullptr && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
  if (can_lock) {
    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  }

  std::printf("%s: %s %s core=%lu count=%lu uptime_ms=%llu\n",
              ROBOT_PICO_APP_NAME, role, event,
              static_cast<unsigned long>(get_core_num()),
              static_cast<unsigned long>(count),
              static_cast<unsigned long long>(UptimeMs()));
  std::fflush(stdout);

  if (can_lock) {
    xSemaphoreGive(g_log_mutex);
  }
}

void LogBoot() {
  std::printf("%s: boot\n", ROBOT_PICO_APP_NAME);
  std::printf("%s: stage=%s\n", ROBOT_PICO_APP_NAME, ROBOT_PICO_APP_STAGE);
  std::printf("%s: board=pico2_w stdio=usb picotool_reset=enabled\n",
              ROBOT_PICO_APP_NAME);
  std::printf("%s: no cyw43, no network, no transport, no hardware output\n",
              ROBOT_PICO_APP_NAME);
  std::printf("%s: FreeRTOS SMP skeleton with pinned service/control tasks\n",
              ROBOT_PICO_APP_NAME);
  std::fflush(stdout);
}

void ServiceTask(void* /*context*/) {
  LogLine("service", "started");

  uint32_t heartbeat = 0;
  while (true) {
    // Core 0 is transport-shaped here, but Step 3 does not parse UART or own
    // control.
    vTaskDelay(kServiceHeartbeatTicks);
    LogLine("service", "heartbeat", heartbeat++);
  }
}

void ControlTask(void* /*context*/) {
  LogLine("control", "started");

  uint32_t tick = 0;
  TickType_t last_wake_time = xTaskGetTickCount();
  while (true) {
    if ((tick % kControlHeartbeatEveryTicks) == 0) {
      LogLine("control", "heartbeat", tick / kControlHeartbeatEveryTicks);
    }
    ++tick;

    // Fixed-cadence placeholder for the future control tick; no motors or
    // encoders are touched.
    vTaskDelayUntil(&last_wake_time, kControlLoopPeriodTicks);
  }
}

bool StartTask(TaskFunction_t entry, const char* name, UBaseType_t priority,
               UBaseType_t affinity_mask) {
  const BaseType_t result = xTaskCreateAffinitySet(
      entry, name, kTaskStackWords, nullptr, priority, affinity_mask, nullptr);
  return result == pdPASS;
}
}  // namespace

int main() {
  stdio_init_all();
  sleep_ms(kUsbSettleMs);

  LogBoot();
  g_log_mutex = xSemaphoreCreateMutex();
  if (g_log_mutex == nullptr) {
    std::printf("%s: log mutex create failed\n", ROBOT_PICO_APP_NAME);
    while (true) {
      // Keep a visible failure state without busy-spinning if allocation fails.
      sleep_ms(1000);
    }
  }

  // Step 3 proves the service/control boundary only: no CYW43, sockets, UART,
  // motors, encoders, or watchdog.
  const bool service_started =
      StartTask(ServiceTask, "svc-core0", kServiceTaskPriority, kServiceCoreAffinityMask);
  const bool control_started =
      StartTask(ControlTask, "ctrl-core1", kControlTaskPriority, kControlCoreAffinityMask);
  if (!service_started || !control_started) {
    std::printf("%s: task creation failed service=%d control=%d\n",
                ROBOT_PICO_APP_NAME, service_started ? 1 : 0, control_started ? 1 : 0);
    while (true) {
      // Keep USB alive for the failure log while avoiding a hot loop.
      sleep_ms(1000);
    }
  }

  std::printf("%s: starting scheduler\n", ROBOT_PICO_APP_NAME);
  std::fflush(stdout);
  // Let the final pre-scheduler line drain before both RTOS tasks start
  // writing to USB; this keeps bench evidence readable.
  sleep_ms(250);
  vTaskStartScheduler();

  std::printf("%s: scheduler returned unexpectedly\n", ROBOT_PICO_APP_NAME);
  std::fflush(stdout);
  while (true) {
    // If the scheduler ever returns, preserve the unexpected-state log without
    // burning a core.
    sleep_ms(1000);
  }
}
