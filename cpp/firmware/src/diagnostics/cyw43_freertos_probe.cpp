#include <stdio.h>

#include "FreeRTOS.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "task.h"

namespace {

constexpr UBaseType_t kWifiProbeCoreAffinityMask = 1U << 0;
#ifndef ROBOT_PICO_CYW43_PROBE_TASK_STACK_WORDS
#define ROBOT_PICO_CYW43_PROBE_TASK_STACK_WORDS 2048
#endif
#ifndef ROBOT_PICO_CYW43_PROBE_TASK_AFFINITY
#define ROBOT_PICO_CYW43_PROBE_TASK_AFFINITY 1
#endif
#ifndef ROBOT_PICO_CYW43_PROBE_WATCHDOG
#define ROBOT_PICO_CYW43_PROBE_WATCHDOG 1
#endif
#ifndef ROBOT_PICO_CYW43_PROBE_USB_SETTLE_MS
#define ROBOT_PICO_CYW43_PROBE_USB_SETTLE_MS 0
#endif
#ifndef ROBOT_PICO_CYW43_PROBE_WAIT_FOR_USB_MS
#define ROBOT_PICO_CYW43_PROBE_WAIT_FOR_USB_MS 0
#endif

constexpr configSTACK_DEPTH_TYPE kWifiProbeStackWords = ROBOT_PICO_CYW43_PROBE_TASK_STACK_WORDS;

void WaitForUsbBeforeCyw43() {
#if ROBOT_PICO_CYW43_PROBE_WAIT_FOR_USB_MS > 0
  printf("robot cyw43 probe waiting for usb max_ms=%d\n", ROBOT_PICO_CYW43_PROBE_WAIT_FOR_USB_MS);
  const absolute_time_t deadline =
      make_timeout_time_ms(ROBOT_PICO_CYW43_PROBE_WAIT_FOR_USB_MS);
  while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  printf("robot cyw43 probe usb_connected=%d\n", stdio_usb_connected() ? 1 : 0);
#endif

#if ROBOT_PICO_CYW43_PROBE_USB_SETTLE_MS > 0
  printf("robot cyw43 probe usb settle ms=%d\n", ROBOT_PICO_CYW43_PROBE_USB_SETTLE_MS);
  vTaskDelay(pdMS_TO_TICKS(ROBOT_PICO_CYW43_PROBE_USB_SETTLE_MS));
#endif
}

void WifiProbeTask(void* /*context*/) {
  printf("robot cyw43 probe task core=%lu\n", static_cast<unsigned long>(get_core_num()));
  printf("robot cyw43 probe config affinity=%d watchdog=%d stack_words=%lu\n",
         ROBOT_PICO_CYW43_PROBE_TASK_AFFINITY, ROBOT_PICO_CYW43_PROBE_WATCHDOG,
         static_cast<unsigned long>(kWifiProbeStackWords));
  WaitForUsbBeforeCyw43();
  printf("robot cyw43 probe init starting\n");

#if ROBOT_PICO_CYW43_PROBE_WATCHDOG
  watchdog_enable(8000, true);
#endif
  const int init_result = cyw43_arch_init();
#if ROBOT_PICO_CYW43_PROBE_WATCHDOG
  watchdog_disable();
#endif

  printf("robot cyw43 probe init result=%d\n", init_result);
  if (init_result == 0) {
    cyw43_arch_enable_sta_mode();
    printf("robot cyw43 probe sta enabled\n");
  }

  uint32_t count = 0;
  while (true) {
    printf("robot cyw43 probe heartbeat=%lu init_result=%d\n",
           static_cast<unsigned long>(count++), init_result);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

}  // namespace

int main() {
  stdio_init_all();
  printf("robot cyw43 freertos probe boot\n");

#if ROBOT_PICO_CYW43_PROBE_TASK_AFFINITY
  const BaseType_t result =
      xTaskCreateAffinitySet(WifiProbeTask, "cyw43-probe", kWifiProbeStackWords, nullptr, 3,
                             kWifiProbeCoreAffinityMask, nullptr);
#else
  const BaseType_t result =
      xTaskCreate(WifiProbeTask, "cyw43-probe", kWifiProbeStackWords, nullptr, 3, nullptr);
#endif
  if (result != pdPASS) {
    printf("robot cyw43 probe task create failed result=%ld\n", static_cast<long>(result));
    while (true) {
      sleep_ms(1000);
    }
  }

  vTaskStartScheduler();

  while (true) {
    printf("robot cyw43 probe scheduler returned\n");
    sleep_ms(1000);
  }
}
