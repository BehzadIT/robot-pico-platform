#include <stdio.h>

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "task.h"

namespace {

void ProbeTask(void* /*context*/) {
  uint32_t count = 0;
  while (true) {
    printf("robot freertos usb probe count=%lu\n", static_cast<unsigned long>(count++));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

}  // namespace

int main() {
  stdio_init_all();
  printf("robot freertos usb probe boot\n");

  const BaseType_t result = xTaskCreate(ProbeTask, "usb-probe", 1024, nullptr, 1, nullptr);
  if (result != pdPASS) {
    printf("robot freertos usb probe task create failed\n");
    while (true) {
      sleep_ms(1000);
    }
  }

  vTaskStartScheduler();

  while (true) {
    printf("robot freertos usb probe scheduler returned\n");
    sleep_ms(1000);
  }
}
