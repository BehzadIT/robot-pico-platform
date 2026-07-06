#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

namespace {

void WaitForUsb() {
  const absolute_time_t deadline = make_timeout_time_ms(5000);
  while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
    sleep_ms(50);
  }
  printf("robot cyw43 reinit probe usb_connected=%d\n", stdio_usb_connected() ? 1 : 0);
  sleep_ms(2000);
}

}  // namespace

int main() {
  stdio_init_all();
  printf("robot cyw43 reinit probe boot\n");
  WaitForUsb();

  printf("robot cyw43 reinit probe first init starting\n");
  const int first_result = cyw43_arch_init();
  printf("robot cyw43 reinit probe first init result=%d\n", first_result);
  if (first_result == 0) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(500);
    printf("robot cyw43 reinit probe deinit starting\n");
    cyw43_arch_deinit();
    printf("robot cyw43 reinit probe deinit completed\n");
  }

  sleep_ms(1000);
  printf("robot cyw43 reinit probe second init starting\n");
  const int second_result = cyw43_arch_init();
  printf("robot cyw43 reinit probe second init result=%d\n", second_result);

  uint32_t count = 0;
  while (true) {
    if (second_result == 0) {
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, (count & 1U) != 0U);
    }
    printf("robot cyw43 reinit probe heartbeat=%lu first=%d second=%d\n",
           static_cast<unsigned long>(count++), first_result, second_result);
    sleep_ms(1000);
  }
}
