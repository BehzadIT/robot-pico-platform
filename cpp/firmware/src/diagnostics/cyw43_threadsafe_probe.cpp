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
  printf("robot cyw43 threadsafe probe usb_connected=%d\n", stdio_usb_connected() ? 1 : 0);
  sleep_ms(2000);
}

}  // namespace

int main() {
  stdio_init_all();
  printf("robot cyw43 threadsafe probe boot\n");
  WaitForUsb();

  printf("robot cyw43 threadsafe probe init starting\n");
  const int init_result = cyw43_arch_init();
  printf("robot cyw43 threadsafe probe init result=%d\n", init_result);
  if (init_result == 0) {
    cyw43_arch_enable_sta_mode();
    printf("robot cyw43 threadsafe probe sta enabled\n");
  }

  uint32_t count = 0;
  while (true) {
    printf("robot cyw43 threadsafe probe heartbeat=%lu init_result=%d\n",
           static_cast<unsigned long>(count++), init_result);
    sleep_ms(1000);
  }
}
