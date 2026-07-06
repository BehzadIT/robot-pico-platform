#include <stdio.h>

#include "pico/async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

namespace {

void WaitForUsb() {
  const absolute_time_t deadline = make_timeout_time_ms(5000);
  while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
    sleep_ms(50);
  }
  printf("robot cyw43 poll probe usb_connected=%d\n", stdio_usb_connected() ? 1 : 0);
  sleep_ms(2000);
}

}  // namespace

int main() {
  stdio_init_all();
  printf("robot cyw43 poll probe boot\n");
  WaitForUsb();

  printf("robot cyw43 poll probe init starting\n");
  const int init_result = cyw43_arch_init();
  printf("robot cyw43 poll probe init result=%d\n", init_result);
  if (init_result == 0) {
    cyw43_arch_enable_sta_mode();
    printf("robot cyw43 poll probe sta enabled\n");
  }

  uint32_t count = 0;
  while (true) {
    async_context_poll(cyw43_arch_async_context());
    printf("robot cyw43 poll probe heartbeat=%lu init_result=%d\n",
           static_cast<unsigned long>(count++), init_result);
    async_context_wait_for_work_ms(cyw43_arch_async_context(), 1000);
  }
}
