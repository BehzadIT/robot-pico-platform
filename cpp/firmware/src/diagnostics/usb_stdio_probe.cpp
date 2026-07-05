#include <stdio.h>

#include "pico/stdlib.h"

int main() {
  stdio_init_all();

  uint32_t count = 0;
  while (true) {
    printf("robot usb stdio probe count=%lu\n", static_cast<unsigned long>(count++));
    sleep_ms(1000);
  }
}
