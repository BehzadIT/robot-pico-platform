#include <cstdio>

#include "pico/stdlib.h"

namespace {
constexpr uint32_t kUsbSettleMs = 1500;
constexpr uint32_t kHeartbeatMs = 1000;

void LogBoot() {
  std::printf("%s: boot\n", ROBOT_PICO_APP_NAME);
  std::printf("%s: stage=%s\n", ROBOT_PICO_APP_NAME, ROBOT_PICO_APP_STAGE);
  std::printf("%s: board=pico2_w stdio=usb picotool_reset=enabled\n",
              ROBOT_PICO_APP_NAME);
  std::printf("%s: no cyw43, no transport, no motor output\n",
              ROBOT_PICO_APP_NAME);
  std::printf("%s: ready for flash/reboot feedback-loop test\n",
              ROBOT_PICO_APP_NAME);
}
}  // namespace

int main() {
  stdio_init_all();
  sleep_ms(kUsbSettleMs);

  LogBoot();

  uint32_t heartbeat = 0;
  while (true) {
    std::printf("%s: heartbeat=%lu uptime_ms=%llu\n", ROBOT_PICO_APP_NAME,
                static_cast<unsigned long>(heartbeat++),
                static_cast<unsigned long long>(to_ms_since_boot(get_absolute_time())));
    sleep_ms(kHeartbeatMs);
  }
}
