#include "robot/platform/pico_service_core.hpp"

#include <cstdio>
#include <string>

#include "FreeRTOS.h"
#include "hardware/watchdog.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "task.h"
#include "robot/config/config.hpp"
#include "robot/config/secrets.hpp"
#include "robot/support/logger.hpp"

namespace robot::platform {

namespace {

constexpr const char* kWifiTag = "wifi";
constexpr const char* kServiceTag = "service";
constexpr TickType_t kServiceTaskDelay = pdMS_TO_TICKS(5000);
constexpr uint32_t kServiceHeartbeatMs = 5000;
constexpr int kMongoosePollMs = 10;
constexpr const char* kMongooseListenUrl = "http://0.0.0.0:80";
constexpr UBaseType_t kServiceTaskPriority = 3;
constexpr UBaseType_t kServiceCoreAffinityMask = 1U << 0;
constexpr configSTACK_DEPTH_TYPE kServiceTaskStackWords = 2048;
constexpr uint32_t kWifiInitWatchdogTimeoutMs = 8000;
#ifndef ROBOT_PICO_WIFI_INIT_WATCHDOG
#define ROBOT_PICO_WIFI_INIT_WATCHDOG 1
#endif
#ifndef ROBOT_PICO_WIFI_WAIT_FOR_USB_MS
#define ROBOT_PICO_WIFI_WAIT_FOR_USB_MS 0
#endif
#ifndef ROBOT_PICO_WIFI_USB_SETTLE_MS
#define ROBOT_PICO_WIFI_USB_SETTLE_MS 0
#endif
constexpr bool kEnableWifiInitWatchdog = ROBOT_PICO_WIFI_INIT_WATCHDOG != 0;

std::string FormatString(const char* format, int a, int b = 0) {
  char buffer[160];
  std::snprintf(buffer, sizeof(buffer), format, a, b);
  return std::string(buffer);
}

std::string CurrentIpString() {
  if (netif_default == nullptr) {
    return "unknown";
  }
  const ip4_addr_t* ip = netif_ip4_addr(netif_default);
  const char* ip_text = ip4addr_ntoa(ip);
  return ip_text == nullptr ? "unknown" : std::string(ip_text);
}

std::string InterfaceSnapshot() {
  if (netif_default == nullptr) {
    return "netif_default=missing ifconfig=unavailable";
  }
  return std::string("netif_default=present ip=") + CurrentIpString();
}

std::string StageName(uint32_t stage) {
  switch (stage) {
    case 0:
      return "init";
    case 1:
      return "task_started";
    case 2:
      return "wifi_init_enter";
    case 3:
      return "wifi_init_complete";
    case 4:
      return "wifi_sta_enabled";
    case 5:
      return "wifi_connect_enter";
    case 6:
      return "wifi_connect_complete";
    case 7:
      return "service_heartbeat";
    default:
      return "unknown";
  }
}

void RecordWifiStage(uint32_t stage, const char* label) {
  watchdog_hw->scratch[1] = stage;
  robot::support::LogI(kWifiTag,
                       std::string("wifi stage=") + label + " code=" + std::to_string(stage));
}

void WaitForUsbBeforeCyw43() {
#if ROBOT_PICO_WIFI_WAIT_FOR_USB_MS > 0
  robot::support::LogI(kWifiTag,
                       std::string("waiting for USB before CYW43 max_ms=") +
                           std::to_string(ROBOT_PICO_WIFI_WAIT_FOR_USB_MS));
  const absolute_time_t deadline = make_timeout_time_ms(ROBOT_PICO_WIFI_WAIT_FOR_USB_MS);
  while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  robot::support::LogI(kWifiTag,
                       std::string("USB wait complete connected=") +
                           (stdio_usb_connected() ? "true" : "false"));
#endif

#if ROBOT_PICO_WIFI_USB_SETTLE_MS > 0
  robot::support::LogI(kWifiTag,
                       std::string("USB settle before CYW43 ms=") +
                           std::to_string(ROBOT_PICO_WIFI_USB_SETTLE_MS));
  vTaskDelay(pdMS_TO_TICKS(ROBOT_PICO_WIFI_USB_SETTLE_MS));
#endif
}

}  // namespace

PicoServiceCore::PicoServiceCore(robot::runtime::ServiceCoreRuntime& service_runtime)
    : service_runtime_(service_runtime) {}

bool PicoServiceCore::Start() {
  TaskHandle_t service_task_handle = nullptr;
  const BaseType_t result = xTaskCreateAffinitySet(&PicoServiceCore::NetworkTaskEntry,
                                                   "svc-core0", kServiceTaskStackWords, this,
                                                   kServiceTaskPriority, kServiceCoreAffinityMask,
                                                   &service_task_handle);
  robot::support::LogI(kServiceTag, std::string("xTaskCreate result=") + std::to_string(result));
  if (result != pdPASS) {
    robot::support::LogE(kServiceTag, "Start() failed: xTaskCreate returned error");
    return false;
  }
  robot::support::LogI(kServiceTag,
                       std::string("service task created core=0 priority=") +
                           std::to_string(kServiceTaskPriority) + " stack_words=" +
                           std::to_string(kServiceTaskStackWords));
  (void)service_task_handle;
  return true;
}

void PicoServiceCore::NetworkTaskEntry(void* context) {
  static_cast<PicoServiceCore*>(context)->NetworkTask();
}

void PicoServiceCore::NetworkTask() {
  robot::support::LogI(kServiceTag,
                       std::string("service task started core=") +
                           std::to_string(get_core_num()));
  RecordWifiStage(1, "task_started");
  bool server_started = false;
#if defined(ROBOT_PICO_SKIP_WIFI) && ROBOT_PICO_SKIP_WIFI
  robot::support::LogW(kServiceTag,
                       "ROBOT_PICO_SKIP_WIFI enabled; skipping CYW43 and Mongoose startup");
#else
  if (!JoinWifi()) {
    robot::support::LogE(kServiceTag, "service task Wi-Fi stage failed");
  } else {
    robot::support::LogI(kServiceTag, "service task Wi-Fi stage completed");
    server_started = server_.Start(service_runtime_, kMongooseListenUrl);
    if (server_started) {
      robot::support::LogI(kServiceTag,
                           std::string("Mongoose listening url=") + kMongooseListenUrl +
                               " poll_ms=" + std::to_string(kMongoosePollMs));
    } else {
      robot::support::LogE(kServiceTag,
                           std::string("Mongoose listen failed url=") + kMongooseListenUrl);
    }
  }
#endif

  TickType_t last_heartbeat = xTaskGetTickCount();
  while (true) {
    if (server_started) {
      server_.Poll(kMongoosePollMs);
    }

    const TickType_t now = xTaskGetTickCount();
    if ((now - last_heartbeat) >= pdMS_TO_TICKS(kServiceHeartbeatMs)) {
      RecordWifiStage(7, "service_heartbeat");
      robot::support::LogI(kServiceTag, "service task heartbeat");
      last_heartbeat = now;
    }

    if (!server_started) {
      vTaskDelay(kServiceTaskDelay);
    }
  }
}

bool PicoServiceCore::JoinWifi() {
  WaitForUsbBeforeCyw43();
  robot::support::LogI(kWifiTag, "cyw43 init starting");
  RecordWifiStage(2, "wifi_init_enter");
  if (kEnableWifiInitWatchdog) {
    watchdog_enable(kWifiInitWatchdogTimeoutMs, true);
  }
  if (cyw43_arch_init()) {
    if (kEnableWifiInitWatchdog) {
      watchdog_disable();
    }
    robot::support::LogE(kWifiTag, "cyw43 init failed reason=cyw43_init_failed");
    return false;
  }
  if (kEnableWifiInitWatchdog) {
    watchdog_disable();
  }
  robot::support::LogI(kWifiTag, "cyw43 init completed");
  RecordWifiStage(3, "wifi_init_complete");
  cyw43_arch_enable_sta_mode();
  RecordWifiStage(4, "wifi_sta_enabled");
  const auto& wifi_config = robot::config::kWifiConfig;
  const int timeout_ms = wifi_config.connect_timeout_s * 1000;
  robot::support::LogI(
      kWifiTag,
      FormatString("connecting timeout_s=%d attempts=%d", wifi_config.connect_timeout_s,
                   wifi_config.max_attempts));
  robot::support::LogD(kWifiTag, std::string("pre-connect state: ") + InterfaceSnapshot());

  for (int attempt = 1; attempt <= wifi_config.max_attempts; ++attempt) {
    robot::support::LogI(kWifiTag,
                         FormatString("attempt=%d/%d starting", attempt,
                                      wifi_config.max_attempts));
    RecordWifiStage(5, "wifi_connect_enter");

    const bool connected =
        cyw43_arch_wifi_connect_timeout_ms(robot::config::kWifiCredentials.ssid,
                                           robot::config::kWifiCredentials.password,
                                           CYW43_AUTH_WPA2_AES_PSK, timeout_ms) == 0;
    RecordWifiStage(6, "wifi_connect_complete");
    robot::support::LogI(kWifiTag,
                         std::string("attempt result attempt=") + std::to_string(attempt) +
                             " connected=" + (connected ? "True" : "False"));
    if (connected) {
      const std::string ip = CurrentIpString();
      robot::support::LogI(
          kWifiTag, std::string("connected attempt=") + std::to_string(attempt) + " ip=" + ip);
      return true;
    }

    robot::support::LogW(kWifiTag,
                         std::string("attempt failed attempt=") + std::to_string(attempt) +
                             "/" + std::to_string(wifi_config.max_attempts) +
                             " reason=connect_timeout_or_auth_failure");
    robot::support::LogD(kWifiTag, std::string("post-failure snapshot: ") + InterfaceSnapshot());
    if (attempt < wifi_config.max_attempts) {
      robot::support::LogI(kWifiTag,
                           std::string("retrying in ") +
                               std::to_string(wifi_config.retry_backoff_ms) + "ms");
      vTaskDelay(pdMS_TO_TICKS(wifi_config.retry_backoff_ms));
    }
  }

  robot::support::LogE(kWifiTag,
                       std::string("Wi-Fi connection failed after retry budget exhausted attempts=") +
                           std::to_string(wifi_config.max_attempts) +
                           " reason=retry_budget_exhausted");
  return false;
}

}  // namespace robot::platform
