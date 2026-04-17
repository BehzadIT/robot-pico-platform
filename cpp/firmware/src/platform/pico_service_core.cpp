#include "robot/platform/pico_service_core.hpp"

#include <cstdio>
#include <string>

#include "FreeRTOS.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "task.h"
#include "robot/config/config.hpp"
#include "robot/config/secrets.hpp"
#include "robot/support/logger.hpp"

namespace robot::platform {

namespace {

constexpr const char* kWifiTag = "wifi";
constexpr const char* kServiceTag = "service";
constexpr TickType_t kServiceTaskDelay = pdMS_TO_TICKS(5000);
constexpr UBaseType_t kServiceTaskPriority = 3;
constexpr UBaseType_t kServiceCoreAffinityMask = 1U << 0;
constexpr configSTACK_DEPTH_TYPE kServiceTaskStackWords = 2048;
constexpr bool kEnableWifiConnectAttempt = false;

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
  robot::support::LogI(kServiceTag,
                       "service task staged: Wi-Fi and server bring-up are disabled");
  while (true) {
    robot::support::LogI(kServiceTag, "service task heartbeat");
    vTaskDelay(kServiceTaskDelay);
  }
}

bool PicoServiceCore::JoinWifi() {
  robot::support::LogI(kWifiTag, "cyw43 init skipped for boundary probe");
  cyw43_arch_enable_sta_mode();
  if (!kEnableWifiConnectAttempt) {
    robot::support::LogI(kWifiTag, "wifi connect stage disabled");
    return true;
  }

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

    const bool connected =
        cyw43_arch_wifi_connect_timeout_ms(robot::config::kWifiCredentials.ssid,
                                           robot::config::kWifiCredentials.password,
                                           CYW43_AUTH_WPA2_AES_PSK, timeout_ms) == 0;
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
