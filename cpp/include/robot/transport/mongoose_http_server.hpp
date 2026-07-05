#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "FreeRTOS.h"
#include "mongoose.h"
#include "robot/runtime/service_core_runtime.hpp"

namespace robot::transport {

class MongooseHttpServer {
 public:
  MongooseHttpServer() = default;
  ~MongooseHttpServer();

  bool Start(robot::runtime::ServiceCoreRuntime& runtime, const char* listen_url);
  void Poll(int timeout_ms);

 private:
  static void StaticEventHandler(mg_connection* connection, int ev, void* ev_data);
  void EventHandler(mg_connection* connection, int ev, void* ev_data);

  void HandleHttp(mg_connection* connection, mg_http_message* message);
  void HandleWebSocketOpen(mg_connection* connection);
  void HandleWebSocketClose(mg_connection* connection);
  void HandleWebSocket(mg_connection* connection, mg_ws_message* message);
  void HandleBootloaderReboot(mg_connection* connection);
  void ReplyAck(mg_connection* connection, const char* code, std::optional<int> seq);
  void ReplyError(mg_connection* connection,
                  const char* code,
                  std::optional<int> seq,
                  const char* message);
  void WaitAndReply(mg_connection* connection, std::optional<int> seq);
  [[nodiscard]] std::optional<robot::runtime::ControlCompletion> WaitForCompletion();
  void ScheduleBootloaderReboot();
  [[nodiscard]] std::string ConnectionSourceId(const mg_connection* connection) const;

  static std::optional<int> ExtractJsonInt(std::string_view json, std::string_view key);
  static std::optional<char> ExtractJsonChar(std::string_view json, std::string_view key);

  mg_mgr manager_{};
  bool initialized_{false};
  bool bootloader_reboot_pending_{false};
  TickType_t bootloader_reboot_deadline_{0};
  robot::runtime::ServiceCoreRuntime* runtime_{nullptr};
};

}  // namespace robot::transport
