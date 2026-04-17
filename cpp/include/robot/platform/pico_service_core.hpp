#pragma once

#include "robot/runtime/service_core_runtime.hpp"
#include "robot/transport/mongoose_http_server.hpp"

namespace robot::platform {

// Owns the core-0 RTOS/service task and staged Wi-Fi/server bring-up.
class PicoServiceCore {
 public:
  explicit PicoServiceCore(robot::runtime::ServiceCoreRuntime& service_runtime);

  [[nodiscard]] bool Start();

 private:
  static void NetworkTaskEntry(void* context);
  void NetworkTask();
  bool JoinWifi();

  robot::runtime::ServiceCoreRuntime& service_runtime_;
  robot::transport::MongooseHttpServer server_;
};

}  // namespace robot::platform
