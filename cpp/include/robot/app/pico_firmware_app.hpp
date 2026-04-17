#pragma once

#include "robot/config/config.hpp"
#include "robot/platform/controller_uart_runtime.hpp"
#include "robot/platform/pico_service_core.hpp"
#include "robot/runtime/control_core_runtime.hpp"
#include "robot/runtime/service_core_runtime.hpp"
#include "robot/runtime/shared_runtime_state.hpp"

struct uart_inst;

namespace robot::app {

// Bootstraps the Pico 2 W runtime and creates the core-pinned control task.
class PicoFirmwareApp {
 public:
  PicoFirmwareApp();
  void Run();

 private:
  void InitializeControllerUart();
  void PollControllerUart(uint32_t now_ms);
  void MaybeLogControllerUartSample(uint32_t now_ms);
  void ControlTask();
  static void ControlTaskEntry(void* context);
  static struct uart_inst* ControllerUartFromId(int uart_id);
  robot::runtime::SharedRuntimeState shared_state_{};
  robot::runtime::ServiceCoreRuntime service_core_runtime_{shared_state_};
  robot::runtime::ControlCoreRuntime control_core_runtime_;
  robot::platform::PicoServiceCore service_core_;
  robot::platform::ControllerUartRuntime controller_uart_runtime_{robot::config::kControllerUartConfig};
  struct uart_inst* controller_uart_{nullptr};
  bool controller_uart_active_{false};
};

}  // namespace robot::app
