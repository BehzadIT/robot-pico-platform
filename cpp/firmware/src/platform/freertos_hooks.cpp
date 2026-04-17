#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

extern "C" void vApplicationStackOverflowHook(TaskHandle_t /*xTask*/, char* pcTaskName) {
  const char* task_name = (pcTaskName == nullptr) ? "unknown" : pcTaskName;
  std::fputs("E (0) rtos: stack overflow detected in task ", stdout);
  std::fputs(task_name, stdout);
  std::fputs("\n", stdout);
  std::fflush(stdout);
  __builtin_trap();
}

extern "C" void vApplicationMallocFailedHook(void) {
  std::fputs("E (0) rtos: heap allocation failed\n", stdout);
  std::fflush(stdout);
  __builtin_trap();
}
