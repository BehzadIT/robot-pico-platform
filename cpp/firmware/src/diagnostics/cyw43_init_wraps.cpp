#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "pico/async_context_freertos.h"
#include "pico/cyw43_driver.h"
#include "pico/lwip_freertos.h"
#include "lwip/err.h"
#include "lwip/sys.h"

extern "C" bool __real_async_context_freertos_init(async_context_freertos_t* self,
                                                   async_context_freertos_config_t* config);
extern "C" bool __real_cyw43_driver_init(async_context_t* context);
extern "C" bool __real_lwip_freertos_init(async_context_t* context);
extern "C" err_t __real_sys_mbox_new(sys_mbox_t* mbox, int size);
extern "C" err_t __real_sys_mutex_new(sys_mutex_t* mutex);
extern "C" sys_thread_t __real_sys_thread_new(const char* name, lwip_thread_fn thread, void* arg,
                                              int stacksize, int prio);
extern "C" BaseType_t __real_xTaskCreate(TaskFunction_t pxTaskCode, const char* const pcName,
                                         const configSTACK_DEPTH_TYPE uxStackDepth,
                                         void* const pvParameters, UBaseType_t uxPriority,
                                         TaskHandle_t* const pxCreatedTask);
extern "C" BaseType_t __real_xTaskCreateAffinitySet(
    TaskFunction_t pxTaskCode, const char* const pcName,
    const configSTACK_DEPTH_TYPE uxStackDepth, void* const pvParameters, UBaseType_t uxPriority,
    UBaseType_t uxCoreAffinityMask, TaskHandle_t* const pxCreatedTask);
extern "C" void __real_pico_lwip_custom_lock_tcpip_core(void);
extern "C" void __real_pico_lwip_custom_unlock_tcpip_core(void);
extern "C" BaseType_t __real_xQueueGenericSend(QueueHandle_t xQueue, const void* pvItemToQueue,
                                               TickType_t xTicksToWait,
                                               const BaseType_t xCopyPosition);
extern "C" BaseType_t __real_xQueueSemaphoreTake(QueueHandle_t xQueue, TickType_t xTicksToWait);
extern "C" uint32_t __real_sys_arch_mbox_fetch(sys_mbox_t* mbox, void** msg,
                                               uint32_t timeout_ms);
extern "C" uint32_t __real_sys_timeouts_sleeptime(void);

extern "C" bool __wrap_async_context_freertos_init(async_context_freertos_t* self,
                                                   async_context_freertos_config_t* config) {
  printf("robot cyw43 wrap async_context_freertos_init enter priority=%lu stack=%lu\n",
         static_cast<unsigned long>(config == nullptr ? 0 : config->task_priority),
         static_cast<unsigned long>(config == nullptr ? 0 : config->task_stack_size));
  const bool result = __real_async_context_freertos_init(self, config);
  printf("robot cyw43 wrap async_context_freertos_init exit result=%d\n", result ? 1 : 0);
  return result;
}

extern "C" bool __wrap_cyw43_driver_init(async_context_t* context) {
  printf("robot cyw43 wrap cyw43_driver_init enter context=%p\n", static_cast<void*>(context));
  const bool result = __real_cyw43_driver_init(context);
  printf("robot cyw43 wrap cyw43_driver_init exit result=%d\n", result ? 1 : 0);
  return result;
}

extern "C" bool __wrap_lwip_freertos_init(async_context_t* context) {
  printf("robot cyw43 wrap lwip_freertos_init enter context=%p\n", static_cast<void*>(context));
  const bool result = __real_lwip_freertos_init(context);
  printf("robot cyw43 wrap lwip_freertos_init exit result=%d\n", result ? 1 : 0);
  return result;
}

extern "C" err_t __wrap_sys_mbox_new(sys_mbox_t* mbox, int size) {
  printf("robot cyw43 wrap sys_mbox_new enter size=%d\n", size);
  const err_t result = __real_sys_mbox_new(mbox, size);
  printf("robot cyw43 wrap sys_mbox_new exit result=%d\n", static_cast<int>(result));
  return result;
}

extern "C" err_t __wrap_sys_mutex_new(sys_mutex_t* mutex) {
  printf("robot cyw43 wrap sys_mutex_new enter\n");
  const err_t result = __real_sys_mutex_new(mutex);
  printf("robot cyw43 wrap sys_mutex_new exit result=%d\n", static_cast<int>(result));
  return result;
}

extern "C" sys_thread_t __wrap_sys_thread_new(const char* name, lwip_thread_fn thread, void* arg,
                                              int stacksize, int prio) {
  printf("robot cyw43 wrap sys_thread_new enter name_ptr=%p thread=%p stacksize=%d prio=%d\n",
         static_cast<const void*>(name), reinterpret_cast<void*>(thread), stacksize, prio);
  TaskHandle_t rtos_task = nullptr;
  const BaseType_t create_result = xTaskCreateAffinitySet(
      reinterpret_cast<TaskFunction_t>(thread), name, static_cast<configSTACK_DEPTH_TYPE>(stacksize),
      arg, static_cast<UBaseType_t>(prio), 1U << 0, &rtos_task);
  printf("robot cyw43 wrap sys_thread_new affinity create result=%ld handle=%p\n",
         static_cast<long>(create_result), static_cast<void*>(rtos_task));
  configASSERT(create_result == pdPASS);
  sys_thread_t result{};
  result.thread_handle = rtos_task;
  printf("robot cyw43 wrap sys_thread_new exit affinity=core0\n");
  return result;
}

extern "C" BaseType_t __wrap_xTaskCreate(TaskFunction_t pxTaskCode, const char* const pcName,
                                         const configSTACK_DEPTH_TYPE uxStackDepth,
                                         void* const pvParameters, UBaseType_t uxPriority,
                                         TaskHandle_t* const pxCreatedTask) {
  printf("robot cyw43 wrap xTaskCreate enter name_ptr=%p task=%p stack_words=%lu prio=%lu\n",
         static_cast<const void*>(pcName), reinterpret_cast<void*>(pxTaskCode),
         static_cast<unsigned long>(uxStackDepth), static_cast<unsigned long>(uxPriority));
  const BaseType_t result =
      __real_xTaskCreate(pxTaskCode, pcName, uxStackDepth, pvParameters, uxPriority,
                         pxCreatedTask);
  printf("robot cyw43 wrap xTaskCreate exit result=%ld handle=%p\n", static_cast<long>(result),
         pxCreatedTask == nullptr ? nullptr : static_cast<void*>(*pxCreatedTask));
  return result;
}

extern "C" BaseType_t __wrap_xTaskCreateAffinitySet(
    TaskFunction_t pxTaskCode, const char* const pcName,
    const configSTACK_DEPTH_TYPE uxStackDepth, void* const pvParameters, UBaseType_t uxPriority,
    UBaseType_t uxCoreAffinityMask, TaskHandle_t* const pxCreatedTask) {
  printf("robot cyw43 wrap xTaskCreateAffinitySet enter name_ptr=%p task=%p stack_words=%lu prio=%lu affinity=0x%lx\n",
         static_cast<const void*>(pcName), reinterpret_cast<void*>(pxTaskCode),
         static_cast<unsigned long>(uxStackDepth), static_cast<unsigned long>(uxPriority),
         static_cast<unsigned long>(uxCoreAffinityMask));
  const BaseType_t result =
      __real_xTaskCreateAffinitySet(pxTaskCode, pcName, uxStackDepth, pvParameters, uxPriority,
                                    uxCoreAffinityMask, pxCreatedTask);
  printf("robot cyw43 wrap xTaskCreateAffinitySet exit result=%ld handle=%p\n",
         static_cast<long>(result),
         pxCreatedTask == nullptr ? nullptr : static_cast<void*>(*pxCreatedTask));
  return result;
}

extern "C" void __wrap_pico_lwip_custom_lock_tcpip_core(void) {
  printf("robot cyw43 wrap pico_lwip_custom_lock_tcpip_core enter\n");
  __real_pico_lwip_custom_lock_tcpip_core();
  printf("robot cyw43 wrap pico_lwip_custom_lock_tcpip_core exit\n");
}

extern "C" void __wrap_pico_lwip_custom_unlock_tcpip_core(void) {
  printf("robot cyw43 wrap pico_lwip_custom_unlock_tcpip_core enter\n");
  __real_pico_lwip_custom_unlock_tcpip_core();
  printf("robot cyw43 wrap pico_lwip_custom_unlock_tcpip_core exit\n");
}

extern "C" BaseType_t __wrap_xQueueGenericSend(QueueHandle_t xQueue, const void* pvItemToQueue,
                                               TickType_t xTicksToWait,
                                               const BaseType_t xCopyPosition) {
  printf("robot cyw43 wrap xQueueGenericSend enter queue=%p item=%p ticks=%lu pos=%ld\n",
         static_cast<void*>(xQueue), pvItemToQueue,
         static_cast<unsigned long>(xTicksToWait), static_cast<long>(xCopyPosition));
  const BaseType_t result =
      __real_xQueueGenericSend(xQueue, pvItemToQueue, xTicksToWait, xCopyPosition);
  printf("robot cyw43 wrap xQueueGenericSend exit result=%ld queue=%p\n",
         static_cast<long>(result), static_cast<void*>(xQueue));
  return result;
}

extern "C" BaseType_t __wrap_xQueueSemaphoreTake(QueueHandle_t xQueue, TickType_t xTicksToWait) {
  printf("robot cyw43 wrap xQueueSemaphoreTake enter queue=%p ticks=%lu\n",
         static_cast<void*>(xQueue), static_cast<unsigned long>(xTicksToWait));
  const BaseType_t result = __real_xQueueSemaphoreTake(xQueue, xTicksToWait);
  printf("robot cyw43 wrap xQueueSemaphoreTake exit result=%ld queue=%p\n",
         static_cast<long>(result), static_cast<void*>(xQueue));
  return result;
}

extern "C" uint32_t __wrap_sys_timeouts_sleeptime(void) {
  printf("robot cyw43 wrap sys_timeouts_sleeptime enter\n");
  const uint32_t result = __real_sys_timeouts_sleeptime();
  printf("robot cyw43 wrap sys_timeouts_sleeptime exit result=%lu\n",
         static_cast<unsigned long>(result));
  return result;
}

extern "C" uint32_t __wrap_sys_arch_mbox_fetch(sys_mbox_t* mbox, void** msg,
                                               uint32_t timeout_ms) {
  printf("robot cyw43 wrap sys_arch_mbox_fetch enter mbox=%p timeout_ms=%lu\n",
         static_cast<void*>(mbox), static_cast<unsigned long>(timeout_ms));
  const uint32_t result = __real_sys_arch_mbox_fetch(mbox, msg, timeout_ms);
  printf("robot cyw43 wrap sys_arch_mbox_fetch exit result=%lu msg=%p\n",
         static_cast<unsigned long>(result),
         (msg == nullptr || *msg == nullptr) ? nullptr : *msg);
  return result;
}
