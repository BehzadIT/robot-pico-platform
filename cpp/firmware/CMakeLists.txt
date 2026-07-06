set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/../third_party/freertos-kernel)
set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include)
set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
# RP2350/Pico SDK FreeRTOS uses the SMP-capable RP2040 port family.
set(FREERTOS_PORT "GCC_RP2040" CACHE STRING "" FORCE)

pico_sdk_init()

add_library(freertos_config INTERFACE)
target_include_directories(
  freertos_config
  SYSTEM
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

add_subdirectory(
  ${FREERTOS_KERNEL_PATH}
  freertos_kernel
)

add_library(
  mongoose_embedded
  STATIC
  ../third_party/mongoose/mongoose.c
)

target_include_directories(
  mongoose_embedded
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
)

target_compile_definitions(
  mongoose_embedded
  PUBLIC
  MG_ENABLE_LOG=1
)

target_link_libraries(
  mongoose_embedded
  PUBLIC
  freertos_kernel
  pico_stdlib
  pico_rand
  pico_cyw43_arch_lwip_sys_freertos
)

add_executable(
  robot_pico_firmware
  main.cpp
  src/app/pico_firmware_app.cpp
  src/platform/pico_service_core.cpp
  src/platform/freertos_hooks.cpp
  src/transport/mongoose_http_server.cpp
  src/hardware/pico_motor_output_driver.cpp
  src/hardware/pico_quadrature_counter.cpp
  src/platform/pico_watchdog.cpp
)

pico_generate_pio_header(
  robot_pico_firmware
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_counter.pio
)

target_include_directories(
  robot_pico_firmware
  PRIVATE
  ../include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(
  robot_pico_firmware
  PRIVATE
  ROBOT_PICO_WIFI_INIT_WATCHDOG=0
  ROBOT_PICO_WIFI_WAIT_FOR_USB_MS=0
  ROBOT_PICO_WIFI_USB_SETTLE_MS=2000
)

target_link_libraries(
    robot_pico_firmware
    PRIVATE
    robot_core
    mongoose_embedded
    freertos_kernel
    pico_stdlib
    pico_multicore
    pico_cyw43_arch_lwip_sys_freertos
    hardware_clocks
    hardware_gpio
    hardware_pio
    hardware_pwm
    hardware_uart
    hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_firmware 0)
pico_enable_stdio_usb(robot_pico_firmware 1)
pico_add_extra_outputs(robot_pico_firmware)

add_executable(
  robot_pico_usb_stdio_probe
  src/diagnostics/usb_stdio_probe.cpp
)

target_link_libraries(
  robot_pico_usb_stdio_probe
  PRIVATE
  pico_stdlib
)

pico_enable_stdio_uart(robot_pico_usb_stdio_probe 0)
pico_enable_stdio_usb(robot_pico_usb_stdio_probe 1)
pico_add_extra_outputs(robot_pico_usb_stdio_probe)

add_executable(
  robot_pico_freertos_usb_probe
  src/diagnostics/freertos_usb_probe.cpp
  src/platform/freertos_hooks.cpp
)

target_include_directories(
  robot_pico_freertos_usb_probe
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(
  robot_pico_freertos_usb_probe
  PRIVATE
  freertos_kernel
  pico_stdlib
)

pico_enable_stdio_uart(robot_pico_freertos_usb_probe 0)
pico_enable_stdio_usb(robot_pico_freertos_usb_probe 1)
pico_add_extra_outputs(robot_pico_freertos_usb_probe)

add_executable(
  robot_pico_ab_cyw43_none_probe
  src/diagnostics/cyw43_none_probe.cpp
)

target_link_libraries(
  robot_pico_ab_cyw43_none_probe
  PRIVATE
  pico_stdlib
  pico_cyw43_arch_none
)

pico_enable_stdio_uart(robot_pico_ab_cyw43_none_probe 0)
pico_enable_stdio_usb(robot_pico_ab_cyw43_none_probe 1)
pico_add_extra_outputs(robot_pico_ab_cyw43_none_probe)

add_executable(
  robot_pico_ab_cyw43_poll_probe
  src/diagnostics/cyw43_poll_probe.cpp
)

target_include_directories(
  robot_pico_ab_cyw43_poll_probe
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src/diagnostics/lwip_nosys
)

target_link_libraries(
  robot_pico_ab_cyw43_poll_probe
  PRIVATE
  pico_stdlib
  pico_cyw43_arch_lwip_poll
)

pico_enable_stdio_uart(robot_pico_ab_cyw43_poll_probe 0)
pico_enable_stdio_usb(robot_pico_ab_cyw43_poll_probe 1)
pico_add_extra_outputs(robot_pico_ab_cyw43_poll_probe)

add_executable(
  robot_pico_ab_cyw43_threadsafe_probe
  src/diagnostics/cyw43_threadsafe_probe.cpp
)

target_include_directories(
  robot_pico_ab_cyw43_threadsafe_probe
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src/diagnostics/lwip_nosys
)

target_link_libraries(
  robot_pico_ab_cyw43_threadsafe_probe
  PRIVATE
  pico_stdlib
  pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_uart(robot_pico_ab_cyw43_threadsafe_probe 0)
pico_enable_stdio_usb(robot_pico_ab_cyw43_threadsafe_probe 1)
pico_add_extra_outputs(robot_pico_ab_cyw43_threadsafe_probe)

add_executable(
  robot_pico_ab_cyw43_reinit_probe
  src/diagnostics/cyw43_reinit_probe.cpp
)

target_link_libraries(
  robot_pico_ab_cyw43_reinit_probe
  PRIVATE
  pico_stdlib
  pico_cyw43_arch_none
)

pico_enable_stdio_uart(robot_pico_ab_cyw43_reinit_probe 0)
pico_enable_stdio_usb(robot_pico_ab_cyw43_reinit_probe 1)
pico_add_extra_outputs(robot_pico_ab_cyw43_reinit_probe)

add_executable(
  robot_pico_ab_cyw43_double_init_probe
  src/diagnostics/cyw43_double_init_probe.cpp
)

target_link_libraries(
  robot_pico_ab_cyw43_double_init_probe
  PRIVATE
  pico_stdlib
  pico_cyw43_arch_none
)

pico_enable_stdio_uart(robot_pico_ab_cyw43_double_init_probe 0)
pico_enable_stdio_usb(robot_pico_ab_cyw43_double_init_probe 1)
pico_add_extra_outputs(robot_pico_ab_cyw43_double_init_probe)

add_executable(
  robot_pico_ab_no_wifi
  main.cpp
  src/app/pico_firmware_app.cpp
  src/platform/pico_service_core.cpp
  src/platform/freertos_hooks.cpp
  src/transport/mongoose_http_server.cpp
  src/hardware/pico_motor_output_driver.cpp
  src/hardware/pico_quadrature_counter.cpp
  src/platform/pico_watchdog.cpp
)

pico_generate_pio_header(
  robot_pico_ab_no_wifi
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_counter.pio
)

target_include_directories(
  robot_pico_ab_no_wifi
  PRIVATE
  ../include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(
  robot_pico_ab_no_wifi
  PRIVATE
  ROBOT_PICO_SKIP_WIFI=1
)

target_link_libraries(
  robot_pico_ab_no_wifi
  PRIVATE
  robot_core
  mongoose_embedded
  freertos_kernel
  pico_stdlib
  pico_multicore
  pico_cyw43_arch_lwip_sys_freertos
  hardware_clocks
  hardware_gpio
  hardware_pio
  hardware_pwm
  hardware_uart
  hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_ab_no_wifi 0)
pico_enable_stdio_usb(robot_pico_ab_no_wifi 1)
pico_add_extra_outputs(robot_pico_ab_no_wifi)

add_executable(
  robot_pico_ab_wifi_usb_settle
  main.cpp
  src/app/pico_firmware_app.cpp
  src/platform/pico_service_core.cpp
  src/platform/freertos_hooks.cpp
  src/transport/mongoose_http_server.cpp
  src/hardware/pico_motor_output_driver.cpp
  src/hardware/pico_quadrature_counter.cpp
  src/platform/pico_watchdog.cpp
)

pico_generate_pio_header(
  robot_pico_ab_wifi_usb_settle
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_counter.pio
)

target_include_directories(
  robot_pico_ab_wifi_usb_settle
  PRIVATE
  ../include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(
  robot_pico_ab_wifi_usb_settle
  PRIVATE
  ROBOT_PICO_WIFI_INIT_WATCHDOG=0
  ROBOT_PICO_WIFI_WAIT_FOR_USB_MS=5000
  ROBOT_PICO_WIFI_USB_SETTLE_MS=2000
)

target_link_libraries(
  robot_pico_ab_wifi_usb_settle
  PRIVATE
  robot_core
  mongoose_embedded
  freertos_kernel
  pico_stdlib
  pico_multicore
  pico_cyw43_arch_lwip_sys_freertos
  hardware_clocks
  hardware_gpio
  hardware_pio
  hardware_pwm
  hardware_uart
  hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_ab_wifi_usb_settle 0)
pico_enable_stdio_usb(robot_pico_ab_wifi_usb_settle 1)
pico_add_extra_outputs(robot_pico_ab_wifi_usb_settle)

add_executable(
  robot_pico_ab_wifi_no_control_usb_settle
  main.cpp
  src/app/pico_firmware_app.cpp
  src/platform/pico_service_core.cpp
  src/platform/freertos_hooks.cpp
  src/transport/mongoose_http_server.cpp
  src/hardware/pico_motor_output_driver.cpp
  src/hardware/pico_quadrature_counter.cpp
  src/platform/pico_watchdog.cpp
)

pico_generate_pio_header(
  robot_pico_ab_wifi_no_control_usb_settle
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_counter.pio
)

target_include_directories(
  robot_pico_ab_wifi_no_control_usb_settle
  PRIVATE
  ../include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(
  robot_pico_ab_wifi_no_control_usb_settle
  PRIVATE
  ROBOT_PICO_SKIP_CONTROL_TASK=1
  ROBOT_PICO_WIFI_INIT_WATCHDOG=0
  ROBOT_PICO_WIFI_WAIT_FOR_USB_MS=5000
  ROBOT_PICO_WIFI_USB_SETTLE_MS=2000
)

target_link_libraries(
  robot_pico_ab_wifi_no_control_usb_settle
  PRIVATE
  robot_core
  mongoose_embedded
  freertos_kernel
  pico_stdlib
  pico_multicore
  pico_cyw43_arch_lwip_sys_freertos
  hardware_clocks
  hardware_gpio
  hardware_pio
  hardware_pwm
  hardware_uart
  hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_ab_wifi_no_control_usb_settle 0)
pico_enable_stdio_usb(robot_pico_ab_wifi_no_control_usb_settle 1)
pico_add_extra_outputs(robot_pico_ab_wifi_no_control_usb_settle)

add_executable(
  robot_pico_ab_wifi_stub_control_usb_settle
  main.cpp
  src/app/pico_firmware_app.cpp
  src/platform/pico_service_core.cpp
  src/platform/freertos_hooks.cpp
  src/transport/mongoose_http_server.cpp
  src/hardware/pico_motor_output_driver.cpp
  src/hardware/pico_quadrature_counter.cpp
  src/platform/pico_watchdog.cpp
)

pico_generate_pio_header(
  robot_pico_ab_wifi_stub_control_usb_settle
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_counter.pio
)

target_include_directories(
  robot_pico_ab_wifi_stub_control_usb_settle
  PRIVATE
  ../include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(
  robot_pico_ab_wifi_stub_control_usb_settle
  PRIVATE
  ROBOT_PICO_SKIP_CONTROL_TASK=1
  ROBOT_PICO_SKIP_CONTROLLER_UART=1
  ROBOT_PICO_STUB_CONTROL_HARDWARE=1
  ROBOT_PICO_WIFI_INIT_WATCHDOG=0
  ROBOT_PICO_WIFI_WAIT_FOR_USB_MS=5000
  ROBOT_PICO_WIFI_USB_SETTLE_MS=2000
)

target_link_libraries(
  robot_pico_ab_wifi_stub_control_usb_settle
  PRIVATE
  robot_core
  mongoose_embedded
  freertos_kernel
  pico_stdlib
  pico_multicore
  pico_cyw43_arch_lwip_sys_freertos
  hardware_clocks
  hardware_gpio
  hardware_pio
  hardware_pwm
  hardware_uart
  hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_ab_wifi_stub_control_usb_settle 0)
pico_enable_stdio_usb(robot_pico_ab_wifi_stub_control_usb_settle 1)
pico_add_extra_outputs(robot_pico_ab_wifi_stub_control_usb_settle)

add_executable(
  robot_pico_ab_wifi_cyw43_tuned_usb_settle
  main.cpp
  src/app/pico_firmware_app.cpp
  src/platform/pico_service_core.cpp
  src/platform/freertos_hooks.cpp
  src/transport/mongoose_http_server.cpp
  src/hardware/pico_motor_output_driver.cpp
  src/hardware/pico_quadrature_counter.cpp
  src/platform/pico_watchdog.cpp
)

pico_generate_pio_header(
  robot_pico_ab_wifi_cyw43_tuned_usb_settle
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_counter.pio
)

target_include_directories(
  robot_pico_ab_wifi_cyw43_tuned_usb_settle
  PRIVATE
  ../include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(
  robot_pico_ab_wifi_cyw43_tuned_usb_settle
  PRIVATE
  CYW43_TASK_PRIORITY=6
  CYW43_TASK_STACK_SIZE=4096
  PICO_CYW43_ARCH_DEBUG_ENABLED=1
  ROBOT_PICO_WIFI_INIT_WATCHDOG=0
  ROBOT_PICO_WIFI_WAIT_FOR_USB_MS=5000
  ROBOT_PICO_WIFI_USB_SETTLE_MS=2000
)

target_link_libraries(
  robot_pico_ab_wifi_cyw43_tuned_usb_settle
  PRIVATE
  robot_core
  mongoose_embedded
  freertos_kernel
  pico_stdlib
  pico_multicore
  pico_cyw43_arch_lwip_sys_freertos
  hardware_clocks
  hardware_gpio
  hardware_pio
  hardware_pwm
  hardware_uart
  hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_ab_wifi_cyw43_tuned_usb_settle 0)
pico_enable_stdio_usb(robot_pico_ab_wifi_cyw43_tuned_usb_settle 1)
pico_add_extra_outputs(robot_pico_ab_wifi_cyw43_tuned_usb_settle)

add_executable(
  robot_pico_ab_wifi_stub_control_cyw43_trace
  main.cpp
  src/app/pico_firmware_app.cpp
  src/platform/pico_service_core.cpp
  src/platform/freertos_hooks.cpp
  src/transport/mongoose_http_server.cpp
  src/hardware/pico_motor_output_driver.cpp
  src/hardware/pico_quadrature_counter.cpp
  src/platform/pico_watchdog.cpp
  src/diagnostics/cyw43_init_wraps.cpp
)

pico_generate_pio_header(
  robot_pico_ab_wifi_stub_control_cyw43_trace
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_counter.pio
)

target_include_directories(
  robot_pico_ab_wifi_stub_control_cyw43_trace
  PRIVATE
  ../include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ../third_party/mongoose
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(
  robot_pico_ab_wifi_stub_control_cyw43_trace
  PRIVATE
  ROBOT_PICO_SKIP_CONTROL_TASK=1
  ROBOT_PICO_SKIP_CONTROLLER_UART=1
  ROBOT_PICO_STUB_CONTROL_HARDWARE=1
  ROBOT_PICO_WIFI_INIT_WATCHDOG=0
  ROBOT_PICO_WIFI_WAIT_FOR_USB_MS=5000
  ROBOT_PICO_WIFI_USB_SETTLE_MS=2000
)

target_link_options(
  robot_pico_ab_wifi_stub_control_cyw43_trace
  PRIVATE
  -Wl,--wrap=async_context_freertos_init
  -Wl,--wrap=cyw43_driver_init
  -Wl,--wrap=lwip_freertos_init
  -Wl,--wrap=sys_mbox_new
  -Wl,--wrap=sys_mutex_new
  -Wl,--wrap=sys_thread_new
  -Wl,--wrap=xTaskCreate
  -Wl,--wrap=xTaskCreateAffinitySet
  -Wl,--wrap=pico_lwip_custom_lock_tcpip_core
  -Wl,--wrap=pico_lwip_custom_unlock_tcpip_core
  -Wl,--wrap=xQueueGenericSend
  -Wl,--wrap=xQueueSemaphoreTake
  -Wl,--wrap=sys_arch_mbox_fetch
  -Wl,--wrap=sys_timeouts_sleeptime
)

target_link_libraries(
  robot_pico_ab_wifi_stub_control_cyw43_trace
  PRIVATE
  robot_core
  mongoose_embedded
  freertos_kernel
  pico_stdlib
  pico_multicore
  pico_cyw43_arch_lwip_sys_freertos
  hardware_clocks
  hardware_gpio
  hardware_pio
  hardware_pwm
  hardware_uart
  hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_ab_wifi_stub_control_cyw43_trace 0)
pico_enable_stdio_usb(robot_pico_ab_wifi_stub_control_cyw43_trace 1)
pico_add_extra_outputs(robot_pico_ab_wifi_stub_control_cyw43_trace)

add_executable(
  robot_pico_ab_cyw43_probe
  src/diagnostics/cyw43_freertos_probe.cpp
  src/platform/freertos_hooks.cpp
)

target_include_directories(
  robot_pico_ab_cyw43_probe
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(
  robot_pico_ab_cyw43_probe
  PRIVATE
  freertos_kernel
  pico_stdlib
  pico_cyw43_arch_lwip_sys_freertos
  hardware_watchdog
)

pico_enable_stdio_uart(robot_pico_ab_cyw43_probe 0)
pico_enable_stdio_usb(robot_pico_ab_cyw43_probe 1)
pico_add_extra_outputs(robot_pico_ab_cyw43_probe)

function(add_robot_pico_cyw43_probe_variant target_name)
  add_executable(
    ${target_name}
    src/diagnostics/cyw43_freertos_probe.cpp
    src/platform/freertos_hooks.cpp
  )

  target_include_directories(
    ${target_name}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
  )

  target_link_libraries(
    ${target_name}
    PRIVATE
    freertos_kernel
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    hardware_watchdog
  )

  target_compile_definitions(${target_name} PRIVATE ${ARGN})

  pico_enable_stdio_uart(${target_name} 0)
  pico_enable_stdio_usb(${target_name} 1)
  pico_add_extra_outputs(${target_name})
endfunction()

add_robot_pico_cyw43_probe_variant(
  robot_pico_ab_cyw43_probe_no_watchdog
  ROBOT_PICO_CYW43_PROBE_WATCHDOG=0
)

add_robot_pico_cyw43_probe_variant(
  robot_pico_ab_cyw43_probe_usb_settle
  ROBOT_PICO_CYW43_PROBE_WATCHDOG=0
  ROBOT_PICO_CYW43_PROBE_WAIT_FOR_USB_MS=5000
  ROBOT_PICO_CYW43_PROBE_USB_SETTLE_MS=2000
)

add_robot_pico_cyw43_probe_variant(
  robot_pico_ab_cyw43_probe_no_affinity
  ROBOT_PICO_CYW43_PROBE_WATCHDOG=0
  ROBOT_PICO_CYW43_PROBE_TASK_AFFINITY=0
)
