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
