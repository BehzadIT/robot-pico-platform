add_executable(
  robot_pico_control_no_wifi
  main.cpp
)

target_link_libraries(
  robot_pico_control_no_wifi
  PRIVATE
  pico_stdlib
)

target_compile_definitions(
  robot_pico_control_no_wifi
  PRIVATE
  ROBOT_PICO_APP_NAME="robot_pico_control_no_wifi"
  ROBOT_PICO_APP_STAGE="clean_bench_loop"
  PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
  PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL=1
  PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT=1
)

pico_enable_stdio_uart(robot_pico_control_no_wifi 0)
pico_enable_stdio_usb(robot_pico_control_no_wifi 1)
pico_add_extra_outputs(robot_pico_control_no_wifi)
