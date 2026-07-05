cmake_minimum_required(VERSION 3.20)

option(ROBOT_PICO_BUILD_TESTS "Build host-side parity tests" ON)
option(ROBOT_PICO_ENABLE_FIRMWARE "Enable the Pico SDK firmware target" OFF)
option(ROBOT_PICO_FIRMWARE_DIAGNOSTIC_O1
       "Build the Pico firmware with -O1 for RP2350/CYW43 diagnostics" ON)

if(ROBOT_PICO_ENABLE_FIRMWARE)
    if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    endif()
    if(NOT PICO_SDK_PATH)
        message(FATAL_ERROR "PICO_SDK_PATH must be set when ROBOT_PICO_ENABLE_FIRMWARE=ON")
    endif()
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(robot_pico_platform_cpp LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(ROBOT_PICO_ENABLE_FIRMWARE AND CMAKE_CROSSCOMPILING)
    set(ROBOT_PICO_BUILD_TESTS OFF CACHE BOOL "Build host-side parity tests" FORCE)
endif()

if(ROBOT_PICO_ENABLE_FIRMWARE AND ROBOT_PICO_FIRMWARE_DIAGNOSTIC_O1)
    # Pico 2 W / CYW43 initialization has shown optimization-sensitive failures
    # on some dual-core builds. Use -O1 as the diagnostic profile until the
    # lower-level driver boundary is proven stable.
    set(CMAKE_C_FLAGS_RELEASE "-g -O1 -DNDEBUG" CACHE STRING "Release C flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-g -O1 -DNDEBUG" CACHE STRING "Release C++ flags" FORCE)
    set(CMAKE_ASM_FLAGS_RELEASE "-g -O1 -DNDEBUG" CACHE STRING "Release ASM flags" FORCE)
endif()

add_library(
    robot_core
    src/control/controller_drive_mapper.cpp
    src/control/controller_uart_pipeline.cpp
    src/control/drivetrain_controller.cpp
    src/control/drive_command_manager.cpp
    src/control/drive_target_store.cpp
    src/control/pid.cpp
    src/hardware/drivetrain_encoder_subsystem.cpp
    src/hardware/motor_output_driver.cpp
    src/platform/controller_uart_runtime.cpp
    src/protocol/api_drive_request.cpp
    src/protocol/joystick_transport.cpp
    src/runtime/control_core_runtime.cpp
    src/runtime/service_core_runtime.cpp
    src/support/logger.cpp
)

target_include_directories(robot_core PUBLIC include)

if(ROBOT_PICO_BUILD_TESTS)
    include(CTest)
    enable_testing()

    add_executable(
        robot_core_tests
        tests/test_main.cpp
    )

    target_link_libraries(robot_core_tests PRIVATE robot_core)

    add_test(NAME robot_core_tests COMMAND robot_core_tests)
endif()

if(ROBOT_PICO_ENABLE_FIRMWARE)
    add_subdirectory(firmware)
endif()
