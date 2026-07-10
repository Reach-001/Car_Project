cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Car_Project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # App 层
    App/Src/app_main.c
    App/Src/app_tasks.c
    App/Src/app_test.c
    App/Src/system_state_pool.c

    # BSP 驱动
    BSP/Src/bsp_buzzer.c
    BSP/Src/bsp_encoder.c
    BSP/Src/bsp_gpio_sensor.c
    BSP/Src/bsp_key.c
    BSP/Src/bsp_led.c
    BSP/Src/bsp_motor.c
    BSP/Src/bsp_servo.c
    BSP/Src/bsp_uart.c

    # V2 Module 层 ── Sensor 域
    Modules/Sensor/Src/sensor_domain.c
    Modules/Sensor/Src/tracker_sensor.c
    Modules/Sensor/Src/ultrasonic_sensor.c

    # V2 Module 层 ── Comm 域
    Modules/Comm/Src/comm.c
    Modules/Comm/Src/bt_link.c
    Modules/Comm/Src/k230_link.c
    Modules/Comm/Src/protocol.c

    # V2 Module 层 ── Estimation 域
    Modules/Estimation/Src/estimation.c

    # V2 Module 层 ── Motion 域
    Modules/Motion/Src/motion.c
    Modules/Motion/Src/ackermann.c
    Modules/Motion/Src/speed_pi.c

    # V2 Module 层 ── Decision 域
    Modules/Decision/Src/decision.c

    # V2 Module 层 ── HMI 域
    Modules/HMI/Src/hmi.c
    Modules/HMI/Src/key_service.c
    Modules/HMI/Src/led_service.c
    Modules/HMI/Src/buzzer_service.c

    # V2 Module 层 ── Debug 域
    Modules/Debug/Src/debug_trace.c

    # Service 层
    Service/Src/ring_buffer.c
    Service/Src/scheduler.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Inc
    BSP/Inc
    Modules/Sensor/Inc
    Modules/Comm/Inc
    Modules/Estimation/Inc
    Modules/Motion/Inc
    Modules/Decision/Inc
    Modules/HMI/Inc
    Modules/Debug/Inc
    Service/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
