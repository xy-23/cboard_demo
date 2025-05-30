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
set(CMAKE_PROJECT_NAME cboard)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

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
    # Add user sources here
    applications/led_task.cpp
    applications/buzzer_task.cpp
    applications/can_task.cpp
    applications/imu_task.cpp
    applications/uart_task.cpp
    applications/plot_task.cpp

    sp_middleware/io/led/led.cpp
    sp_middleware/io/buzzer/buzzer.cpp
    sp_middleware/io/can/can.cpp
    sp_middleware/io/bmi088/bmi088.cpp
    sp_middleware/io/plotter/plotter.cpp
    sp_middleware/io/dbus/dbus.cpp
    
    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/motor/dm_motor/dm_motor.cpp
    sp_middleware/motor/lk_motor/lk_motor.cpp

    sp_middleware/referee/pm02/pm02.cpp
    sp_middleware/referee/ui/element.cpp
    sp_middleware/referee/ui/manager.cpp
    sp_middleware/referee/vt02/vt02.cpp

    sp_middleware/tools/math_tools/math_tools.cpp
    sp_middleware/tools/pid/pid.cpp
    sp_middleware/tools/mahony/mahony.cpp
    sp_middleware/tools/crc/crc.cpp
    sp_middleware/tools/diff_drive/diff_drive.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    sp_middleware/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
