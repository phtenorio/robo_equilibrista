# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Set any variables required for importing libraries
if (DEFINED ENV{FREERTOS_PATH})
  SET(FREERTOS_PATH $ENV{FREERTOS_PATH})
else()
  SET(FREERTOS_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS)
endif()

message("FreeRTOS Kernel located in ${FREERTOS_PATH}")

# Import those libraries
include(${FREERTOS_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(robo_equilibrista C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(robo_equilibrista
         src/main.c
         src/mpu6050.c
         src/hardware_init.c
         src/motor.c )

pico_set_program_name(robo_equilibrista "robo_equilibrista")
pico_set_program_version(robo_equilibrista "0.1")

target_compile_definitions(robo_equilibrista PRIVATE PICO_WIFI_NO_POWERSAVE)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(robo_equilibrista 0)
pico_enable_stdio_usb(robo_equilibrista 1)

# Add the standard library to the build
target_link_libraries(robo_equilibrista
        FreeRTOS-Kernel-Heap4 
        hardware_i2c
        hardware_pwm
        hardware_dma
        hardware_timer
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(robo_equilibrista PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(robo_equilibrista)

