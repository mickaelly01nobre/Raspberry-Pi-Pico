cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pull in FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(semafaros C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(semafaros main.c)

target_include_directories(semafaros PRIVATE ${CMAKE_CURRENT_LIST_DIR})
 
# pull in common dependencies
target_link_libraries(semafaros pico_stdlib hardware_pwm hardware_gpio FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(semafaros)

pico_enable_stdio_usb(semafaros 1)

pico_enable_stdio_uart(semafaros 0)
