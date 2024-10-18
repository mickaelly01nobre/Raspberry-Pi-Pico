cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pull in FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(Mutex C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_library(ultrasonic ultrasonic.h ultrasonic.c)

target_link_libraries(ultrasonic pico_stdlib hardware_gpio hardware_timer)

target_include_directories(ultrasonic PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(Mutex main.c ultrasonic.c)

target_include_directories(Mutex PRIVATE ${CMAKE_CURRENT_LIST_DIR})
 
# pull in common dependencies
target_link_libraries(Mutex pico_stdlib hardware_adc  hardware_timer FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(Mutex)

pico_enable_stdio_usb(Mutex 1)

pico_enable_stdio_uart(Mutex 0)
