cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pull in FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(LCD C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_library(lcd1602 lcd1602_i2c.h lcd1602_i2c.c)

target_link_libraries(lcd1602 pico_stdlib hardware_i2c)

target_include_directories(lcd1602 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(LCD main.c lcd1602_i2c.c)

target_include_directories(LCD PRIVATE ${CMAKE_CURRENT_LIST_DIR})
 
# pull in common dependencies
target_link_libraries(LCD pico_stdlib pico_multicore hardware_adc  hardware_timer hardware_i2c hardware_gpio FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(LCD)



pico_enable_stdio_usb(LCD 1)

pico_enable_stdio_uart(LCD 0)


