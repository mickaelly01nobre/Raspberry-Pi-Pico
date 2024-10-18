cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pull in FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(Gerenciamento_memoria C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()
add_library(mfrc mfrc522.h mfrc522.c)

target_link_libraries(mfrc pico_stdlib hardware_spi)

target_include_directories(mfrc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(Gerenciamento_memoria 
    main.c
    mfrc522.c  # Adicione o arquivo-fonte correto aqui
    	
)

target_include_directories(Gerenciamento_memoria PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Pull in common dependencies
target_link_libraries(Gerenciamento_memoria pico_stdlib hardware_adc hardware_pwm hardware_gpio FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

target_link_libraries(Gerenciamento_memoria pico_stdlib hardware_spi hardware_i2c hardware_gpio)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(Gerenciamento_memoria)

# Enable USB output, disable UART output
pico_enable_stdio_usb(Gerenciamento_memoria 1)
pico_enable_stdio_uart(Gerenciamento_memoria 0)

