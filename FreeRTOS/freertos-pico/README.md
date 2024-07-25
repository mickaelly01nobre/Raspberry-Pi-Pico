## Development Environment

To use FreeRTOS as the Embedded Operating System on the Raspberry Pi Pico, it is necessary to configure the development environment. The required tools for this configuration are: Git, CMake, Make, and the GNU Arm Embedded Toolchain.
Below are the commands for installing these tools:

~~~
$ sudo apt install cmake

$ sudo apt install gcc-arm-none-eabi

$ sudo apt install build-essential
~~~
After completing the specific operating system configuration, it is necessary to install the C/C++ SDK (used on the Raspberry Pi Pico) and FreeRTOS. As shown below:

~~~
$ mkdir freertos-pico

$ cd freertos-pico

$ git clone https://github.com/RaspberryPi/pico-sdk --recurse-submodules

$ git clone -b smp https://github.com/FreeRTOS/FreeRTOS-Kernel --recurse-submodules

$ exportar PICO_SDK_PATH=$PWD/pico-sdk

$ exportar FREERTOS_KERNEL_PATH=$PWD/FreeRTOS-Kernel
~~~


