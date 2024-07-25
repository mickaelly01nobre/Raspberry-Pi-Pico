### Compilation and Execution

After downloading this project and configuring the development environment, some additional steps are necessary for the application to run on the Raspberry Pi Pico. Inside the "idle_Hook" folder, create another folder for the program to be compiled. Subsequently, an image will be generated, which will be used on the microcontroller.

As shown below:
~~~
$ mkdir build
$ cd build
$ cmake ..
$ make
~~~

Once the project successfully builds, there should now be a ‘idle_Hook.uf2’ in the ‘build’ directory. This file is the binary we will flash to the Pico. In order to flash this file, first hold down the BOOTSEL button on the Pico board while plugging it in to the USB interface. This will mount the Pico as a drive. Then copy the ‘blink.uf2’ file to the drive location and the Pico will automatically reboot and run the application.

