![](./resources/official_armmbed_example_badge.png)
# MBEDDED SYSTEMS PLAN MONITORING FINAL PROJECT


## Mbed OS build tools

### Mbed CLI 2
Starting with version 6.5, Mbed OS uses Mbed CLI 2. It uses Ninja as a build system, and CMake to generate the build environment and manage the build process in a compiler-independent manner. If you are working with Mbed OS version prior to 6.5 then check the section [Mbed CLI 1](#mbed-cli-1).
1. [Install Mbed CLI 2](https://os.mbed.com/docs/mbed-os/latest/build-tools/install-or-upgrade.html).
1. From the command-line, import the example: `mbed-tools import mbed-os-example-blinky`
1. Change the current directory to where the project was imported.

### Mbed CLI 1
1. [Install Mbed CLI 1](https://os.mbed.com/docs/mbed-os/latest/quick-start/offline-with-mbed-cli.html).
1. From the command-line, import the example: `mbed import mbed-os-example-blinky`
1. Change the current directory to where the project was imported.

## Application functionality

The `main()` function is the single thread in the application. It toggles the state of a digital output connected to an LED on the board.

**Note**: This example requires a target with RTOS support, i.e. one with `rtos` declared in `supported_application_profiles` in `targets/targets.json` in [mbed-os](https://github.com/ARMmbed/mbed-os). For non-RTOS targets (usually with small memory sizes), please use [mbed-os-example-blinky-baremetal](https://github.com/ARMmbed/mbed-os-example-blinky-baremetal) instead.

Main Class Functionality:

Listen for the user button rise, in case that happens the method button_isr()is invoked and the operation mode changes.
Turn off the RGB LED light.
Enable the RGB Sensor and validate if it is present.
Initiate the GPS Serial object
Create a Timer object for future sensor reading
Establish the GPS Connections
Start the timer to count.
While loop functionality:
   Check which operation mode is selected by the user(0: test, 1:normal, 2:advanced) and with conditionals turn on a board LED (test mode: LED1, normal mode: LED2 and advanced mode: LED3).
   Query the GPS for synchronizing.
   Check if the timer has reached the refresh time value.
      In case that happens, the timer is reset and the methods for reading the sensors are called (read_soil_moisture(); read_light();           read_temp_hum(); read_sensorRGB(); read_accel();)
      Also the values that the sensors returned are printed with console output.
      Check if the operation mode is in normal mode
         In case the system is in normal mode, call the method validateLimits(); to check if any sensor value is out of range and take action on it.
         Increment by 1 a counter variable (for 1 hour statistics future use).
         Call the maxMinSum() method for the temperature, humidity, soil moisture and light sensor current values. And update the arrays of Max, Min, and sum.
         Check if the counter have reach the value of “120”, to show the statistics, the logic for this is the following: 
         In normal mode the frequency for showing the data is 30 seconds each time the data is shown, the counter will add 1 to its current value, so when the counter reaches 120, that will mean that 3600 seconds(1 hour) have passed. After printing all the statistics, we call the method resetArr() to reset all the Min, Max and Sum values of the sensor.
