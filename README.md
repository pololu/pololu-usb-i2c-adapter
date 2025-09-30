# Pololu Isolated USB-to-I²C Adapter software and drivers

[www.pololu.com](https://www.pololu.com/)

## Summary

This repository contains the firmware source code, drivers, and a
Python library for the following Pololu products:

* [Pololu Isolated USB-to-I²C Adapter][5396]
* [Pololu Isolated USB-to-I²C Adapter with Isolated Power][5397]

[5396]: https://www.pololu.com/product/5396
[5397]: https://www.pololu.com/product/5397


## Python library

The `python` directory of this repository contains a Python library
that implements the adapter's protocol, making it easy to control
the adapter from a Python program running on PC.

See [python/README.md](python/README.md) for more details.


## Building the firmware

After installing the `arm-none-eabi-gcc` toolchain, CMake, and Ninja,
navigate to the `firmware` directory in your shell and run these commands
to build the firmware:

  cmake --preset release
  cd build_release
  cmake --build .

Alternatively, you can compile the firmware using the
[STM32 extension for VS Code](https://www.st.com/content/st_com/en/campaigns/stm32-vs-code-extension-z11.html).


## Getting the board into bootloader mode

To update the firmware on your board, you must first get it into bootloader
mode, which means it is running the bootloader in the STM32's system memory.

The simplest way to start the bootloader is to open the adapter's virtual
serial port with a serial terminal program, set the baud rate to 600,
and then close it.  Some general-purpose terminal programs that work for this are
[Google Chrome Labs serial terminal], [Tera Term], and [PuTTY].

[PuTTY]: https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html
[Tera Term]: https://ttssh2.osdn.jp/index.html.en
[Google Chrome Labs serial terminal]: https://googlechromelabs.github.io/serial-terminal/

If that method does not work, you can short together two exposed pads while
powering up the board to force it into bootloader mode.
Contact Pololu for more information.  <!-- TODO: just add pictures of the pads -->


## Updating the firmware using the bootloader

After getting the board into bootloader mode, you can use the bootloader to
update the firmware.  First install the [STM32CubeProgrammer software] from
ST.  You can either load the firmware using the GUI provided by that software,
or the `STM32_Programmer_CLI` command-line interface.  The `firmware/program.sh`
script in this repository shows an example of how to use the CLI.

[STM32CubeProgrammer]: https://www.st.com/en/development-tools/stm32cubeprog.html


## Configuring the code style in STM32CubeIDE

Go to Window > Preferences > C/C++ > Code Style > Formatter.  Select
"BSD/Allman" and click Edit.  Set the Tab Policy to "Spaces only" and set the
indentation size to 2.  Set the Profile name to "Pololu" and click OK.
(This must be done once for each workspace.)

Now you can use the "Correct Indentation" or "Format" commands in the "Source"
menu on your code.


## Third-party libraries

This firmware includes code from the following third-party libraries:

- [TinyUSB]
- [cmsis-device-c0]
- [STM32CubeC0]

[TinyUSB]: https://github.com/STMicroelectronics/cmsis-device-c0
[cmsis-device-c0]: https://github.com/STMicroelectronics/cmsis-device-c0
[STM32CubeC0]: https://github.com/STMicroelectronics/STM32CubeC0


## Drivers

The `drivers` directory of this repository contains a USB driver
for Windows enables it to recognize the adapter.
You can install the driver by right-clicking on `pololu-usb-i2c-adapter.inf`
and selecting "Install".

Driver installation is not required on Windows 10 and later,
but installing the driver is recommended because it allows you to see the
correct product name in the Device Manager instead of "USB Serial Device".
