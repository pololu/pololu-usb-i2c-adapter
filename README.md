# Pololu Isolated USB-to-I2C Adapter: firmware and example code

[www.pololu.com](https://www.pololu.com/)


## Building the firmware with STM32CubeIDE

Download the [STM32CubeIDE] software from ST's website and install it.
Also, update to the latest version using '''Help > Check for Updates'''.

Open the IDE and select this folder (the folder with README.md) as the
workspace.

In the "Project" menu, select "Build All..." to build the project.

The IDE does not automatically detect projects in the workspace.  To open those
projects and build them, you have first go to File > Import... >
Existing Projects into Workspace.

[STM32CubeIDE]: https://www.st.com/en/development-tools/stm32cubeide.html


## Building the firmware with Bash and GCC

If you have the Bash shell and arm-none-eabi-gcc installed on your PATH, you
can build the firmware by running the `build.sh` script in the
`firmware` directory.  By default, it builds in release mode.  You can
specify what configuration to build by providing an argument to the script,
which should be `release` or `debug`.


## Getting the board into bootloader mode

To update the firmware on your board, you must first get it into bootloader mode,
which means it is running the bootloader the STM32's system memory.

The simplest way to start the bootloader is to open the adapter's virtual
serial port with a serial terminal program, set the baud rate to 600,
and then close it.  Some general-purpose terminal programs that work for this are
[Google Chrome Labs serial terminal], [Tera Term], and [PuTTY].

[PuTTY]: https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html
[Tera Term]: https://ttssh2.osdn.jp/index.html.en
[Google Chrome Labs serial terminal]: https://googlechromelabs.github.io/serial-terminal/

If that method does not work, you can short together two exposed pads while
powering up the board to force it into bootloader mode.
Contact Pololu for mor information.  <!-- TODO: just add pictures of the pads -->

## Updating the firmware using the bootloader

After getting the board into bootloader mode, you can use the bootloader to
update the firmware.  First install the [STM32CubeProgrammer] software from
ST.  You can either load the programmer using the GUI provided by that software,
or the `STM32_Programmer_CLI` command-line interface.  The `firmware/program.sh`
script in this repository shows an example of how to use the CLI.

<!-- TODO: mode details about using the GUI -->

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
