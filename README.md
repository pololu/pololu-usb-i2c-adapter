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

### Configuring the code style in STM32CubeIDE

Go to Window > Preferences > C/C++ > Code Style > Formatter.  Select
"BSD/Allman" and click Edit.  Set the Tab Policy to "Spaces only" and set the
indentation size to 2.  Set the Profile name to "Pololu" and click OK.
(This must be done once for each workspace.)

Now you can use the "Correct Indentation" or "Format" commands in the "Source"
menu on your code.

## Building the firmware with Bash and GCC

TODO

## Uploading firmware to your device

TODO

## Updating third-party libraries

TODO


