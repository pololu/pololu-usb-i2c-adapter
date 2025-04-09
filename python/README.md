# Pololu Isolated USB-to-I²C Adapter library for Python

[www.pololu.com](https://www.pololu.com/)

## Summary

This is a Python 3 library that helps interface with the
Pololu Isolated USB-to-I²C Adapters.


## Supported platforms

This library is designed to run on any Python 3 interpreter that has
[PySerial](https://pypi.org/project/pyserial/) or an equivalent library.


## Getting started

Download this code to your system, navigate to this directory, and then run
`pip3 install .` to install this library and PySerial.


## Example code

Here is some example Python code showing how to import the library and
read the "Model ID" and "Model Type" registers from a
[VL53L1X time-of-flight distance sensor][3415]:

```py
#!/usr/bin/env python3
import pololu_usb_i2c_adapter
i2c = pololu_usb_i2c_adapter.Adapter('/dev/ttyACM0')
address = 0b0101001
i2c.write_to(address, b'\x01\x0F')
data = i2c.read_from(address, 2)
print(data)
```

If everything is connected properly, the program should print
<code>b'\xea\xcc'</code>, confirming that the connected device is a VL53L1X.

The first line of Python code imports the library.

The second line creates an `Adapter` object and connects to the specified
serial port.  The port name you have to use here depends on your operating
system and the number of other CDC ACM devices that are connected, so you might
need to change it.

- On Windows, the port name will be something like "COM6", and you can
  determine it by the looking in the "Ports (COM & LPT)" category of the
  Device Manager.  The adapter will either appear as "USB Serial Device" or
  it will have a descriptive name starting with
  "Pololu Isolated USB-to-I2C Adapter".  You can also change the COM port
  number using the Device Manager.
- On Linux, you can run `ls /dev/ttyACM*` to list the USB serial ports connected
  to your system.  One of these will be the adapter.  If you have multiple
  ports in the list, you might consider using the symbolic links provided in
  `/dev/serial/by-id`, which do not depend on the order the devices were
  plugged in.
- On macOS, you can run `ls /dev/cu.usbmodem*` to list the USB serial ports
  connected to your system.  One of these will be the adapter.

The third line sets a variable named `adddress` to 41.  This is 7-bit
I²C address of the VL53L1X.

The fourth line writes the bytes 0x01 and 0x0F to the VL53L1X.  This tells
the VL53L1X that we want to select register 0x10F, which is the model ID.
Instead of writing <code>b'\x01\x0F'</code>, you can write
`(0x10F).to_bytes(2, 'big')`.

The fifth line reads two bytes from the VL53L1X and returns them as a Python
`bytes` object.  You can write `data[0]` or `data[1]` to get the value
of an individual byte as an integer, or you can use the `unpack` method in
Python's `struct` module to decode the binary data to desired format.

For more documentation, see the comments in pololu_usb_i2c_adapter.py.

[3415]: https://www.pololu.com/product/3415

