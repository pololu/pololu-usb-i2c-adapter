# Python library for talking to the Pololu Isolated USB-to-I2C Adapter.
#
# Copyright (C) 2025 Pololu Corporation

import serial
import struct

ERROR_NONE = 0
ERROR_PROTOCOL = 1
ERROR_PREVIOUS_TIMEOUT = 2
ERROR_TIMEOUT = 3
ERROR_ADDRESS_TIMEOUT = 4
ERROR_TX_TIMEOUT = 5
ERROR_RX_TIMEOUT = 6
ERROR_NACK = 7
ERROR_ADDRESS_NACK = 8
ERROR_TX_DATA_NACK = 9
ERROR_BUS_ERROR = 10
ERROR_ARBITRATION_LOST = 11
ERROR_OTHER = 12
ERROR_NOT_SUPPORTED = 13

I2C_STANDARD_MODE = 0
I2C_FAST_MODE = 1
I2C_FAST_MODE_PLUS = 2
I2C_10_KHZ = 3

class AdapterError(RuntimeError):
    def __init__(self, error_code):
        if error_code == ERROR_PROTOCOL:
            msg = "Protocol error"
        elif error_code == ERROR_PREVIOUS_TIMEOUT:
            msg = "Timeout from previous command."
        elif error_code == ERROR_TIMEOUT:
            msg = "Timeout"
        elif error_code == ERROR_ADDRESS_TIMEOUT:
            msg = "Timeout while sending address"
        elif error_code == ERROR_TX_TIMEOUT:
            msg = "Timeout while transmtiting"
        elif error_code == ERROR_TX_TIMEOUT:
            msg = "Timeout while receiving"
        elif error_code == ERROR_ADDRESS_NACK:
            msg = "Target device did not respond"
        elif error_code == ERROR_TX_DATA_NACK:
            msg = "Received NACK for TX data"
        elif error_code == ERROR_ARBITRATION_LOST:
            msg = "Arbitration lost"
        elif error_code == ERROR_NOT_SUPPORTED:
            msg = "Operation not supported"
        else:
            msg = f"Error code {error_code}"
        super().__init__(msg)
        self.error_code = error_code

    def __str__(self):
        return f"{self.args[0]} ({self.error_code})."

class UsbToI2CAdapter():
    def __init__(self, port=None):
        if port:
            self.connect(port)

    def connect(self, port):
        if isinstance(port, str):
            port = serial.Serial(port, baudrate=115200, timeout=0.1)
        self.port = port
        self.port.read()  # discard old received data

    def is_connected(self):
        return self.port != None

    def _check_response_length(self, response, expected_len):
        if response is None or len(response) < expected_len:
           raise RuntimeError(f"Timeout while reading response from adapter " \
               f"(received {len(response)} bytes).")

    def _check_response(self, response, expected_len):
        self._check_response_length(response, expected_len)
        error_code = response[0]
        if error_code:
            raise AdapterError(error_code)

    # Writes data to the I2C target.
    #
    # 'address' is the 7-bit I2C address to write to.
    # 'data' is a bytes or bytearray object with the data to write.
    #
    # Raises an exception if the I2C address is not acknowledged by the target,
    # if any of the data bytes is not acknowledged, or if there is a timeout
    # due to clock stretching.
    # Returns the number of bytes transferred (always len(data)).
    def write_to(self, address, data):
      if len(data) > 255:
          raise RuntimeError("Cannot write more than 255 bytes.")

      cmd = b'\x91' + address.to_bytes(1, 'little') + \
          len(data).to_bytes(1, 'little') + data
      self.port.write(cmd)

      response = self.port.read(1)
      self._check_response(response, 1)

      return len(data)

    # Reads data from an I2C target.
    #
    # 'address' is the 7-bit I2C address to read from.
    # 'count' is the number of bytes to read.
    #
    # Raises an exception if the I2C address is not acknowledged by the target
    # or if there is a timeout due to clock stretching.
    # Returns the data received, which will always have the specified number
    # of bytes.
    def read_from(self, address, count):
        if count > 255:
            raise RuntimeError("Cannot read more than 255 bytes.")

        cmd = b'\x92' + address.to_bytes(1, 'little') + \
            count.to_bytes(1, 'little')
        self.port.write(cmd)

        response = self.port.read(1 + count)

        self._check_response(response, 1 + count)

        return response[1:]

    def set_i2c_frequency(self, frequency_khz):
        if frequency_khz >= 1000: mode = I2C_FAST_MODE_PLUS
        elif frequency_khz >= 400: mode = I2C_FAST_MODE
        elif frequency_khz >= 100: mode = I2C_STANDARD_MODE
        else: mode = I2C_10_KHZ
        cmd = b'\x94' + mode.to_bytes(1, 'little')
        self.port.write(cmd)

    def set_i2c_timeout(self, timeout_ms):
        cmd = b'\x97' + struct.pack("<H", timeout_ms)
        self.port.write(cmd)

    def clear_bus(self):
        self.port.write(b'\x98')

    def set_stm32_timing(self, timingr, gpio_fmp_mode):
        cmd = b'\xA1' + struct.pack("<Lb", timingr, gpio_fmp_mode)
        self.port.write(cmd)

    # Gets digital readings of SCL, SDA, and BOOT0 to help debug problems.
    def digital_read(self):
        self.port.write(b'\xA2')
        response = self.port.read(1)
        self._check_response_length(response, 1)
        r = response[0]
        return { 'SCL': r >> 0 & 1, 'SDA': r >> 1 & 1 }

    # Enables or disables the on-board regulator.
    def set_regulator(self, enabled):
        self.port.write(b'\xA4' + (b'\1' if enabled else b'\0'))
        response = self.port.read(1)
        self._check_response(response, 1)

    def get_device_info_raw(self):
        self.port.write(b'\xA7')
        part1 = self.port.read(1)
        self._check_response_length(part1, 1)
        length = part1[0]
        part2 = self.port.read(length - 1)
        self._check_response_length(part2, length - 1)
        return part1 + part2

    # Returns the binary string from the device describing info like its
    # firmware version.
    def get_device_info(self):
        raw = self.get_device_info_raw()
        parts = struct.unpack_from("<xBHHH8s12s", raw)
        if parts[0] != 0:
            raise RuntimeError("Unrecognized device info version: %d\n" % parts[0])

        firmware_modification = parts[4].split(b'\0', 1)[0].decode('ascii', errors='ignore')
        if firmware_modification == '-': firmware_modification = None

        firmware_version = "%x.%02x" % (parts[3] >> 8, parts[3] & 0xFF)
        if firmware_modification: firmware_version += firmware_modification

        sn = parts[5]
        serial_number = '-'.join(f"{a:02X}{b:02X}" for a, b in zip(sn[::2], sn[1::2]))

        info = {
            'vendor_id': parts[1],
            'product_id': parts[2],
            'firmware_version': firmware_version,
            'firmware_version_bcd': parts[3],
            'firmware_modification': firmware_modification,
            'serial_number': serial_number,
        }
        return info

    # Performs multiple commands with a single serial port write
    # and a single serial port read.
    #
    # Note: We don't know exactly how many commands you can pass to this
    # function.  At some point, if there too many commands, they fill up all
    # the buffers in the OS and the adapter, and the write would timeout.
    # If that happens, this function could be improved to interleave the
    # writing and the reading.
    #
    # Example usage:
    #   commands = [
    #     ['write_to', vl53l1x_address, b'\x01\x0F' ],
    #     ['read_from', vl53l1x_address, 2 ]
    #   ]
    #   responses = i2c.multiple_commands(commands)
    def multiple_commands(self, commands):
        cmd_bytes = bytearray()
        response_size = 0
        for command in commands:
            if command[0] == 'write_to':
                _, address, data = command
                cmd_bytes += b'\x91' + address.to_bytes(1, 'little') + \
                    len(data).to_bytes(1, 'little') + data
                response_size += 1
            elif command[0] == 'read_from':
                _, address, count = command
                cmd_bytes += b'\x92' + address.to_bytes(1, 'little') + \
                    count.to_bytes(1, 'little')
                response_size += 1 + count
            else:
                raise RuntimeError(f"Unknown command: {command[0]}.")

        self.port.write(cmd_bytes)
        full_response = self.port.read(response_size)
        self._check_response_length(full_response, response_size)

        response_index = 0
        responses = []
        for command in commands:
            response_size = None
            if command[0] == 'write_to':
                response_size = 1
            elif command[0] == 'read_from':
                response_size = 1 + command[2]
            responses.append(full_response[response_index:response_index + response_size])
            response_index += response_size
        return responses

    def scan(self):
        addresses_to_scan = range(128)
        cmd_bytes = b''
        for address in addresses_to_scan:
            cmd_bytes += b'\x91' + address.to_bytes(1, 'little') + b'\0'
        self.port.write(cmd_bytes)
        full_response = self.port.read(len(addresses_to_scan))
        self._check_response_length(full_response, len(addresses_to_scan))
        addresses_found = []
        for i in range(len(addresses_to_scan)):
            if full_response[i] == 0:
                addresses_found.append(addresses_to_scan[i])
            elif full_response[i] != ERROR_ADDRESS_NACK:
                raise RuntimeError(
                    "Unexpected error when scanning address %d: error code %d." % \
                    (addresses_to_scan[i], full_response[i]))
        return addresses_found

    # For compatibility with MicroPython's I2C class.
    def writeto(self, address, data, stop=True):
        if stop is not True:
            raise RuntimeError("The 'stop' argument must be True.")
        return self.write_to(address, data)

    # For compatibility with MicroPython's I2C class.
    def readfrom(self, address, count, stop=True):
        if stop is not True:
            raise RuntimeError("The 'stop' argument must be True.")
        return self.read_from(address, count)

