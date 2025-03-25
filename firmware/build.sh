#!/usr/bin/env bash

# Usage: ./build.sh [release|debug]

set -ue

CONFIG=${1:-release}
shopt -s nocasematch
if [[ "$CONFIG" == "release" ]]; then
    CONFIG=release
    CONFIG_CFLAGS=" -DNDEBUG -Os"
elif [[ "$CONFIG" == "debug" ]]; then
    CONFIG=debug
    CONFIG_CFLAGS=" -DDEBUG -Og -g3"
else
    echo "Invalid argument. Please use 'release' or 'debug', or no argument."
    exit 1
fi

FLAGS="-mcpu=cortex-m0plus --specs=nano.specs -mfloat-abi=soft -mthumb"
CFLAGS="$FLAGS -std=gnu11 -DF_CPU=48000000 -DSTM32C071G8Ux -DSTM32C071xx -DSTM32C0 -DSTM32"
CFLAGS+=" -I../src -I../st -I../tinyusb"
CFLAGS+=" -Wall -Wextra"
CFLAGS+=" -ffunction-sections -fdata-sections -fno-strict-aliasing -fwrapv"
CFLAGS+=" $CONFIG_CFLAGS"
LDFLAGS="$FLAGS -Wl,--gc-sections"
OUTNAME=firmware

cd "$(dirname "$0")"
mkdir -p build_$CONFIG
cd build_$CONFIG
rm -f *.o

set -x
arm-none-eabi-gcc "../src/main.c" -c $CFLAGS
arm-none-eabi-gcc "../src/system.c" -c $CFLAGS
arm-none-eabi-gcc "../tinyusb/stm32_fsdev/dcd_stm32_fsdev.c" -c $CFLAGS
arm-none-eabi-gcc "../tinyusb/device/usbd.c" -c $CFLAGS
arm-none-eabi-gcc "../tinyusb/device/usbd_control.c" -c $CFLAGS
arm-none-eabi-gcc "../tinyusb/common/tusb_fifo.c" -c $CFLAGS
arm-none-eabi-gcc "../tinyusb/class/cdc/cdc_device.c" -c $CFLAGS
arm-none-eabi-gcc "../tinyusb/tusb.c" -c $CFLAGS
arm-none-eabi-gcc -x assembler-with-cpp "../src/startup_stm32c071rbtx.s" -c $FLAGS
arm-none-eabi-gcc "../src/usb_descriptors.c" -c $CFLAGS
arm-none-eabi-gcc -o "$OUTNAME.elf" *.o $LDFLAGS -T"../STM32C071G8UX_FLASH.ld" -Wl,-Map="$OUTNAME.map"

arm-none-eabi-objdump -h -S firmware.elf  > firmware.list
arm-none-eabi-size firmware.elf
set +x
echo "Firmware is in build_$CONFIG/firmware.elf"
