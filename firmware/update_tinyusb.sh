#!/usr/bin/env bash

# This script updates the files from STMicroelectonics in the "st" directory.

REV=master
URL=https://raw.githubusercontent.com/hathach/tinyusb

cd "$(dirname "$0")"/tinyusb
set -x

curl --fail --show-error \
  $URL/$REV/LICENSE -o LICENSE \
  $URL/$REV/src/tusb_option.h -o tusb_option.h \
  $URL/$REV/src/tusb.c -o tusb.c \
  $URL/$REV/src/tusb.h -o tusb.h \
  $URL/$REV/src/class/cdc/cdc_device.c -o class/cdc/cdc_device.c \
  $URL/$REV/src/class/cdc/cdc.h -o class/cdc/cdc.h \
  $URL/$REV/src/class/cdc/cdc_device.h -o class/cdc/cdc_device.h \
  $URL/$REV/src/common/tusb_common.h -o common/tusb_common.h \
  $URL/$REV/src/common/tusb_compiler.h -o common/tusb_compiler.h \
  $URL/$REV/src/common/tusb_debug.h -o common/tusb_debug.h \
  $URL/$REV/src/common/tusb_fifo.c -o common/tusb_fifo.c \
  $URL/$REV/src/common/tusb_fifo.h -o common/tusb_fifo.h \
  $URL/$REV/src/common/tusb_mcu.h -o common/tusb_mcu.h \
  $URL/$REV/src/common/tusb_private.h -o common/tusb_private.h \
  $URL/$REV/src/common/tusb_types.h -o common/tusb_types.h \
  $URL/$REV/src/common/tusb_verify.h -o common/tusb_verify.h \
  $URL/$REV/src/device/usbd.c -o device/usbd.c \
  $URL/$REV/src/device/usbd_control.c -o device/usbd_control.c \
  $URL/$REV/src/device/dcd.h -o device/dcd.h \
  $URL/$REV/src/device/usbd.h -o device/usbd.h \
  $URL/$REV/src/device/usbd_pvt.h -o device/usbd_pvt.h \
  $URL/$REV/src/osal/osal.h -o osal/osal.h \
  $URL/$REV/src/osal/osal_none.h -o osal/osal_none.h \
  $URL/$REV/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c -o stm32_fsdev/dcd_stm32_fsdev.c \
  $URL/$REV/src/portable/st/stm32_fsdev/fsdev_stm32.h -o stm32_fsdev/fsdev_stm32.h \
  $URL/$REV/src/portable/st/stm32_fsdev/fsdev_type.h -o stm32_fsdev/fsdev_type.h
