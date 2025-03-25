#!/usr/bin/env bash

# This script updates the files from STMicroelectonics in the "st" directory.

CMSIS_REV=main
CMSIS_URL=https://raw.githubusercontent.com/STMicroelectronics/cmsis-device-c0

CUBE_REV=main
CUBE_URL=https://raw.githubusercontent.com/STMicroelectronics/STM32CubeC0

cd "$(dirname "$0")"/st
set -x

curl --fail --show-error \
  $CMSIS_URL/$CMSIS_REV/LICENSE.md -o LICENSE \
  $CMSIS_URL/$CMSIS_REV/Include/stm32c071xx.h -o stm32c071xx.h \
  $CMSIS_URL/$CMSIS_REV/Include/stm32c0xx.h -o stm32c0xx.h \
  $CMSIS_URL/$CMSIS_REV/Include/system_stm32c0xx.h -o system_stm32c0xx.h \
  $CUBE_URL/$CUBE_REV/Drivers/CMSIS/Include/cmsis_compiler.h -o cmsis_compiler.h \
  $CUBE_URL/$CUBE_REV/Drivers/CMSIS/Include/cmsis_gcc.h -o cmsis_gcc.h \
  $CUBE_URL/$CUBE_REV/Drivers/CMSIS/Include/cmsis_version.h -o cmsis_version.h \
  $CUBE_URL/$CUBE_REV/Drivers/CMSIS/Include/core_cm0plus.h -o core_cm0plus.h \
  $CUBE_URL/$CUBE_REV/Drivers/CMSIS/Include/mpu_armv7.h -o mpu_armv7.h
