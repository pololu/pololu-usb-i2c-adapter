#!/usr/bin/env bash

# This bash script programs the firmware to a board using
# the STM32_Programmer_CLI utility from STM32CubeProgrammer.
# The board must already be in bootloader mode.
#
# Usage example: ./program.sh Release/firmware.elf

set -uex
STM32_Programmer_CLI -c port=USB1 \
  -ob NBOOT_SEL=0 NRST_MODE=1 BORF_LEV=2 BORR_LEV=2 BOR_EN=1 \
  -w $1 --go
