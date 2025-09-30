cmake_minimum_required(VERSION 3.22)
project(firmware)
enable_language(C ASM)
set(CMAKE_C_STANDARD 11)
add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_descriptors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/startup_stm32c071rbtx.s
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/stm32_fsdev/dcd_stm32_fsdev.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/device/usbd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/device/usbd_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/common/tusb_fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/class/cdc/cdc_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/tusb.c
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/st
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb
)
target_compile_options(${PROJECT_NAME} PRIVATE
    -DUSB_FIRMWARE_VERSION=0x100
    -DUSB_MS_OS_20_DESCRIPTORS=1 -DUSB_CDC_ACM=1
    -mthumb -mcpu=cortex-m0plus -mfloat-abi=soft
    -DSTM32C071G8Ux -DSTM32C071xx -DSTM32C0 -DSTM32 -DF_CPU=48000000
    -ffunction-sections -fdata-sections
    -fwrapv -fno-strict-aliasing
    -Wall -Wextra
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:-Og -g3 -ggdb -DDEBUG>
    $<$<CONFIG:Release>:-Os -g0 -DNDEBUG>
)
target_link_options(${PROJECT_NAME} PRIVATE
    -mthumb -mcpu=cortex-m0plus -mfloat-abi=soft
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32C071G8UX_FLASH.ld
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--print-memory-usage
)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)
