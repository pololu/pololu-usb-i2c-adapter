// This is a GPIO library for the STM32C071xx written by Pololu that
// chooses efficiency over safety.
// There is no automatic checking of the arguments, so an invalid argument
// could make your code write to incorrect registers or write invalid values.
// The pin numbers are 32-bit values that are designed to be very easy to decode
// by the GPIO functions, but that means they are not very nice for the user to
// look at and should be re-encoded if you want to store many of them compactly.
//
// Before using this library, you must enable the clock signal to the ports
// you want to use.  To enable all ports on the STM32C071xx, do:
//   RCC->IOPENR = 0x2F;

#pragma once

#include <stm32c0xx.h>
#include <stdint.h>
#include <stdbool.h>

// GPIO config bitmaps:
#define GPIO_INPUT                           0b00000
#define GPIO_INPUT_PULLED_UP                 0b01000
#define GPIO_INPUT_PULLED_DOWN               0b10000
#define GPIO_OUTPUT                          0b00001
#define GPIO_OUTPUT_OPEN_DRAIN               0b00101
#define GPIO_OUTPUT_OPEN_DRAIN_PULLED_UP     0b01101
#define GPIO_OUTPUT_OPEN_DRAIN_PULLED_DOWN   0b10101
#define GPIO_ALT_FUNC                        0b00010
#define GPIO_ALT_FUNC_OPEN_DRAIN             0b00110
#define GPIO_ALT_FUNC_OPEN_DRAIN_PULLED_UP   0b01110
#define GPIO_ALT_FUNC_OPEN_DRAIN_PULLED_DOWN 0b10110
#define GPIO_ANALOG                          0b00011

#define LOW    0
#define HIGH   1

#define PA0 (GPIOA_BASE | 0)
#define PA1 (GPIOA_BASE | 1)
#define PA2 (GPIOA_BASE | 2)
#define PA3 (GPIOA_BASE | 3)
#define PA4 (GPIOA_BASE | 4)
#define PA5 (GPIOA_BASE | 5)
#define PA6 (GPIOA_BASE | 6)
#define PA7 (GPIOA_BASE | 7)
#define PA8 (GPIOA_BASE | 8)
#define PA9 (GPIOA_BASE | 9)
#define PA10 (GPIOA_BASE | 10)
#define PA11 (GPIOA_BASE | 11)
#define PA12 (GPIOA_BASE | 12)
#define PA13 (GPIOA_BASE | 13)
#define PA14 (GPIOA_BASE | 14)
#define PA15 (GPIOA_BASE | 15)

#define PB0 (GPIOB_BASE | 0)
#define PB1 (GPIOB_BASE | 1)
#define PB2 (GPIOB_BASE | 2)
#define PB3 (GPIOB_BASE | 3)
#define PB4 (GPIOB_BASE | 4)
#define PB5 (GPIOB_BASE | 5)
#define PB6 (GPIOB_BASE | 6)
#define PB7 (GPIOB_BASE | 7)
#define PB8 (GPIOB_BASE | 8)
#define PB9 (GPIOB_BASE | 9)
#define PB10 (GPIOB_BASE | 10)
#define PB11 (GPIOB_BASE | 11)
#define PB12 (GPIOB_BASE | 12)
#define PB13 (GPIOB_BASE | 13)
#define PB14 (GPIOB_BASE | 14)
#define PB15 (GPIOB_BASE | 15)

#define PC0 (GPIOC_BASE | 0)
#define PC1 (GPIOC_BASE | 1)
#define PC2 (GPIOC_BASE | 2)
#define PC3 (GPIOC_BASE | 3)
#define PC4 (GPIOC_BASE | 4)
#define PC5 (GPIOC_BASE | 5)
#define PC6 (GPIOC_BASE | 6)
#define PC7 (GPIOC_BASE | 7)
#define PC8 (GPIOC_BASE | 8)
#define PC9 (GPIOC_BASE | 9)
#define PC10 (GPIOC_BASE | 10)
#define PC11 (GPIOC_BASE | 11)
#define PC12 (GPIOC_BASE | 12)
#define PC13 (GPIOC_BASE | 13)
#define PC14 (GPIOC_BASE | 14)
#define PC15 (GPIOC_BASE | 15)

#define PD0 (GPIOD_BASE | 0)
#define PD1 (GPIOD_BASE | 1)
#define PD2 (GPIOD_BASE | 2)
#define PD3 (GPIOD_BASE | 3)
#define PD4 (GPIOD_BASE | 4)
#define PD5 (GPIOD_BASE | 5)
#define PD6 (GPIOD_BASE | 6)
#define PD7 (GPIOD_BASE | 7)
#define PD8 (GPIOD_BASE | 8)
#define PD9 (GPIOD_BASE | 9)
#define PD10 (GPIOD_BASE | 10)
#define PD11 (GPIOD_BASE | 11)
#define PD12 (GPIOD_BASE | 12)
#define PD13 (GPIOD_BASE | 13)
#define PD14 (GPIOD_BASE | 14)
#define PD15 (GPIOD_BASE | 15)

#define PF0 (GPIOF_BASE | 0)
#define PF1 (GPIOF_BASE | 1)
#define PF2 (GPIOF_BASE | 2)
#define PF3 (GPIOF_BASE | 3)
#define PF4 (GPIOF_BASE | 4)
#define PF5 (GPIOF_BASE | 5)
#define PF6 (GPIOF_BASE | 6)
#define PF7 (GPIOF_BASE | 7)
#define PF8 (GPIOF_BASE | 8)
#define PF9 (GPIOF_BASE | 9)
#define PF10 (GPIOF_BASE | 10)
#define PF11 (GPIOF_BASE | 11)
#define PF12 (GPIOF_BASE | 12)
#define PF13 (GPIOF_BASE | 13)
#define PF14 (GPIOF_BASE | 14)
#define PF15 (GPIOF_BASE | 15)

#define _DECODE_PIN() GPIO_TypeDef * port = (void *)(pin & ~0xFF); uint32_t y = (pin & 0x0F);

// Configure the specified pin.
//
// The config argument should be one of the macros in the
// "GPIO config bitmaps" section above.
//
// This function is NOT safe to call in an interrupt: it does
// read-modify-write operations.
static inline void gpio_config(uint32_t pin, uint32_t config)
{
  _DECODE_PIN();

  // config bits 0:1 -> MODERy[1:0]
  port->MODER = (port->MODER & ~(3 << y << y)) | (config & 3) << y << y;

  // config bit 2    -> OTy
  port->OTYPER = (port->OTYPER & ~(1 << y)) | (config >> 2 & 1) << y;

  // config bits 3:4 -> PUPDy[1:0]
  port->PUPDR = (port->PUPDR & ~(3 << y << y)) | (config >> 3 & 3) << y << y;
}

// Changes the MODEy bits of a pin.  This is more limited than gpio_config()
// but fine for many purposes, such as switching a pin between output and
// input without touching other settings.
//
// The mode argument should be GPIO_INPUT, GPIO_OUTPUT, GPIO_ALT_FUNC, or
// GPIO_ANALOG.
//
// This function is NOT safe to call in an interrupt: it does a
// read-modify-write operation.
static inline void gpio_config_mode(uint32_t pin, uint32_t mode)
{
  _DECODE_PIN();
  port->MODER = (port->MODER & ~(3 << y << y)) | (mode & 3) << y << y;
}

// Returns GPIO_INPUT, GPIO_OUTPUT, GPIO_ALT_FUNC, or GPIO_ANALOG.
static inline uint8_t gpio_get_mode(uint32_t pin)
{
  _DECODE_PIN();
  return port->MODER >> y >> y & 3;
}

// Changes the AFSEL bits of the a pin to select what alternate function it
// will be.  Don't forget to all gpio_config_mode or gpio_config to put the
// pin in alternate function mode.
static inline void gpio_set_alt_func(uint32_t pin, uint32_t func)
{
  GPIO_TypeDef * port = (void *)(pin & ~0xFF);
  uint32_t i = pin >> 3 & 1;
  uint32_t s = (pin & 7) * 4;
  port->AFR[i] = (port->AFR[i] & ~(15 << s)) | (func & 0xF) << s;
}

// Sets the value of an OUTPUT pin.
static inline void gpio_write(uint32_t pin, bool value)
{
  _DECODE_PIN();
  port->BSRR = value ? 1 << y : 1 << y << 16;
}

// Reads the digital value of an input pin.
static inline bool gpio_read(uint32_t pin)
{
  _DECODE_PIN();
  return port->IDR >> y & 1;
}

#undef _DECODE_PIN
