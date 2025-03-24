// Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

// system.h and system.c are a collection of useful routines written by
// Pololu for the STM32C071xx, which we anticipate using in many different
// firmware projects.

#pragma once

#include <stdint.h>
#include <stdbool.h>

extern volatile uint32_t time_ms;

// Return time_ms.  This makes it easier to use ST HAL libraries.
uint32_t HAL_GetTick(void);

// Call this at the beginning of main to set up the clock sources,
// enable the clocks needed by the system, and enable SysTick.
void system_init(void);

// Attempts to put the chip back in its default state, so you
// can safely jump into a bootloader.
void system_deinit(void);

// Call system_deinit() before this.
void start_bootloader_in_system_memory(void);

// Enables the independent watchdog timer.  After calling this,
// you must call iwdg_clear() every second or else the STM32 will reset.
void iwdg_init(void);

// See iwdg_init().
void iwdg_clear(void);

// Works for delays up to 89 seconds (2^32 / 48000000).
void delay_cycles(uint32_t);

static inline void delay_us(uint32_t us)
{
  delay_cycles(us * (F_CPU / 1000000));
}

static inline void delay_ms(uint32_t ms)
{
  delay_cycles(ms * (F_CPU / 1000));
}

// If you have an interrupt that sets some flags for the main loop to handle,
// be sure to set do_not_sleep to 1 in the ISR to ensure that the sleep
// functions in this library do not put the CPU to sleep before the main loop
// can see the flag, resulting in an extra millisecond of latency.
extern volatile uint8_t do_not_sleep;

// Call this once per main loop to put the CPU to sleep if appropriate.
//
// The system will wake up when any interrupt becomes pended, including the
// SysTick interrupt that happens once per millisecond.
//
// Note: This function disables interrupts temporarily in order to check
// the do_not_sleep flag.  Therefore, do NOT call it if you system has any
// interrupts enabled that must be handled with near-zero latency
// (e.g. interrupts that generate servo pulses).
void sleep_service(void);

// Puts the CPU into "Stop" mode, where the clocks are turned off, which
// is much lower power than sleep mode.  This function uses the RTC to wake
// up every 32 ms, and it also changes the USB CNTR register so that the WKUP
// interrupt will wake up the CPU.
//
// This function disables interrupts globally, so interrupt vectors are not
// executed, but any interrupt that is enabled and pending will wake up the
// CPU constantly, so you should disable those interrupts before calling this.
//
// Every time it wakes up, this function calls deep_sleep_callback(), which must
// be defined by the user.
void deep_sleep(void);

// This user-defined function is called by deep_sleep every time the CPU wakes up.
// When deep_sleep calls this function, interrupts are disabled and the chip is
// running with a slower clock than normal.
//
// The 'time' argument is the number of 32-millisecond timespans that
// have passed while sleeping, which is useful for blinking an LED.
//
// The return value should be true to keep sleeping, or false to wake up.
bool deep_sleep_callback(uint32_t time);
