// Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

#include "system.h"
#include <stm32c0xx.h>
#include <sys/stat.h>
#include <errno.h>

//#include <debug_uart.h>
//#include <stdio.h>

#define USB USB_DRD_FS

volatile uint32_t time_ms;

volatile uint8_t do_not_sleep;

#ifdef DEBUG_UART_H
#define DEBUG_WRITE(count, msg) (debug_uart_tx_write((count), (msg)))
#else
#define DEBUG_WRITE(count, msg)
#endif

void SysTick_Handler()  // ISR, priority 2
{
  time_ms++;
  do_not_sleep = 1;
}

uint32_t HAL_GetTick()
{
  return time_ms;
}

static void system_clocks_init()
{
  // Enable the HSIUSB48 48 MHz oscillator.
  RCC->CR |= RCC_CR_HSIUSB48ON;

  // Wait for HSIUSB48 to be ready.
  while (!(RCC->CR & RCC_CR_HSIUSB48RDY)) { }

  // Change the SYSCLK source to HSIUSB48.
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | 2;

  // Wait for the SYSCLK source to change to HSIUSB48.
  while ((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos != 2) { }

  // Disable HSI48 to save power.
  // This saves 0.4 mA on the STM32C0.
  RCC->CR &= ~RCC_CR_HSION;
}

static void system_clocks_deinit()
{
  // Enable HSI48.
  RCC->CR |= RCC_CR_HSION;

  // Wait for HSI48 to be ready.
  while (!(RCC->CR & RCC_CR_HSIRDY)) { }

  // Change the SYSCLK source to HSI48.
  RCC->CFGR &= ~RCC_CFGR_SW;

  // Wait for the SYSCLK source to change to HSI48.
  while ((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos != 0) { }

  // Disable HSIUSB48.
  RCC->CR &= ~RCC_CR_HSIUSB48ON;

  // Clear RCC interrupt flags.
  RCC->CICR = RCC_CICR_LSECSSC | RCC_CICR_CSSC | RCC_CICR_HSERDYC
      | RCC_CICR_HSIRDYC | RCC_CICR_HSIUSB48RDYC
      | RCC_CICR_LSERDYC | RCC_CICR_LSIRDYC;
}

void system_init()
{
  system_clocks_init();

  RCC->APBENR1 |= RCC_APBENR1_CRSEN | RCC_APBENR1_PWREN;

  // Enable all GPIO clocks.
  RCC->IOPENR = 0x2F;

  // Turn on CRS to make the HSIUSB48 clock more precise when USB is connected.
  CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

  // Set up SYSTICK to generate an interrupt every millisecond.
  NVIC_SetPriority(SysTick_IRQn, 2);
  SysTick->LOAD = F_CPU / 1000 - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
      SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

  // Keep power applied to flash memory during Sleep and Stop modes.
  // I'm not sure why, but this prevents hard faults from happening
  // immediately after waking up from a Stop mode WFI instruction.
  PWR->CR1 = 0;
}

void system_deinit()
{
  // Delay to allow the USB host to finish the control transfer
  // that requested the restart.
  delay_ms(1);

  // Disable all interrupts.
  NVIC->ICER[0]=0xFFFFFFFF;
  static_assert(sizeof(NVIC->ICER) == 4);

  // Reset all peripherals except DBG, including all I/O ports, PWR, and CRS.
  RCC->AHBRSTR = RCC_AHBRSTR_CRCRST | RCC_AHBRSTR_DMA1RST;
  RCC->APBRSTR1 = RCC_APBRSTR1_PWRRST
    | RCC_APBRSTR1_I2C2RST | RCC_APBRSTR1_I2C1RST
    | RCC_APBRSTR1_USART2RST | RCC_APBRSTR1_CRSRST
    | RCC_APBRSTR1_SPI2RST | RCC_APBRSTR1_USBRST
    | RCC_APBRSTR1_TIM3RST | RCC_APBRSTR1_TIM2RST;
  RCC->APBRSTR2 = RCC_APBRSTR2_ADCRST
    | RCC_APBRSTR2_TIM17RST | RCC_APBRSTR2_TIM16RST
    | RCC_APBRSTR2_TIM14RST | RCC_APBRSTR2_USART1RST
    | RCC_APBRSTR2_SPI1RST | RCC_APBRSTR2_TIM1RST
    | RCC_APBRSTR2_SYSCFGRST;
  RCC->IOPRSTR = RCC_IOPRSTR_GPIOFRST | RCC_IOPRSTR_GPIODRST
      | RCC_IOPRSTR_GPIOCRST | RCC_IOPRSTR_GPIOBRST | RCC_IOPRSTR_GPIOARST;
  RCC->AHBRSTR = 0;
  RCC->APBRSTR1 = 0;
  RCC->APBRSTR2 = 0;
  RCC->IOPRSTR = 0;

  // Disable all peripheral clocks.
  RCC->IOPENR = 0;
  RCC->APBENR1 = 0;
  RCC->APBENR2 = 0;

  // Delay for a while so that the USB host can detect
  // we have disconnected.
  iwdg_clear();
  delay_ms(100);

  // Reset the SysTick.
  SysTick->CTRL = 0;
  SysTick->VAL = 0;
  SysTick->LOAD = 0;

  // Clear pending bits on all interrupts.
  NVIC->ICPR[0]=0xFFFFFFFF;
  static_assert(sizeof(NVIC->ICPR) == 4);

  // Set all interrupt priorities back to 0.
  NVIC->IP[0] = 0;
  NVIC->IP[1] = 0;
  NVIC->IP[2] = 0;
  NVIC->IP[3] = 0;
  NVIC->IP[4] = 0;
  NVIC->IP[5] = 0;
  NVIC->IP[6] = 0;
  NVIC->IP[7] = 0;
  static_assert(sizeof(NVIC->IP) == 4 * 8);

  system_clocks_deinit();
}

// This code comes from:
// https://community.st.com/t5/stm32-mcus/how-to-jump/ta-p/49424
void start_bootloader_in_system_memory()
{
  volatile uint32_t * system_memory = (uint32_t *)0x1FFF0000;
  void (*func_ptr)(void) = (void (*)(void))system_memory[1];
  __set_MSP(system_memory[0]);
  func_ptr();
  while(1) { /* Unreachable */ }
}

void iwdg_init()
{
  // Enable write access to watchdog registers.
  IWDG->KR = 0x5555;

  // Wait for all pending writes to watchdog registers to finish.
  while (IWDG->SR) {};

  // Set the timeout period to 16 * 0x800 / (32 kHz) = 1.024 s.
  IWDG->PR = 2;       // 1/16 prescaler
  IWDG->RLR = 0x800;
  IWDG->KR = 0xAAAA;  // ensures the first timeout is accurate

  // Start the watchdog timer (has no effect if the WDG_SW option bit is 0).
  IWDG->KR = 0xCCCC;
}

void iwdg_clear()
{
  IWDG->KR = 0xAAAA;
}

void delay_cycles(uint32_t cycles)
{
  const uint32_t period = SysTick->LOAD + 1;
  uint32_t last_val = SysTick->VAL;
  uint32_t cycles_elapsed = 0;
  while (cycles_elapsed < cycles)
  {
    uint32_t val = SysTick->VAL;
    if (val > last_val) { last_val += period; }
    cycles_elapsed += last_val - val;
    last_val = val;
  }
}

void sleep_service()
{
  if (do_not_sleep)
  {
    do_not_sleep = 0;
    return;
  }
  __disable_irq();
  if (!do_not_sleep)
  {
    // Go to sleep until an interrupt happens.
    SCB->SCR = 0;
    __WFI();
    do_not_sleep = 0;
  }
  __enable_irq();
}

void deep_sleep()
{
#ifdef DEBUG_UART_H
  printf("%lu STOP\n", time_ms);
#endif

  assert(PWR->CR1 == 0);

  // Select deep sleep mode instead of sleep mode.
  SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;

  // Turn on the RTC's APB clock.
  RCC->APBENR1 |= RCC_APBENR1_RTCAPBEN;

  // Turn on the LSI clock
  RCC->CSR2 |= RCC_CSR2_LSION;
  while((RCC->CSR2 & RCC_CSR2_LSIRDY) == 0){}

  // Send the LSI clock signal to the RTC.
  {
    uint32_t tmp = RCC_CSR1_RTCEN | (2 << RCC_CSR1_RTCSEL_Pos);
    RCC->CSR1 = RCC_CSR1_RTCRST | tmp;
    RCC->CSR1 = tmp;
  }

  // Set up the RTC with prescalers 128 and 8, so 1 "second" is
  // 128*8/(32 kHz) = 32 ms, and an alarm interrupt fires that often.
  // This process resets both prescalers, so the first alarm happens
  // precisely from now.
  RTC->WPR = 0xCA;            // Unlock
  RTC->WPR = 0x53;            // Unlock
  RTC->ICSR = RTC_ICSR_INIT;  // Init mode
  while (!(RTC->ICSR & RTC_ICSR_INITF));
  RTC->CR = RTC_CR_BYPSHAD;  // Bypass shadow registers, turn off alarm.
  while (!(RTC->ICSR & RTC_ICSR_ALRAWF));
  RTC->PRER = 0x7F0007;       // Prescalers
  RTC->ALRMAR = 0x80808080;   // Alarm: ignore everything except subseconds
  RTC->ALRMASSR = 0x0F000007; // Alarm: trigger when subseconds == 7
  RTC->CR = RTC_CR_ALRAE | RTC_CR_ALRAIE | RTC_CR_BYPSHAD;   // Turn on alarm.
  RTC->ICSR = 0;              // exit init mode
  RTC->WPR = 0;               // lock the RTC

  // Disable the SysTick interrupt.  Should be done before __disable_irq
  // so that pending interrupts get handled.
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

#ifdef DEBUG_UART_H
  debug_uart_tx_flush();
#endif

  __disable_irq();

  // Set up the RTC so it can interrupt the CPU.
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_EnableIRQ(RTC_IRQn);

  // If the USB module is enabled and its interrupt is enabled, this
  // code makes it so the USB module's WKUP signal generates an interrupt
  // to wake up the CPU.
  uint32_t saved_usb_cntr = USB->CNTR;
  USB->CNTR = USB_CNTR_WKUPM;

  uint32_t sleep_time = 0;  // increments by 1 every 32 ms
  while (1)
  {
    __WFI();  // Actually sleep (stop mode)

    if (RTC->SR & RTC_SR_ALRAF)
    {
      RTC->SCR = RTC_SCR_CALRAF;
      NVIC_ClearPendingIRQ(RTC_IRQn);
      sleep_time++;
      time_ms += 32;
    }

    bool sleep = deep_sleep_callback(sleep_time);
    if (!sleep) { break; }
  }

  USB->CNTR = saved_usb_cntr;

  // Turn off LSI and the RTC to save about 0.2 mA on the STM32C0.
  RCC->CSR1 = RCC_CSR1_RTCRST;
  RCC->CSR2 &= ~RCC_CSR2_LSION;
  RCC->APBENR1 &= ~RCC_APBENR1_RTCAPBEN;

  NVIC_DisableIRQ(RTC_IRQn);

  system_clocks_init();

#ifdef DEBUG_UART_H
  uint32_t icsr = SCB->ICSR;
  printf("%lu DONE time=%lu ICSR=0x%lx\n", time_ms, sleep_time, icsr);
#endif
  __enable_irq();

  // Re-enable the SysTick interrupt.
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

  do_not_sleep = 1;
}

extern uint8_t _end;  // Symbol defined in the linker script
extern uint8_t _estack;          // Symbol defined in the linker script
extern uint32_t _Min_Stack_Size; // Symbol defined in the linker script
void * heap_end = &_end;

// This is called by malloc() to allocate memory.
void * _sbrk(int size)
{
  const void * limit = (void *)((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size);
  void * p = heap_end;
  if (size < 0 || size > (int)(limit - p)) { return (void *)-1; }
  heap_end += size;
  return p;
}

// Suppress a warning from the linker about _write not being implemented.
int __attribute__((weak)) _write(int file, char * ptr, int len)
{
  (void)file; (void)ptr; (void)len;
  return 0;
}

// Suppress a warning from the linker about _read not being implemented.
int __attribute__((weak)) _read(int file, char * ptr, int len)
{
  (void)file; (void)ptr; (void)len;
  return len;
}

// Suppress a warning from the linker about _lseek not being implemented.
int __attribute__((weak)) _lseek(int file, int ptr, int dir)
{
  (void)file; (void)ptr; (void)dir;
  return 0;
}

// Suppress a warning from the linker about _close not being implemented.
int __attribute__((weak)) _close(int file)
{
  (void)file;
  return -1;
}

// Suppress a warning from the linker about _fstat not being implemented.
int __attribute__((weak)) _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

// Suppress a warning from the linker about _isatty not being implemented.
int __attribute__((weak)) _isatty(int file)
{
  (void)file;
  return 1;
}

int __attribute__((weak)) _getpid(void)
{
  return 1;
}

int __attribute__((weak)) _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}
