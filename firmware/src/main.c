// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

// USB-to-I2C adapter firmware
//
// For more info, see the user's guide: https://www.pololu.com/docs/0J89
//
// Pins when running on usb08a/usb08b:
//   PB0/GREEN_LED
//   PA7/YELLOW_LED
//   PA6/RED_LED
//   PB8/I2C1_SCL
//   PB9/I2C1_SDA
// Pins when running on Nucleo:
//   PA5/GREEN_LED
//   PA7/YELLOW_LED  (not actually connected to an LED on the board)
//   PA6/RED_LED     (not actually connected to an LED on the board)
//   PB8/I2C1_SCL
//   PB9/I2C1_SDA
// Pins when running on any board:
//   PA4/REG_EN_PIN

#include "system.h"
#include "protocol.h"
#include <tusb.h>
#include <gpio.h>
#include <stm32c0xx.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef USE_NUCLEO_64
#define LED_GREEN_PIN PA5
#define LED_YELLOW_PIN PA7
#define LED_RED_PIN PA6
#define SCL_PIN PB8
#define SDA_PIN PB9
#else
#define LED_GREEN_PIN PB0
#define LED_YELLOW_PIN PA7
#define LED_RED_PIN PA6
#define SCL_PIN PB8
#define SDA_PIN PB7
#endif

#define REG_EN_PIN PA4

#define RX_STATE_IDLE 0
#define RX_STATE_GET_ADDRESS 1
#define RX_STATE_GET_LENGTH 2
#define RX_STATE_GET_DATA 3

#define MAX_RESPONSE_SIZE (1+255)

bool board_has_output_regulator;

bool start_bootloader_soon;

volatile bool usb_activity_flag;

//// Buffers to help us read and write from the virtual serial port.
uint8_t rx_buffer[1024];
size_t rx_byte_count, rx_index;
uint8_t tx_buffer[1024];
size_t tx_byte_count, tx_index;

//// Variables to help us keep track of the command we are receiving.
uint8_t rx_state;
uint8_t command;
size_t command_data_length;
uint8_t i2c_address;
uint8_t command_data[256];
size_t command_data_received;

uint32_t i2c_timeout_base;
uint32_t i2c_timeout_ms = 50;

bool regulator_user_enable = 1;

typedef struct __attribute__((packed)) Stm32Timing
{
  uint32_t i2c_desired_timingr;
  uint8_t gpio_fmp_mode;
} Stm32Timing;

// These are the TIMINGR values that the STM32CubeIDE generates with these
// settings: rise time = 100 ns, fall time = 100 ns, digital filter = 0,
// analog filter enabled.
const Stm32Timing i2c_modes[] = {
  { 0x10805D88, 0 }, // 0 = Standard mode (100 kHz)
  { 0x0090194B, 0 }, // 1 = Fast mode (400 kHz)
  { 0x00700814, 1 }, // 2 = Fast mode plus (1000 kHz)
  { 0x9010DDFF, 0 }, // 3 = 10 kHz
};

Stm32Timing timing = i2c_modes[0];

//// Green LED variables
uint8_t usb_last_activity;
bool usb_blink_active;
uint8_t usb_blink_start;

//// Red/yellow LED variables
bool rw_error;
uint32_t rw_error_start_time;
uint32_t write_events;
uint32_t read_events;
uint32_t rw_last_time_no_events;
bool rw_blink_active;
uint32_t rw_blink_start_time;
uint32_t rw_blink_period;
uint32_t rw_blink_red_time;
uint32_t rw_blink_yellow_time;
#define RW_EVENT_COLLECTION_TIME 16
#define RW_MAX_BLINK 96

struct __attribute__((packed)) DebugData
{
  uint16_t porta;
  uint16_t portb;
  uint16_t portc;
  uint16_t portf;
  uint16_t startup_en_reading;
  uint16_t reference_factory_reading;
  uint16_t reference_reading;
} debug_data;

void USB_IRQHandler()
{
  tud_int_handler(0);
  do_not_sleep = 1;
  usb_activity_flag = 1;
}

static void adc_init()
{
  RCC->APBENR2 |= RCC_APBENR2_ADCEN;

  ADC1->CFGR1 = 0;

  // Set the ADC clock to PCLK/4 = 12 MHz, and
  // set the sample time to 79.5 cycles (6.6 us).
  ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;
  ADC1->SMPR = 6;

  // Turn on the ADC voltage regulator.
  ADC1->CR = ADC_CR_ADVREGEN;
  delay_us(25);

  // Perform calibration.
  ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL);

  // Enable the ADC.
  ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADEN;
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}

static uint16_t analog_read(uint32_t channel)
{
  // Clear CCRDY and EOC.
  ADC1->ISR = ADC_ISR_CCRDY | ADC_ISR_EOC;

  // Select the channel
  ADC1->CHSELR = 1 << channel;
  while (!(ADC1->ISR & ADC_ISR_CCRDY));

  // Start conversion
  ADC1->CR |= ADC_CR_ADSTART;

  // Wait for conversion to complete
  while (!(ADC1->ISR & ADC_ISR_EOC));

  return ADC1->DR;
}

// Set up unused pins to be pulled-down inputs so they are not floating,
// as recommended by chapter 6 of AN4899.
static void unused_pins_init()
{
  gpio_config(PA0, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PA1, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PA2, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PA3, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PA5, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PA8, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PA15, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PB1, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PB3, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PB4, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PB5, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PB6, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PC6, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PC14, GPIO_INPUT_PULLED_DOWN);
  gpio_config(PC15, GPIO_INPUT_PULLED_DOWN);
}

static void leds_init()
{
  gpio_config(LED_GREEN_PIN, GPIO_OUTPUT);
  gpio_config(LED_YELLOW_PIN, GPIO_OUTPUT);
  gpio_config(LED_RED_PIN, GPIO_OUTPUT);
}

static void led_green(bool state)
{
  gpio_write(LED_GREEN_PIN, state);
}

static void led_yellow(bool state)
{
  gpio_write(LED_YELLOW_PIN, state);
}

static void led_red(bool state)
{
  gpio_write(LED_RED_PIN, state);
}

// Update the green LED, which indicates the USB state and USB traffic.
static void led_usb_service()
{
  if (tud_mounted())
  {
    // Make the USB LED be on most of the time, but blink it for USB
    // activity.

    if (usb_activity_flag)
    {
      // Some USB activity happened recently.
      usb_activity_flag = 0;

      // Record the time that the USB activity occurred.
      usb_last_activity = time_ms;

      // If we are not already blinking to indicate USB activity,
      // start blinking.
      if (!usb_blink_active)
      {
        usb_blink_active = 1;
        usb_blink_start = time_ms;
      }
    }

    if (usb_blink_active)
    {
      // Keep blinking with a period of 128 ms as long
      // as USB requests keep coming in at least every 96 ms.

      led_green((time_ms - usb_blink_start) & 64);

      if ((uint8_t)(time_ms - usb_last_activity) > 96)
      {
        usb_blink_active = 0;
      }
    }
    else
    {
      led_green(1);
    }
  }
  else
  {
    // We have not reached the configured state.

    // 50% duty cycle blink once per second
    led_green(time_ms >> 9 & 1);

    usb_blink_active = 0;
    usb_activity_flag = 0;
  }
}

// Update the red and yellow LEDs, which indicate I2C reads and writes.
static void led_rw_service()
{
  uint32_t now = time_ms;

  if (read_events == 0 && write_events == 0)
  {
    rw_last_time_no_events = now;
  }

  if (!rw_blink_active &&
    // !(I2C1->ISR & I2C_ISR_BUSY) &&
    (read_events || write_events) &&
    (uint32_t)(now - rw_last_time_no_events) > RW_EVENT_COLLECTION_TIME)
  {
    // There are some events to show, so calculate the parameters
    // of our next blinking period.
    uint32_t red_time = write_events * RW_MAX_BLINK;
    uint32_t yellow_time = read_events * RW_MAX_BLINK;
    while (red_time > RW_MAX_BLINK || yellow_time > RW_MAX_BLINK)
    {
      red_time >>= 1;
      yellow_time >>= 1;
    }
    if (red_time < 8 && write_events) { red_time = 8; }
    if (yellow_time < 8 && read_events) { yellow_time = 8; }

    rw_blink_period = (red_time + yellow_time) + 24;
    rw_blink_red_time = red_time;
    rw_blink_yellow_time = yellow_time;
    rw_blink_start_time = now;
    rw_blink_active = 1;

    // Consume the events.
    write_events = 0;
    read_events = 0;
    rw_last_time_no_events = now;
  }

  if (rw_blink_active)
  {
    // We are currently blinking to indicate some read/write events.
    uint32_t progress = now - rw_blink_start_time;

    if (progress < rw_blink_red_time)
    {
      led_yellow(0);
      led_red(1);
    }
    else if (progress < rw_blink_red_time + rw_blink_yellow_time)
    {
      led_yellow(1);
      led_red(0);
    }
    else
    {
      led_yellow(0);
      led_red(0);
    }

    if (progress >= rw_blink_period)
    {
      rw_blink_active = 0;
    }
  }
  else
  {
    led_yellow(0);
    if (rw_error)
    {
      led_red((uint32_t)(time_ms - rw_error_start_time + 256) & 512);
    }
    else
    {
      led_red(0);
    }
  }
}

// Our desired option bytes are the defaults, except for:
// NBOOT_SEL=0 NRST_MODE=1 BORF_LEV=1 BORR_LEV=1 BOR_EN=1
static void check_option_bytes()
{
#ifndef USE_NUCLEO_64
  if (FLASH->OPTR != 0x2EEFEBAA)
  {
    while(1)
    {
      led_red(1);
      delay_ms(133);
      led_red(0);
      delay_ms(200);
      iwdg_clear();
    }
  }
#endif
}

static void determine_board()
{
  gpio_config(REG_EN_PIN, GPIO_INPUT_PULLED_UP);
  delay_us(25);

  uint16_t reading = analog_read(4);  // PA4/ADC_IN4/REG_EN
  debug_data.startup_en_reading = reading;

  // If this is a board that can supply power with an output regulator, then
  // the EN pin is pulled down with a 100k resistor on the board, and when
  // we pull it up with the STM32 we should get a reading of at most:
  // values: 105/(105+25)*4096 = 3308.
  board_has_output_regulator = reading < 3700;

  if (board_has_output_regulator)
  {
    set_usb_product_pololu_usb08b();
  }
  else
  {
    set_usb_product_pololu_usb08a();
  }
}

static void regulator_service()
{
  bool enable = tud_mounted() && regulator_user_enable;
  gpio_write(REG_EN_PIN, enable);
}

static void start_bootloader_service()
{
  if (start_bootloader_soon)
  {
    system_deinit();
    start_bootloader_in_system_memory();
  }
}

static void quick_tasks()
{
  tud_task();
  iwdg_clear();
  led_usb_service();
  led_rw_service();
}

static void i2c_init()
{
  // Enable the I2C1 clock.
  RCC->APBENR1 |= RCC_APBENR1_I2C1EN;

  gpio_set_alt_func(SCL_PIN, 6);
  gpio_set_alt_func(SDA_PIN, 6);
  gpio_config(SCL_PIN, GPIO_ALT_FUNC_OPEN_DRAIN);
  gpio_config(SDA_PIN, GPIO_ALT_FUNC_OPEN_DRAIN);

  // Disable the I2C peripheral so it is safe to change TIMINGR.
  I2C1->CR1 = 0;

  I2C1->TIMINGR = timing.i2c_desired_timingr;

  // Turn on Fast-mode plus (20 mA drive).
  if (timing.gpio_fmp_mode & 1)
  {
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C1_FMP;
  }
  else
  {
    SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_I2C1_FMP;
  }

  I2C1->CR1 = I2C_CR1_PE;
}

static void i2c_timeout_start()
{
  i2c_timeout_base = time_ms;
}

static bool i2c_timeout_check_and_quick_tasks()
{
  uint32_t elapsed = time_ms - i2c_timeout_base;
  if (elapsed >= 2)
  {
    quick_tasks();
  }
  return elapsed > i2c_timeout_ms;
}

static uint8_t i2c_check_bus()
{
  // Wait for the I2C module to be done with previous transactions.
  // (Probably not necessary, since i2c_write and i2c_read already wait for their
  // transactions to be done, and there is code to reinitialize the I2C module if
  // it is still busy after a command is executed.  But we might remove some of
  // those features later if we want to optimize them for more speed.)
  while (I2C1->ISR & I2C_ISR_BUSY)
  {
    if (i2c_timeout_check_and_quick_tasks())
    {
      return ERROR_PREVIOUS_TIMEOUT;
    }
  }

  // Clear flags that might have been set by previous transactions,
  // and which we might be checking during this transaction.
  I2C1->ICR = I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_STOPCF | I2C_ICR_NACKCF;

  // Note: page 649 of RM0490 Rev 4 suggests reading SDA and SCL as GPIO inputs
  // here and making sure they are high.
  return ERROR_NONE;
}

// This is called during an I2C data transfer to wait for a flag to be set.
static uint8_t i2c_wait_for_flag(uint32_t mask)
{
  uint32_t isr;
  while (1)
  {
    isr = I2C1->ISR;
    if (isr & mask)
    {
      break;
    }
    if (i2c_timeout_check_and_quick_tasks())
    {
      return ERROR_TIMEOUT;
    }
    if (isr & I2C_ISR_NACKF)
    {
      return ERROR_NACK;
    }
    if (isr & I2C_ISR_BERR)
    {
      return ERROR_BUS_ERROR;
    }
    if (isr & I2C_ISR_ARLO)
    {
      return ERROR_ARBITRATION_LOST;
    }
  }
  return ERROR_NONE;
}

// This is called at the end of a command to wait for the I2C module to be
// finished performing its operation, and check for any final errors.
//
// Note: Perhaps we should add an option to disable this part,
// allowing for high throughput.
static uint8_t i2c_wait_for_not_busy()
{
  uint32_t isr;
  while (1)
  {
    isr = I2C1->ISR;
    if (!(isr & I2C_ISR_BUSY))
    {
      break;
    }
    if (i2c_timeout_check_and_quick_tasks())
    {
      return ERROR_TIMEOUT;
    }
  }
  if (isr & I2C_ISR_ARLO)
  {
    return ERROR_ARBITRATION_LOST;
  }
  if (isr & I2C_ISR_BERR)
  {
    return ERROR_BUS_ERROR;
  }
  return ERROR_NONE;
}

static uint8_t i2c_write(uint8_t address, size_t count, const uint8_t data[count])
{
  if (count > 255) { return ERROR_OTHER; }

  uint8_t error = i2c_check_bus();
  if (error) { return error; }

  I2C1->CR2 = I2C_CR2_START | (address << 1) | (count << I2C_CR2_NBYTES_Pos);
  write_events += 8;

  // Write the data.
  for (size_t i = 0; i < count; i++)
  {
    error = i2c_wait_for_flag(I2C_ISR_TXIS);
    if (error)
    {
      if (error == ERROR_TIMEOUT) { return i ? ERROR_TX_TIMEOUT: ERROR_ADDRESS_TIMEOUT; }
      if (error == ERROR_NACK) { return i ? ERROR_TX_DATA_NACK : ERROR_ADDRESS_NACK; }
      return error;
    }
    read_events += 1;
    I2C1->TXDR = data[i];
    write_events += 8;
  }

  // Wait for the last byte to finish.
  error = i2c_wait_for_flag(I2C_ISR_TC);
  if (error)
  {
    if (error == ERROR_TIMEOUT) { return count ? ERROR_TX_TIMEOUT : ERROR_ADDRESS_TIMEOUT; }
    if (error == ERROR_NACK) { return count ? ERROR_TX_DATA_NACK : ERROR_ADDRESS_NACK; }
    return error;
  }
  read_events += 1;

  // Issue a STOP condition.
  I2C1->CR2 = I2C_CR2_STOP;
  assert(!(I2C1->ISR & I2C_ISR_TC));

  return i2c_wait_for_not_busy();
}

// Note: This function actually returns before the I2C module has taken care of the
// final ACK bit and the STOP condition.  That's why it's important to check the BUSY bit
// before we do any extra I/O.
static uint8_t i2c_read(uint8_t address, size_t count, uint8_t data[count])
{
  if (count == 0 || count > 255)
  {
    memset(data, 0, count);
    return ERROR_OTHER;
  }

  uint8_t error = i2c_check_bus();
  if (error)
  {
    memset(data, 0, count);
    return error;
  }

  I2C1->CR2 = I2C_CR2_START | I2C_CR2_RD_WRN |
    (address << 1) | (count << I2C_CR2_NBYTES_Pos);
  write_events += 8;

  for (size_t i = 0; i < count; i++)
  {
    error = i2c_wait_for_flag(I2C_ISR_RXNE);
    if (error)
    {
      if (error == ERROR_TIMEOUT) { return i ? ERROR_RX_TIMEOUT : ERROR_ADDRESS_TIMEOUT; }
      if (error == ERROR_NACK) { return i ? ERROR_NACK : ERROR_ADDRESS_NACK; }
      memset(data + i, 0, count - i);
      return error;
    }
    if (i == 0) { read_events++; }
    data[i] = I2C1->RXDR;
    read_events += 8;
    if (i < count - 1) { write_events++; }
  }

  I2C1->CR2 = I2C_CR2_STOP;
  assert(!(I2C1->ISR & I2C_ISR_TC));

  return i2c_wait_for_not_busy();
}

// Send pulses on SCL to help fix situations where a device is driving its SDA
// line low, as described in the "Bus Clear" section of the I2C specification.
// However, that document say sending 9 pulses is sufficient, but we actually
// need 10 because the target device could be driving SDA low to acknowledge a
// previous byte it received, and you need to send 10 pulses to get past the
// ACK bit for the next byte it is expecting to receive.
static void i2c_clear_bus()
{
  gpio_write(SCL_PIN, 1);
  gpio_config(SCL_PIN, GPIO_OUTPUT_OPEN_DRAIN);
  delay_us(50);
  for (uint8_t i = 0; i < 10; i++)
  {
    gpio_write(SCL_PIN, 0);
    delay_us(50);
    gpio_write(SCL_PIN, 1);
    delay_us(50);
    write_events++;
  }
  gpio_config(SCL_PIN, GPIO_ALT_FUNC_OPEN_DRAIN);
}

static void send_byte(uint8_t byte)
{
  assert(tx_byte_count + 1 <= sizeof(tx_buffer));
  tx_buffer[tx_byte_count++] = byte;
}

static void send_data(size_t count, uint8_t data[count])
{
  assert(tx_byte_count + count <= sizeof(tx_buffer));
  memcpy(tx_buffer + tx_byte_count, data, count);
  tx_byte_count += count;
}

static void send_zeros(size_t count)
{
  assert(tx_byte_count + count <= sizeof(tx_buffer));
  memset(tx_buffer + tx_byte_count, 0, count);
  tx_byte_count += count;
}

static void execute_write()
{
  rw_error = 0;
  i2c_timeout_start();
  uint8_t error = i2c_write(i2c_address, command_data_length, command_data);
  send_byte(error);
  rw_error = error && command_data_length;
  rw_error_start_time = time_ms;
}

static void execute_read()
{
  // Zero-length reads seem to put the bus in a bad state.
  if (command_data_length == 0)
  {
    send_byte(ERROR_PROTOCOL);
    send_zeros(command_data_length);
    return;
  }

  rw_error = 0;
  i2c_timeout_start();
  uint8_t error = i2c_read(i2c_address, command_data_length, tx_buffer + tx_byte_count + 1);
  send_byte(error);
  tx_byte_count += command_data_length;
  rw_error = (bool)error;
  rw_error_start_time = time_ms;
}

static void execute_set_i2c_mode()
{
  uint8_t mode = command_data[0];
  if (mode >= sizeof(i2c_modes) / sizeof(i2c_modes[0]))
  {
    mode = 0;
  }
  timing = i2c_modes[mode];
  i2c_init();
}

static void execute_set_i2c_timeout()
{
  i2c_timeout_ms = command_data[0] + (command_data[1] << 8);
}

static void execute_set_stm32_timing()
{
  memcpy(&timing, command_data, sizeof(timing));
  i2c_init();
}

static void execute_digital_read()
{
  uint8_t response =
      gpio_read(SCL_PIN) << 0 |
      gpio_read(SDA_PIN) << 1;
  send_byte(response);
}

static void execute_enable_vcc_out()
{
  regulator_user_enable = command_data[0] & 1;
  send_byte(board_has_output_regulator ? 0 : ERROR_NOT_SUPPORTED);
}

static void execute_get_device_info()
{
  struct __attribute__((packed)) DeviceInfo {
    uint8_t size;
    uint8_t info_version;
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t firmware_version;
    char firmware_modification[8];
    uint8_t serial_number[12];
  } info = {
    .size = sizeof(struct DeviceInfo),
    .info_version = 0,
    .vendor_id = get_usb_vendor_id(),
    .product_id = get_usb_product_id(),
    .firmware_modification = FIRMWARE_MODIFICATION_STR,
    .firmware_version = FIRMWARE_VERSION_BCD,
  };
  memcpy(info.serial_number, (void *)UID_BASE, 12);

  send_data(sizeof(info), (uint8_t *)&info);
}

static void execute_get_debug_data()
{
  ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
  debug_data.porta = GPIOA->IDR;
  debug_data.portb = GPIOB->IDR;
  debug_data.portc = GPIOC->IDR;
  debug_data.portf = GPIOF->IDR;
  debug_data.reference_factory_reading = *(uint16_t *)0x1FFF756A;
  debug_data.reference_reading = analog_read(10);  // Internal voltage reference
  send_byte(sizeof(debug_data));
  send_data(sizeof(debug_data), (void *)&debug_data);
}

static void execute_command()
{
  switch (command)
  {
  case CMD_I2C_WRITE:
    execute_write();
    break;
  case CMD_I2C_READ:
    execute_read();
    break;
  case CMD_SET_I2C_MODE:
    execute_set_i2c_mode();
    break;
  case CMD_SET_I2C_TIMEOUT:
    execute_set_i2c_timeout();
    break;
  case CMD_SET_STM32_TIMING:
    execute_set_stm32_timing();
    break;
  case CMD_ENABLE_VCC_OUT:
    execute_enable_vcc_out();
    break;
  case CMD_DIGITAL_READ:
    execute_digital_read();
    break;
  case CMD_GET_DEVICE_INFO:
    execute_get_device_info();
    break;
  case CMD_GET_DEBUG_DATA:
    execute_get_debug_data();
    break;
  }

  if (I2C1->ISR & I2C_ISR_BUSY)
  {
    // There was a timeout (or a buggy command that didn't wait for the I2C
    // transaction to finish before returning).  Reinitialize the I2C module
    // so that the data the user was trying to transfer doesn't randomly get
    // transferred later when the condition causing the timeout is fixed
    // (e.g. when SDA stops being shorted to VDD).
    i2c_init();
  }
  assert(!(I2C1->ISR & I2C_ISR_BUSY));

  quick_tasks();
}

static void prepare_to_receive_data(size_t length)
{
  command_data_length = length;
  command_data_received = 0;
  rx_state = RX_STATE_GET_DATA;
}

static void handle_rx_byte(uint8_t byte)
{
  switch (rx_state)
  {
  case RX_STATE_IDLE:
    command = byte;
    switch (byte)
    {
    case CMD_I2C_WRITE:
    case CMD_I2C_READ:
      rx_state = RX_STATE_GET_ADDRESS;
      break;
    case CMD_SET_I2C_MODE:
    case CMD_ENABLE_VCC_OUT:
      prepare_to_receive_data(1);
      break;
    case CMD_SET_I2C_TIMEOUT:
      prepare_to_receive_data(2);
      break;
    case CMD_CLEAR_BUS:
      i2c_clear_bus();
      break;
    case CMD_SET_STM32_TIMING:
      prepare_to_receive_data(sizeof(timing));
      break;
    case CMD_DIGITAL_READ:
    case CMD_GET_DEVICE_INFO:
    case CMD_GET_DEBUG_DATA:
      execute_command();
      break;
    }
    break;
  case RX_STATE_GET_ADDRESS:
    i2c_address = byte;
    rx_state = RX_STATE_GET_LENGTH;
    break;
  case RX_STATE_GET_LENGTH:
    command_data_length = byte;
    command_data_received = 0;
    if (command == CMD_I2C_WRITE && command_data_length > 0)
    {
      rx_state = RX_STATE_GET_DATA;
    }
    else
    {
      execute_command();
      rx_state = RX_STATE_IDLE;
    }
    break;
  case RX_STATE_GET_DATA:
    assert(command_data_received < command_data_length);
    assert(command_data_length <= sizeof(command_data));
    command_data[command_data_received++] = byte;
    if (command_data_received >= command_data_length)
    {
      execute_command();
      rx_state = RX_STATE_IDLE;
    }
    break;
  }
}

static void reset_serial_port_state()
{
  rx_state = RX_STATE_IDLE;
  rx_index = 0;
  rx_byte_count = 0;

  tx_index = 0;
  tx_byte_count = 0;
}

// When the USB Host sends "Send Break" request with a non-zero
// duration, reset the serial port state.  This is useful for the
// user to do after opening the port, so the next byte is guaranteed
// to be interpreted as a command.
void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms)
{
  (void)itf;
  if (duration_ms)
  {
    reset_serial_port_state();
  }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void)itf; (void)rts;
  cdc_line_coding_t coding;
  tud_cdc_n_get_line_coding(0, &coding);

  // Start the bootloader when the serial port is closed
  // while the baud rate is 600.
  if (coding.bit_rate == 600 && dtr == 0)
  {
    start_bootloader_soon = 1;
  }

  // Forget the state of the commands whenever the serial port
  // is closed.
  if (dtr == 0)
  {
    reset_serial_port_state();
  }
}

static void rx_service()
{
  if (!tud_mounted())
  {
    reset_serial_port_state();
    return;
  }

  if (rx_index >= rx_byte_count)
  {
    // There is no data left in the RX buffer, so read more.
    rx_byte_count = tud_cdc_read(rx_buffer, sizeof(rx_buffer));
    rx_index = 0;
  }
  while (rx_index < rx_byte_count && tx_byte_count <= sizeof(tx_buffer) - MAX_RESPONSE_SIZE)
  {
    uint8_t byte_received = rx_buffer[rx_index++];
    handle_rx_byte(byte_received);
  }
}

static void tx_service()
{
  if (tx_byte_count == 0) { return; }
  assert(tx_index < tx_byte_count);
  tx_index += tud_cdc_write(tx_buffer + tx_index, tx_byte_count - tx_index);
  tud_cdc_write_flush();
  if (tx_index >= tx_byte_count)
  {
    tx_index = 0;
    tx_byte_count = 0;
  }
}

// Note: We check tud_mounted to fix a problem that sometimes causes
// the device to go into deep sleep mode when enumerating shortly after power
// up and then never wake up.
static bool deep_sleep_needed()
{
  return (USB_DRD_FS->CNTR & USB_CNTR_SUSPRDY) && tud_mounted();
}

static void deep_sleep_service()
{
  if (!deep_sleep_needed()) { return; }
  gpio_write(REG_EN_PIN, 0);
  led_green(0);
  led_yellow(0);
  led_red(0);
  deep_sleep();
}

bool deep_sleep_callback(uint32_t time)
{
  iwdg_clear();
  led_green((~time & 31) == 0);
  return deep_sleep_needed();
}

int main()
{
  system_init();
  iwdg_init();
  unused_pins_init();
  leds_init();
  adc_init();
  check_option_bytes();
  determine_board();
  RCC->APBENR1 |= RCC_APBENR1_USBEN;
  tud_init(0);
  i2c_init();
  gpio_config(REG_EN_PIN, GPIO_OUTPUT);

  while (1)
  {
	tud_task();
    rx_service();
    tx_service();
    regulator_service();
    led_usb_service();
    led_rw_service();
    start_bootloader_service();
    iwdg_clear();
    deep_sleep_service();
    sleep_service();
  }
}
