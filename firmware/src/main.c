// USB-to-I2C adapter firmware
//
//  I2C Write command format:
//    Command:
//      Byte 0:                'W' (0x57)
//      Byte 1:                7-bit I2C address
//      Byte 2:                length (0 to 255)
//      Bytes 3 to (2+length): data to write
//      Byte 3+length:         '.' (0x2E) to mark the end of the command
//    Response:
//      Byte 0:                error code (0 for success)
//    You can use a zero-length write command to check for the presence of a
//    device.
//
//  I2C Read command format:
//    Command:
//      Byte 0:                'R' (0x52)
//      Byte 1:                7-bit I2C address
//      Byte 2:                length (1 to 255)
//      Byte 3:                '.' (0x2E) to mark the end of the command
//    Response:
//      Byte 0:                error code (0 for success)
//      Bytes 1 to length:     received data (even if there was an error)
//
// The "bit number" in these responses is the number of times the adapter
// successfully waited for SCL to go high before the error occurred.
// It waits for SCL to go high once during a start condition, once during
// a stop condition, and 9 times for each byte written or read from the
// target.  For a NACK error, the bit number is 1 + 9 * N for some N > 0.
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
//   PA0/A0/DEBUG0         - debug output 0
//   PA2/USART2_TX/CN10_18 - serial debug output
//   PA4/REG_EN_PIN
//   PA14/SWCLK/BOOT0

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
#define BOOT0_PIN PA14

#ifdef DEBUG
#define DEBUG0 PA0
#endif

#define RX_STATE_IDLE 0
#define RX_STATE_GET_ADDRESS 1
#define RX_STATE_GET_LENGTH 2
#define RX_STATE_GET_DATA 3

#define MAX_RESPONSE_SIZE (1+255)

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
uint32_t i2c_timeout_ms = 20;

bool regulator_user_enable = 1;

typedef struct __attribute__((packed)) Stm32Timing
{
  uint32_t i2c_desired_timingr;
  uint8_t gpio_fmp_mode;
} Stm32Timing;

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


void USB_IRQHandler()
{
  tud_int_handler(0);
  do_not_sleep = 1;
  usb_activity_flag = 1;
}

// stm32c071xx.s from cmsis_device_c0 uses a different ISR name.
void USB_DRD_FS_IRQHandler() __attribute__((alias("USB_IRQHandler")));

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
  else if (rw_error)
  {
    led_yellow(0);
    led_red((uint32_t)(time_ms - rw_error_start_time + 256) & 512);
  }
  else
  {
    led_yellow(0);
    led_red(0);
  }
}

static void check_option_bits()
{
  // Some early usb08a/b devices were programmed with NRST_MODE=11
  // (the default) instead of NRST_MODE=01 which is a better choice
  // in general, so we just mask off NRST_MODE[1].
  if ((FLASH->OPTR & ~0x10000000) != 0x2EEFFEAA)
  {
    while(1)
    {
      led_red(1);
      delay_ms(100);
      led_red(0);
      delay_ms(200);
    }
  }
}

static void regulator_service()
{
  bool enable = tud_mounted() && regulator_user_enable;
  gpio_write(REG_EN_PIN, enable);
}

// We call gpio_get_mode to check if the BOOT0 pin is actually being
// used as an input, because we might be using it as SWCLK to debug
// the system.
static void start_bootloader_service()
{
  static uint16_t last_time_boot0_low = 0;
  if (gpio_get_mode(BOOT0_PIN) != GPIO_INPUT || !gpio_read(BOOT0_PIN))
  {
    last_time_boot0_low = time_ms;
  }
  else if ((uint16_t)(time_ms - last_time_boot0_low) > 20)
  {
    start_bootloader_soon = 1;
  }

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
  while ((I2C1->ISR & mask) == 0)
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
// finish performing its operation, and check for any final errors.
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
  uint8_t error = i2c_check_bus();
  if (error) { return error; }

  if (count == 0 || count > 255) { return ERROR_OTHER; }

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

static void execute_set_regulator()
{
  regulator_user_enable = command_data[0] & 1;
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
      .vendor_id = USB_VENDOR_ID,
      .product_id = USB_PRODUCT_ID,
      .firmware_modification = FIRMWARE_MODIFICATION_STR,
      .firmware_version = FIRMWARE_VERSION_BCD,
  };
  memcpy(info.serial_number, (void *)UID_BASE, 12);

  send_data(sizeof(info), (uint8_t *)&info);
}

static void execute_command()
{
  switch (command)
  {
  case CMD_WRITE:
    execute_write();
    break;
  case CMD_READ:
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
  case CMD_SET_REGULATOR:
    execute_set_regulator();
    break;
  case CMD_DIGITAL_READ:
    execute_digital_read();
    break;
  case CMD_GET_DEVICE_INFO:
    execute_get_device_info();
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
    case CMD_WRITE:
    case CMD_READ:
      rx_state = RX_STATE_GET_ADDRESS;
      break;
    case CMD_SET_I2C_MODE:
    case CMD_SET_REGULATOR:
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
    if (command == CMD_WRITE && command_data_length > 0)
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
  if (coding.bit_rate == 600 && dtr == 0)
  {
    start_bootloader_soon = 1;
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
#ifdef DEBUG
  gpio_config(DEBUG0, GPIO_OUTPUT);
  gpio_write(DEBUG0, 0);
#else
  gpio_config(BOOT0_PIN, GPIO_INPUT_PULLED_DOWN);
#endif
  leds_init();
  check_option_bits();
  RCC->APBENR1 |= RCC_APBENR1_USBEN;
  tud_init(0);
  i2c_init();

  gpio_config(REG_EN_PIN, GPIO_OUTPUT);
  gpio_write(REG_EN_PIN, 1); // tmphax

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
