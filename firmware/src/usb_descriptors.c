/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <tusb.h>
#include <stm32c071xx.h>

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]         HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )

#define USB_VID   0xCafe
#define USB_BCD   0x0200
// TODO: should use real USB VID/PID here

// USB device descriptor
const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,

    // TODO: change this to be a non-composite device, right?
    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

enum {
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x83

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

// USB configuration descriptor
const uint8_t desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// String Descriptor Index
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

// array of pointer to string descriptors
static char const * usb_strings[] = {
    NULL,                          // 0: Language is hardcoded
    "TinyUSB",                     // 1: Manufacturer
    "TinyUSB Device",              // 2: Product
    NULL,                          // 3: Serial number is handled specially
    "TinyUSB CDC",                 // 4: CDC Interface
};

#define USB_STRING_MAX_LENGTH 64
static uint16_t usb_string_desc[USB_STRING_MAX_LENGTH + 1];

size_t board_get_unique_id(uint8_t id[], size_t max_len)
{
  (void)max_len;
  volatile uint32_t * stm32_uuid = (volatile uint32_t *)UID_BASE;
  uint32_t * id32 = (uint32_t *)(uintptr_t)id;
  id32[0] = stm32_uuid[0];
  id32[1] = stm32_uuid[1];
  id32[2] = stm32_uuid[2];
  return 12;
}

static const char nibble_to_hex[16] = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

const uint16_t * tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  size_t length;

  switch (index) {
  case STRID_LANGID:
    usb_string_desc[1] = 0x409;
    length = 1;
    break;

  case STRID_SERIAL:
    for (size_t offset = 0; offset < 6; offset++)
    {
      uint16_t v = *(volatile uint16_t *)(UID_BASE + offset * 2);
      usb_string_desc[1 + offset * 5 + 0] = nibble_to_hex[v >> 4 & 0xF];
      usb_string_desc[1 + offset * 5 + 1] = nibble_to_hex[v >> 0 & 0xF];
      usb_string_desc[1 + offset * 5 + 2] = nibble_to_hex[v >> 12 & 0xF];
      usb_string_desc[1 + offset * 5 + 3] = nibble_to_hex[v >> 8 & 0xF];
      usb_string_desc[1 + offset * 5 + 4] = '-';
    }
    length = 29;
    break;

  default:
    if (index >= sizeof(usb_strings) / sizeof(usb_strings[0])) { return NULL; }

    const char * str = usb_strings[index];

    length = strlen(str);
    if (length > USB_STRING_MAX_LENGTH){ length = USB_STRING_MAX_LENGTH; }

    // Generate UTF-16 string
    for (size_t i = 0; i < length; i++)
    {
      usb_string_desc[1 + i] = str[i];
    }
    break;
  }

  usb_string_desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * length + 2));
  return usb_string_desc;
}
