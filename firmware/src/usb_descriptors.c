#include <tusb.h>
#include <stm32c071xx.h>

#define STRING_ID_MANUFACTURER    1
#define STRING_ID_PRODUCT         2
#define STRING_ID_SERIAL_NUMBER   3
#define STRING_ID_CDC_PORT        4
#define STRING_ID_FIRMWARE_MOD    5

// We don't set the Vendor ID or Product ID here:
// the set_board_* functions set those.
tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_CDC,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bcdDevice          = FIRMWARE_VERSION_BCD,
    .iManufacturer      = STRING_ID_MANUFACTURER,
    .iProduct           = STRING_ID_PRODUCT,
    .iSerialNumber      = STRING_ID_SERIAL_NUMBER,
    .bNumConfigurations = 1
};

const char * usb_strings[] = {
    [STRING_ID_FIRMWARE_MOD] = FIRMWARE_MODIFICATION_STR,
};

void set_usb_product_pololu_usb08a()
{
  desc_device.idVendor = USB_VENDOR_ID_POLOLU;
  desc_device.idProduct = USB_PRODUCT_ID_USB08A;
  usb_strings[STRING_ID_MANUFACTURER] = USB_STRING_POLOLU;
  usb_strings[STRING_ID_PRODUCT] = USB_STRING_USB08A;
  usb_strings[STRING_ID_CDC_PORT] = USB_STRING_USB08A;
}

void set_usb_product_pololu_usb08b()
{
  desc_device.idVendor = USB_VENDOR_ID_POLOLU;
  desc_device.idProduct = USB_PRODUCT_ID_USB08B;
  usb_strings[STRING_ID_MANUFACTURER] = USB_STRING_POLOLU;
  usb_strings[STRING_ID_PRODUCT] = USB_STRING_USB08B;
  usb_strings[STRING_ID_CDC_PORT] = USB_STRING_USB08B;
}

uint16_t get_usb_vendor_id(void)
{
  return desc_device.idVendor;
}

uint16_t get_usb_product_id(void)
{
  return desc_device.idProduct;
}

const uint8_t * tud_descriptor_device_cb(void)
{
  return (const uint8_t *)&desc_device;
}

#define EP_ID_CDC_NOTIF   0x81
#define EP_ID_CDC_OUT     0x02
#define EP_ID_CDC_IN      0x83

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

// USB configuration descriptor
const uint8_t desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(0, STRING_ID_CDC_PORT, EP_ID_CDC_NOTIF, 8, EP_ID_CDC_OUT, EP_ID_CDC_IN, 64),
};

const uint8_t * tud_descriptor_configuration_cb(uint8_t index)
{
  (void)index;
  return desc_fs_configuration;
}

#define USB_STRING_MAX_LENGTH 64
static uint16_t usb_string_desc[USB_STRING_MAX_LENGTH + 1];

static const char nibble_to_hex[16] = "0123456789ABCDEF";

const uint16_t * tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void)langid;
  size_t length;

  switch (index) {
  case 0:  // Language
    usb_string_desc[1] = 0x409;
    length = 1;
    break;

  case STRING_ID_SERIAL_NUMBER:
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
    if (str == NULL) { return NULL; }

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
