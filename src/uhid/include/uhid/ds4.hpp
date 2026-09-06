#pragma once

#include <crc32.hpp>
#include <linux/uhid.h>
#include <uhid/ps5.hpp> // reuses HAT_STATES, dualsense_touch_point and the PS_*_CRC32 seeds (identical on DS4)
#include <vector>

namespace uhid {

/*
 * DS4 hardware limits. The touchpad is natively 1920x942 (shorter than
 * DualSense's 1920x1080), everything else (sticks/triggers/buttons/hat) is
 * the standard 8-bit HID gamepad range.
 */
static constexpr int DS4_TOUCHPAD_WIDTH = 1920;
static constexpr int DS4_TOUCHPAD_HEIGHT = 942;
static constexpr int DS4_AXIS_MIN = 0;
static constexpr int DS4_AXIS_MAX = 0xFF;
static constexpr int DS4_AXIS_NEUTRAL = 0x80;

/*
 * Report descriptor, feature-report payloads and report/offset layout below
 * are ported from LizardByte/libvirtualhid (MIT), which in turn documents
 * them as "following the Linux hid-playstation DS4 layout" — the same public
 * protocol inputtino's own DualSense support (uhid/ps5.hpp) is derived from,
 * which is why the two share field-for-field structure (touch point packing,
 * CRC32 seeds, hat-switch encoding).
 */
static constexpr unsigned char ds4_rdesc_bt[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x11,       //   Report ID (17 / 0x11 — DS4's Bluetooth input report id)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x20,       //   Usage (0x20)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x05, 0x01,       //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x32,       //   Usage (Z)
    0x09, 0x35,       //   Usage (Rz)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x04,       //   Report Count (4)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x09, 0x39,       //   Usage (Hat switch)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x07,       //   Logical Maximum (7)
    0x35, 0x00,       //   Physical Minimum (0)
    0x46, 0x3B, 0x01, //   Physical Maximum (315)
    0x65, 0x14,       //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,       //   Report Size (4)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x42,       //   Input (Data,Var,Abs,Null State)
    0x65, 0x00,       //   Unit (None)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (0x01)
    0x29, 0x0E,       //   Usage Maximum (0x0E) — 14 buttons
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x0E,       //   Report Count (14)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x21,       //   Usage (0x21)
    0x75, 0x06,       //   Report Size (6)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x05, 0x01,       //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x33,       //   Usage (Rx)
    0x09, 0x34,       //   Usage (Ry)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x22,       //   Usage (0x22)
    0x95, 0x42,       //   Report Count (66)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x85, 0x11,       //   Report ID (17)
    0x09, 0x23,       //   Usage (0x23)
    0x95, 0x4D,       //   Report Count (77)
    0x91, 0x02,       //   Output (Data,Var,Abs)
    0x85, 0x05,       //   Report ID (5)
    0x09, 0x24,       //   Usage (0x24)
    0x95, 0x28,       //   Report Count (40)
    0xB1, 0x02,       //   Feature (Data,Var,Abs)
    0x85, 0x12,       //   Report ID (18 / 0x12)
    0x09, 0x25,       //   Usage (0x25)
    0x95, 0x0F,       //   Report Count (15)
    0xB1, 0x02,       //   Feature (Data,Var,Abs)
    0x85, 0xA3,       //   Report ID (163 / 0xA3)
    0x09, 0x26,       //   Usage (0x26)
    0x95, 0x30,       //   Report Count (48)
    0xB1, 0x02,       //   Feature (Data,Var,Abs)
    0xC0,             // End Collection
};

enum DS4_REPORT_TYPES : unsigned int {
  DS4_CALIBRATION_USB = 0x02,
  DS4_CALIBRATION_BT = 0x05,
  DS4_PAIRING_INFO = 0x12,
  DS4_FIRMWARE_INFO = 0xA3
};

// Feature-report payloads, verbatim from LizardByte/libvirtualhid (MIT).
static constexpr unsigned char ds4_calibration_info_bt[] = {
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0xF0, 0xD8, 0x10, 0x27,
    0xF0, 0xD8, 0x10, 0x27, 0xF0, 0xD8, 0xF4, 0x01, 0xF4, 0x01, 0x10, 0x27, 0xF0,
    0xD8, 0x10, 0x27, 0xF0, 0xD8, 0x10, 0x27, 0xF0, 0xD8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,
};

static constexpr unsigned char ds4_firmware_info[] = {
    0xA3, 0x41, 0x75, 0x67, 0x20, 0x20, 0x33, 0x20, 0x32, 0x30, 0x31, 0x33, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x30, 0x37, 0x3A, 0x30, 0x31, 0x3A, 0x31, 0x32, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x31, 0x03, 0x00,
    0x00, 0x00, 0x49, 0x00, 0x05, 0x00, 0x00, 0x80, 0x03, 0x00,
};

static constexpr unsigned char ds4_pairing_info[] = {
    0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/*
 * Button masks for the DS4 input report's first two bitfield bytes
 * (offset payload+4 and payload+5 below). Bits 0-3 of the first byte are the
 * hat switch (reuses HAT_STATES from ps5.hpp).
 */
enum DS4_BUTTONS0 : uint8_t {
  DS4_SQUARE = 0x10,
  DS4_CROSS = 0x20,
  DS4_CIRCLE = 0x40,
  DS4_TRIANGLE = 0x80
};

enum DS4_BUTTONS1 : uint8_t {
  DS4_L1 = 0x01,
  DS4_R1 = 0x02,
  DS4_L2 = 0x04,
  DS4_R2 = 0x08,
  DS4_SHARE = 0x10,
  DS4_OPTIONS = 0x20,
  DS4_L3 = 0x40,
  DS4_R3 = 0x80
};

enum DS4_BUTTONS2 : uint8_t {
  DS4_PS_HOME = 0x01,
  DS4_TOUCHPAD = 0x02
};

/*
 * Bluetooth input report, matching lvh::reports::pack_dualshock4_input_report:
 * report_id(1) + hid_present(1) + reserved(1) [payload_offset=3] + the 33-byte
 * gamepad payload + 1-byte touch packet counter + 2 touch contacts (4 bytes
 * each) + padding up to 78 bytes total, with the last 4 bytes overwritten by
 * a CRC32 (same seed/algorithm as DualSense, see uhid::PS_INPUT_CRC32).
 */
#pragma pack(push, 1)
struct ds4_input_report_bt_header {
  uint8_t report_id = 0x11;
  uint8_t hid_state = 0x80; // "HID data present" flag; DS4 always sets this.
  uint8_t reserved = 0;
};

struct ds4_input_report_usb_header {
  uint8_t report_id = 0x01;
};

constexpr size_t DS4_INPUT_REPORT_BT_SIZE = 78;
constexpr size_t DS4_INPUT_REPORT_USB_SIZE = 64;

struct ds4_input_report_payload {
  uint8_t x, y = DS4_AXIS_NEUTRAL;   // LS
  uint8_t rx, ry = DS4_AXIS_NEUTRAL; // RS (Z/Rz in the descriptor)
  // buttons[0] bits 0-3: hat (HAT_NEUTRAL when centred); bits 4-7: DS4_BUTTONS0
  uint8_t buttons[3] = {HAT_NEUTRAL, 0, 0};
  uint8_t l2_trigger = 0;
  uint8_t r2_trigger = 0;
  __le16 sensor_timestamp = 0;
  uint8_t battery_level = 0xFF;
  __le16 gyro[3] = {0, 0, 0};  /* x, y, z */
  __le16 accel[3] = {0, 0, 0}; /* x, y, z */
  uint8_t reserved[5] = {};
  uint8_t battery_status = 0x1B; // matches dualshock4_battery_status(full)
  uint8_t reserved2[2] = {};
  uint8_t touch_data_format = 0x01; // constant marker byte preceding the touch frame
};
static_assert(sizeof(ds4_input_report_payload) == 33, "DS4 payload must match pack_dualshock4_input_report's layout");

struct ds4_touch_report {
  uint8_t reserved = 0; // touch_report_offset itself; unused in this single-frame emulation
  dualsense_touch_point points[2] = {};
};

// header(3) + payload(33) + touch(9) + padding(29) + crc32(4) == 78
struct ds4_input_report_bt {
  ds4_input_report_bt_header header = {};
  ds4_input_report_payload payload = {};
  ds4_touch_report touch = {};
  uint8_t padding[29] = {};
  __le32 crc32 = 0;
};
static_assert(sizeof(ds4_input_report_bt) == DS4_INPUT_REPORT_BT_SIZE, "DS4 BT report must be exactly 78 bytes");

// header(1) + payload(33) + touch(9) + padding(21) == 64
struct ds4_input_report_usb {
  ds4_input_report_usb_header header = {};
  ds4_input_report_payload payload = {};
  ds4_touch_report touch = {};
  uint8_t padding[21] = {};
};
static_assert(sizeof(ds4_input_report_usb) == DS4_INPUT_REPORT_USB_SIZE, "DS4 USB report must be exactly 64 bytes");

/*
 * Output report (rumble + lightbar), matching lvh::reports::append_dualshock4_outputs.
 * valid_flag0 bit0 = rumble present, bit1 = lightbar present.
 */
enum DS4_OUTPUT_FLAG0 : uint8_t {
  DS4_FLAG0_RUMBLE = 0x01,
  DS4_FLAG0_LIGHTBAR = 0x02
};

static constexpr uint8_t DS4_OUTPUT_REPORT_USB = 0x05;
struct ds4_output_report_common {
  uint8_t valid_flag0;
  uint8_t valid_flag1;
  uint8_t reserved;
  uint8_t motor_right; // weak
  uint8_t motor_left;  // strong
  uint8_t lightbar_red;
  uint8_t lightbar_green;
  uint8_t lightbar_blue;
};

struct ds4_output_report_usb {
  uint8_t report_id; /* 0x05 */
  ds4_output_report_common common;
  uint8_t reserved[23];
};

static constexpr uint8_t DS4_OUTPUT_REPORT_BT = 0x11;
struct ds4_output_report_bt {
  uint8_t report_id; /* 0x11 */
  uint8_t control;   // bit6 (0x40) = CRC32 present
  uint8_t reserved;
  ds4_output_report_common common;
  uint8_t reserved2[65];
  __le32 crc32;
};
#pragma pack(pop)

} // namespace uhid
