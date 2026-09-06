#pragma once

#include <linux/uhid.h>
#include <uhid/ps5.hpp> // reuses HAT_STATES (0-indexed N..NW, matching the GIP hat convention this shifts by one)
#include <vector>

namespace uhid {

/*
 * Report descriptor and report layout below are ported from
 * LizardByte/libvirtualhid (MIT): the descriptor is its
 * make_xbox_bluetooth_report_descriptor(), and the input-report field layout
 * (button bits, hat shift, trigger packing) is taken from its
 * make_xbox_bluetooth_input_report()/pack_xbox_gip_input_report() — this is
 * the standard Xbox Wireless Controller Bluetooth HID report Linux's
 * hid-generic/xpad already parse, not something libvirtualhid invented.
 * The rumble (SET_EFFECT) output report is decoded directly from this same
 * descriptor's own field layout (enable-actuators nibble + 4 magnitude
 * bytes); which physical motor each of the 4 magnitude bytes drives is a
 * documented-but-unverified-here assumption (see joypad_xbox.cpp) — getting
 * that order backwards only swaps which motor buzzes, it can't misreport
 * buttons/axes.
 */
static constexpr unsigned char xbox_rdesc_bt_no_battery[] = {
    0x05, 0x01,             // Usage Page (Generic Desktop)
    0x09, 0x05,             // Usage (Game Pad)
    0xA1, 0x01,             // Collection (Application)
    0x85, 0x01,             //   Report ID (1)
    0x09, 0x01,             //   Usage (Pointer)
    0xA1, 0x00,             //   Collection (Physical)
    0x09, 0x30,             //     Usage (X)
    0x09, 0x31,             //     Usage (Y)
    0x15, 0x00,             //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
    0x95, 0x02,             //     Report Count (2)
    0x75, 0x10,             //     Report Size (16)
    0x81, 0x02,             //     Input (Data,Var,Abs)
    0xC0,                   //   End Collection
    0x09, 0x01,             //   Usage (Pointer)
    0xA1, 0x00,             //   Collection (Physical)
    0x09, 0x33,             //     Usage (Rx)
    0x09, 0x34,             //     Usage (Ry)
    0x15, 0x00,             //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
    0x95, 0x02,             //     Report Count (2)
    0x75, 0x10,             //     Report Size (16)
    0x81, 0x02,             //     Input (Data,Var,Abs)
    0xC0,                   //   End Collection
    0x05, 0x01,             //   Usage Page (Generic Desktop)
    0x09, 0x32,             //   Usage (Z) — left trigger
    0x15, 0x00,             //   Logical Minimum (0)
    0x26, 0xFF, 0x03,       //   Logical Maximum (1023)
    0x95, 0x01,             //   Report Count (1)
    0x75, 0x0A,             //   Report Size (10)
    0x81, 0x02,             //   Input (Data,Var,Abs)
    0x15, 0x00, 0x25, 0x00, //   Logical Min/Max (0,0) — padding
    0x75, 0x06, 0x95, 0x01, //   Report Size (6) Count (1)
    0x81, 0x03,             //   Input (Const,Var,Abs)
    0x05, 0x01,             //   Usage Page (Generic Desktop)
    0x09, 0x35,             //   Usage (Rz) — right trigger
    0x15, 0x00,             //   Logical Minimum (0)
    0x26, 0xFF, 0x03,       //   Logical Maximum (1023)
    0x95, 0x01,             //   Report Count (1)
    0x75, 0x0A,             //   Report Size (10)
    0x81, 0x02,             //   Input (Data,Var,Abs)
    0x15, 0x00, 0x25, 0x00, //   Logical Min/Max (0,0) — padding
    0x75, 0x06, 0x95, 0x01, //   Report Size (6) Count (1)
    0x81, 0x03,             //   Input (Const,Var,Abs)
    0x05, 0x01,             //   Usage Page (Generic Desktop)
    0x09, 0x39,             //   Usage (Hat switch)
    0x15, 0x01,             //   Logical Minimum (1)
    0x25, 0x08,             //   Logical Maximum (8)
    0x35, 0x00,             //   Physical Minimum (0)
    0x46, 0x3B, 0x01,       //   Physical Maximum (315)
    0x66, 0x14, 0x00,       //   Unit (Degrees)
    0x75, 0x04, 0x95, 0x01, //   Report Size (4) Count (1)
    0x81, 0x42,             //   Input (Data,Var,Abs,Null State)
    0x75, 0x04, 0x95, 0x01, //   Report Size (4) Count (1) — padding
    0x15, 0x00, 0x25, 0x00, 0x35, 0x00, 0x45, 0x00, 0x65, 0x00,
    0x81, 0x03,             //   Input (Const,Var,Abs)
    0x05, 0x09,             //   Usage Page (Button)
    0x19, 0x01, 0x29, 0x0F, //   Usage Minimum/Maximum (Button 1-15)
    0x15, 0x00, 0x25, 0x01, //   Logical Minimum/Maximum (0,1)
    0x75, 0x01, 0x95, 0x0F, //   Report Size (1) Count (15)
    0x81, 0x02,             //   Input (Data,Var,Abs)
    0x15, 0x00, 0x25, 0x00, //   Logical Min/Max (0,0) — 1-bit pad to byte boundary
    0x75, 0x01, 0x95, 0x01,
    0x81, 0x03,             //   Input (Const,Var,Abs)
    0x05, 0x0C,             //   Usage Page (Consumer)
    0x0A, 0xB2, 0x00,       //   Usage (Record) — Series' Share/Capture button
    0x15, 0x00, 0x25, 0x01, //   Logical Minimum/Maximum (0,1)
    0x95, 0x01, 0x75, 0x01, //   Report Count (1) Size (1)
    0x81, 0x02,             //   Input (Data,Var,Abs)
    0x15, 0x00, 0x25, 0x00, //   Logical Min/Max (0,0) — 7-bit pad to byte boundary
    0x75, 0x07, 0x95, 0x01,
    0x81, 0x03,             //   Input (Const,Var,Abs)
    0x05, 0x0F,             //   Usage Page (Physical Interface Device)
    0x09, 0x21,             //   Usage (Set Effect Report)
    0x85, 0x03,             //   Report ID (3)
    0xA1, 0x02,             //   Collection (Logical)
    0x09, 0x97,             //     Usage (DC Enable Actuators)
    0x15, 0x00, 0x25, 0x01, //     Logical Minimum/Maximum (0,1)
    0x75, 0x04, 0x95, 0x01, //     Report Size (4) Count (1)
    0x91, 0x02,             //     Output (Data,Var,Abs)
    0x15, 0x00, 0x25, 0x00, //     Logical Min/Max (0,0) — pad
    0x75, 0x04, 0x95, 0x01,
    0x91, 0x03,             //     Output (Const,Var,Abs)
    0x09, 0x70,             //     Usage (Magnitude)
    0x15, 0x00, 0x25, 0x64, //     Logical Minimum/Maximum (0,100)
    0x75, 0x08, 0x95, 0x04, //     Report Size (8) Count (4)
    0x91, 0x02,             //     Output (Data,Var,Abs)
    0x09, 0x50,             //     Usage (Duration)
    0x66, 0x01, 0x10,       //     Unit (Seconds)
    0x55, 0x0E,             //     Unit Exponent (-2)
    0x15, 0x00, 0x26, 0xFF, 0x00, //     Logical Minimum/Maximum (0,255)
    0x75, 0x08, 0x95, 0x01, //     Report Size (8) Count (1)
    0x91, 0x02,             //     Output (Data,Var,Abs)
    0x09, 0xA7,             //     Usage (Start Delay)
    0x15, 0x00, 0x26, 0xFF, 0x00, //     Logical Minimum/Maximum (0,255)
    0x75, 0x08, 0x95, 0x01,
    0x91, 0x02,             //     Output (Data,Var,Abs)
    0x65, 0x00, 0x55, 0x00, //     Unit (None), Unit Exponent (0)
    0x09, 0x7C,             //     Usage (Loop Count)
    0x15, 0x00, 0x26, 0xFF, 0x00, //     Logical Minimum/Maximum (0,255)
    0x75, 0x08, 0x95, 0x01,
    0x91, 0x02,             //     Output (Data,Var,Abs)
    0xC0,                   //   End Collection
};

static constexpr unsigned char xbox_battery_rdesc_fragment[] = {
    0x05, 0x06,       // Usage Page (Generic Device Controls)
    0x09, 0x20,       // Usage (Battery Strength)
    0x85, 0x04,       // Report ID (4)
    0x15, 0x04,       // Logical Minimum (wireless, empty)
    0x25, 0x07,       // Logical Maximum (wireless, full)
    0x75, 0x08,       // Report Size (8)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x02,       // Input (Data,Var,Abs)
};

enum XBOX_HAT : uint8_t {
  XBOX_HAT_NEUTRAL = 0,
  XBOX_HAT_N = 1,
  XBOX_HAT_NE = 2,
  XBOX_HAT_E = 3,
  XBOX_HAT_SE = 4,
  XBOX_HAT_S = 5,
  XBOX_HAT_SW = 6,
  XBOX_HAT_W = 7,
  XBOX_HAT_NW = 8
};

enum XBOX_BUTTONS0 : uint8_t {
  XBOX_A = 0x01,
  XBOX_B = 0x02,
  XBOX_X = 0x08,
  XBOX_Y = 0x10,
  XBOX_LB = 0x40,
  XBOX_RB = 0x80
};

enum XBOX_BUTTONS1 : uint8_t {
  XBOX_BACK = 0x04,
  XBOX_START = 0x08,
  XBOX_GUIDE = 0x10,
  XBOX_L3 = 0x20,
  XBOX_R3 = 0x40
};

#pragma pack(push, 1)
struct xbox_input_report {
  uint8_t report_id = 0x01;
  uint16_t x = 0x7FFF, y = 0x7FFF;   // LS, u16 LE (centre ~32767)
  uint16_t rx = 0x7FFF, ry = 0x7FFF; // RS
  uint16_t z = 0;                    // left trigger, 0-1023 (upper 6 bits unused)
  uint16_t rz = 0;                   // right trigger, 0-1023
  uint8_t hat = XBOX_HAT_NEUTRAL;
  uint8_t buttons0 = 0;
  uint8_t buttons1 = 0;
  uint8_t share = 0; // Series-only Record/Share button
};
static_assert(sizeof(xbox_input_report) == 17, "must match xbox_bluetooth_input_report_size");

static constexpr uint8_t XBOX_OUTPUT_REPORT_RUMBLE = 0x03;
struct xbox_output_report_rumble {
  uint8_t report_id; /* 0x03 */
  uint8_t enable;     // bits 0-3: which of the 4 magnitude slots are active
  uint8_t magnitude[4]; // 0-100 each; slot order — see joypad_xbox.cpp
  uint8_t duration;
  uint8_t start_delay;
  uint8_t loop_count;
};
static_assert(sizeof(xbox_output_report_rumble) == 9, "must match xbox_bluetooth_rumble_report_size");

static constexpr uint8_t XBOX_INPUT_REPORT_BATTERY = 0x04;
struct xbox_battery_report {
  uint8_t report_id = 0x04;
  uint8_t level = 0x07; // 0x04 | (0-3); 0x07 == full
};
#pragma pack(pop)

enum XBOX_BATTERY_LEVEL : uint8_t {
  XBOX_BATTERY_SOURCE_WIRELESS = 0x04,
  XBOX_BATTERY_EMPTY = 0,
  XBOX_BATTERY_LOW = 1,
  XBOX_BATTERY_MEDIUM = 2,
  XBOX_BATTERY_FULL = 3
};

} // namespace uhid
