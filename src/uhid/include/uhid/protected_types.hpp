#pragma once
#include <functional>
#include <inputtino/input.hpp>
#include <mutex>
#include <optional>
#include <uhid/ps5.hpp>
#include <uhid/switch.hpp>
#include <uhid/uhid.hpp>

namespace inputtino {
struct SwitchJoypadState {
  std::shared_ptr<uhid::Device> dev;

  /**
   * The uhid definition used to create dev. Kept so recreate_device() can tear the device down and build an
   * identical one (same MAC/uniq, so get_sys_nodes() still matches) without losing the joypad's other state.
   */
  uhid::DeviceDefinition def = {};

  Mac mac;
  uint16_t vendor_id;
  uint16_t product_id;
  uint8_t controller_type = uhid::JOYCON_CTLR_TYPE_PRO;

  std::mutex mtx;

  uint8_t buttons[3] = {};
  uint16_t lx = uhid::SWITCH_AXIS_CENTER;
  uint16_t ly = uhid::SWITCH_AXIS_CENTER;
  uint16_t rx = uhid::SWITCH_AXIS_CENTER;
  uint16_t ry = uhid::SWITCH_AXIS_CENTER;
  uhid::switch_imu_sample imu[3] = {};

  uint8_t timer = 0;
  uint8_t report_mode = uhid::SWITCH_REPORT_MODE_STANDARD_FULL;
  bool imu_enabled = false;
  bool vibration_enabled = false;
  std::optional<std::function<void(int, int)>> on_rumble = std::nullopt;
  bool stop_repeat_thread = false;
};

struct PS5JoypadState {
  std::shared_ptr<uhid::Device> dev;

  /**
   * The uhid definition used to create dev. Kept so recreate_device() can tear the device down and build an
   * identical one (same MAC/uniq, so get_sys_nodes() still matches) without losing the joypad's other state.
   */
  uhid::DeviceDefinition def = {};

  // MAC allocated from the pooled Mac type (switch-pro-controller work);
  // supersedes the legacy mac_address[6] array. dev_paths derive from
  // this so recreate_device() rebuilds with the same /dev/input/* layout.
  Mac mac;
  uint16_t vendor_id;

  uhid::dualsense_input_report current_state = {};
  uint8_t last_touch_id = 0;

  std::optional<std::function<void(int, int)>> on_rumble = std::nullopt;
  std::optional<std::function<void(int, int, int)>> on_led = std::nullopt;
  std::optional<std::function<void(const PS5Joypad::TriggerEffect &)>> on_trigger_effect = std::nullopt;
  uint32_t last_left_trigger_event = 0;
  uint32_t last_right_trigger_event = 0;

  bool stop_repeat_thread = false;
  bool is_bluetooth = true;
};
} // namespace inputtino
