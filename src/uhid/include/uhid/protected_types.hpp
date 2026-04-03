#pragma once
#include <functional>
#include <inputtino/input.hpp>
#include <optional>
#include <uhid/ps5.hpp>
#include <uhid/switch.hpp>
#include <uhid/uhid.hpp>

namespace inputtino {
struct SwitchJoypadState {
  std::shared_ptr<uhid::Device> dev;
  unsigned char mac_address[6] = {0x02, 0x57, 0x7E, 0x00, 0x00, 0x01};
  uint16_t vendor_id;
  uint16_t product_id;
  uhid::switch_input_report current_state = {};
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
   * This will be the MAC address of the device
   *
   * IMPORTANT: this needs to be unique for each virtual device,
   * otherwise the kernel driver will return an error:
   * "Duplicate device found for MAC address XX:XX:XX:XX"
   *
   * We also use this information internally to unique match a device with the
   * /dev/input/devXX files; see get_nodes()
   */
  unsigned char mac_address[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
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
