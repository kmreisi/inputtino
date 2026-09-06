#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <endian.h>
#include <filesystem>
#include <fstream>
#include <inputtino/input.hpp>
#include <udev_helpers.hpp>
#include <uhid/protected_types.hpp>
#include <uhid/uhid.hpp>

namespace inputtino {

static std::vector<unsigned char> build_xbox_rdesc(bool include_battery) {
  std::vector<unsigned char> descriptor(std::begin(uhid::xbox_rdesc_bt_no_battery), std::end(uhid::xbox_rdesc_bt_no_battery));
  if (include_battery) {
    descriptor.insert(descriptor.end(),
                      std::begin(uhid::xbox_battery_rdesc_fragment),
                      std::end(uhid::xbox_battery_rdesc_fragment));
  }
  descriptor.push_back(0xC0); // End Collection (Application)
  return descriptor;
}

static void send_report(XboxJoypadState &state) {
  struct uhid_event ev {};
  ev.type = UHID_INPUT2;
  std::memcpy(ev.u.input2.data, &state.current_state, sizeof(state.current_state));
  ev.u.input2.size = sizeof(state.current_state);

  std::lock_guard<std::mutex> lock(state.mtx);
  if (state.dev) {
    state.dev->send(ev);
  }
}

static void send_battery_report(XboxJoypadState &state, uhid::xbox_battery_report report) {
  struct uhid_event ev {};
  ev.type = UHID_INPUT2;
  std::memcpy(ev.u.input2.data, &report, sizeof(report));
  ev.u.input2.size = sizeof(report);

  std::lock_guard<std::mutex> lock(state.mtx);
  if (state.dev) {
    state.dev->send(ev);
  }
}

static void on_uhid_event(std::shared_ptr<XboxJoypadState> state, uhid_event ev, int fd) {
  switch (ev.type) {
  case UHID_OUTPUT: {
    if (ev.u.output.size < sizeof(uhid::xbox_output_report_rumble) ||
        ev.u.output.data[0] != uhid::XBOX_OUTPUT_REPORT_RUMBLE) {
      break;
    }
    auto *report = reinterpret_cast<const uhid::xbox_output_report_rumble *>(ev.u.output.data);

    // The descriptor's "DC Enable Actuators" nibble + 4 magnitude bytes (0-100
    // each) don't carry per-motor usage tags, so which physical motor each
    // slot drives isn't specified by the descriptor itself. We map slots
    // 0/1 to the ordinary weak/strong rumble motors and slots 2/3 to the
    // left/right trigger motors — if a real client expects the opposite
    // order the two rumble channels simply swap, buttons/axes are unaffected.
    if (report->enable & 0x3) {
      if (state->on_rumble) {
        auto weak = (report->magnitude[1] / 100.0f) * 0xFFFF;
        auto strong = (report->magnitude[0] / 100.0f) * 0xFFFF;
        (*state->on_rumble)(static_cast<int>(strong), static_cast<int>(weak));
      }
    } else if (report->enable == 0) {
      if (state->on_rumble) {
        (*state->on_rumble)(0, 0);
      }
      if (state->on_trigger_rumble) {
        (*state->on_trigger_rumble)(0, 0);
      }
    }
    if ((report->enable & 0xC) && state->on_trigger_rumble) {
      auto left = (report->magnitude[2] / 100.0f) * 0xFFFF;
      auto right = (report->magnitude[3] / 100.0f) * 0xFFFF;
      (*state->on_trigger_rumble)(static_cast<int>(left), static_cast<int>(right));
    }
    break;
  }
  default:
    break;
  }
}

XboxJoypad::XboxJoypad(uint16_t vendor_id, uint16_t product_id, const Mac &mac, bool include_share_button)
    : _state(std::make_shared<XboxJoypadState>()) {
  this->_state->mac = mac;
  this->_state->vendor_id = vendor_id;
  this->_state->product_id = product_id;
  this->_state->include_share_button = include_share_button;
}

XboxJoypad::~XboxJoypad() {
  if (this->_state) {
    if (this->_state->dev) {
      this->_state->stop_repeat_thread = true;
      if (this->_send_input_thread.joinable()) {
        this->_send_input_thread.join();
      }
      this->_state->dev->stop_thread();
      this->_state->dev.reset();
    }
  }
}

Result<XboxJoypad> XboxJoypad::create(const DeviceDefinition &device) {
  // The Series identity (0x0B13) gets the extra Share/Capture button; the
  // older Xbox One wireless identity (0x0B20) doesn't advertise it.
  bool include_share_button = device.product_id != 0x0B20;

  auto def = uhid::DeviceDefinition{
      .name = device.name,
      .phys = device.device_phys,
      .uniq = device.device_uniq,
      .bus = BUS_BLUETOOTH,
      .vendor = static_cast<uint32_t>(device.vendor_id),
      .product = static_cast<uint32_t>(device.product_id),
      .version = static_cast<uint32_t>(device.version),
      .country = 0,
      .report_description = build_xbox_rdesc(/* include_battery */ true)};

  auto mac = def.uniq.empty() ? Mac::generate() : Mac::parse(def.uniq);
  if (!mac) {
    return Error(mac.getErrorMessage());
  }
  auto joypad = XboxJoypad(device.vendor_id, device.product_id, *mac, include_share_button);
  joypad._state->has_battery = true;

  if (def.phys.empty()) {
    def.phys = "INPUTTINO_BT_LINK";
  }
  if (def.uniq.empty()) {
    def.uniq = joypad.get_mac_address();
  }

  auto dev =
      uhid::Device::create(def, [state = joypad._state](uhid_event ev, int fd) { on_uhid_event(state, ev, fd); });
  if (dev) {
    joypad._state->dev = std::make_shared<uhid::Device>(std::move(*dev));
    joypad._state->def = def;

    joypad._send_input_thread = uhid_joypad::start_report_pump<XboxJoypadState>(
        joypad._state,
        [](XboxJoypadState &s) { send_report(s); },
        std::chrono::milliseconds(10));

    uhid_joypad::wait_for_sys_nodes([&joypad]() { return joypad.get_sys_nodes(); });
    return joypad;
  }
  return Error(dev.getErrorMessage());
}

static uint16_t scale_axis(short value) {
  // signed 16-bit [-32768,32767] -> unsigned 16-bit [0,65534], matching
  // normalize_unsigned_axis()'s range in the verified source.
  return static_cast<uint16_t>(std::clamp(static_cast<int>(value) + 32768, 0, 65534));
}

static uint16_t scale_trigger(int16_t value) {
  // [0,255] -> [0,1023]
  return static_cast<uint16_t>(std::clamp((value * 1023) / 255, 0, 1023));
}

std::string XboxJoypad::get_mac_address() const {
  return _state->mac.to_string();
}

std::vector<std::string> XboxJoypad::get_sys_nodes() const {
  return uhid::find_uhid_sys_nodes(_state->vendor_id, _state->mac);
}

std::vector<std::string> XboxJoypad::get_nodes() const {
  return uhid::sys_nodes_to_dev_paths(get_sys_nodes());
}

void XboxJoypad::set_pressed_buttons(unsigned int pressed) {
  auto &report = this->_state->current_state;
  report.hat = uhid::XBOX_HAT_NEUTRAL;
  report.buttons0 = 0;
  report.buttons1 = 0;
  report.share = 0;

  if (DPAD_UP & pressed) {
    report.hat = (DPAD_LEFT & pressed) ? uhid::XBOX_HAT_NW : (DPAD_RIGHT & pressed) ? uhid::XBOX_HAT_NE : uhid::XBOX_HAT_N;
  } else if (DPAD_DOWN & pressed) {
    report.hat = (DPAD_LEFT & pressed) ? uhid::XBOX_HAT_SW : (DPAD_RIGHT & pressed) ? uhid::XBOX_HAT_SE : uhid::XBOX_HAT_S;
  } else if (DPAD_LEFT & pressed) {
    report.hat = uhid::XBOX_HAT_W;
  } else if (DPAD_RIGHT & pressed) {
    report.hat = uhid::XBOX_HAT_E;
  }

  if (A & pressed)
    report.buttons0 |= uhid::XBOX_A;
  if (B & pressed)
    report.buttons0 |= uhid::XBOX_B;
  if (X & pressed)
    report.buttons0 |= uhid::XBOX_X;
  if (Y & pressed)
    report.buttons0 |= uhid::XBOX_Y;
  if (LEFT_BUTTON & pressed)
    report.buttons0 |= uhid::XBOX_LB;
  if (RIGHT_BUTTON & pressed)
    report.buttons0 |= uhid::XBOX_RB;
  if (BACK & pressed)
    report.buttons1 |= uhid::XBOX_BACK;
  if (START & pressed)
    report.buttons1 |= uhid::XBOX_START;
  if (HOME & pressed)
    report.buttons1 |= uhid::XBOX_GUIDE;
  if (LEFT_STICK & pressed)
    report.buttons1 |= uhid::XBOX_L3;
  if (RIGHT_STICK & pressed)
    report.buttons1 |= uhid::XBOX_R3;
  if (this->_state->include_share_button && (MISC_FLAG & pressed))
    report.share = 0x01;

  send_report(*this->_state);
}

void XboxJoypad::set_triggers(int16_t left, int16_t right) {
  auto &report = this->_state->current_state;
  report.z = htole16(scale_trigger(left));
  report.rz = htole16(scale_trigger(right));
  send_report(*this->_state);
}

void XboxJoypad::set_stick(Joypad::STICK_POSITION stick_type, short x, short y) {
  auto &report = this->_state->current_state;
  switch (stick_type) {
  case RS:
    report.rx = htole16(scale_axis(x));
    report.ry = htole16(scale_axis(static_cast<short>(-y)));
    break;
  case LS:
    report.x = htole16(scale_axis(x));
    report.y = htole16(scale_axis(static_cast<short>(-y)));
    break;
  }
  send_report(*this->_state);
}

void XboxJoypad::set_on_rumble(const std::function<void(int, int)> &callback) {
  this->_state->on_rumble = callback;
}

void XboxJoypad::set_on_trigger_rumble(const std::function<void(int, int)> &callback) {
  this->_state->on_trigger_rumble = callback;
}

void XboxJoypad::set_battery(Joypad::BATTERY_STATE state, int percentage) {
  if (!this->_state->has_battery) {
    return;
  }
  auto level = uhid::XBOX_BATTERY_FULL;
  if (percentage <= 25) {
    level = uhid::XBOX_BATTERY_EMPTY;
  } else if (percentage <= 55) {
    level = uhid::XBOX_BATTERY_LOW;
  } else if (percentage <= 85) {
    level = uhid::XBOX_BATTERY_MEDIUM;
  }
  (void)state; // Xbox's BT battery report has no separate "charging" bit to set.

  uhid::xbox_battery_report report;
  report.level = uhid::XBOX_BATTERY_SOURCE_WIRELESS | level;
  send_battery_report(*this->_state, report);
}

std::vector<Joypad::UdevEvent> XboxJoypad::get_udev_events() const {
  std::vector<Joypad::UdevEvent> events;

  auto sys_nodes = this->get_sys_nodes();
  for (const auto &sys_entry : sys_nodes) {
    for (const auto &sys_node : std::filesystem::directory_iterator{sys_entry}) {
      auto fname = sys_node.path().filename().string();
      if (sys_node.is_directory() &&
          (fname.rfind("event", 0) == 0 || fname.rfind("mouse", 0) == 0 || fname.rfind("js", 0) == 0)) {
        auto sys_path = sys_node.path().string();
        sys_path.erase(0, 4);
        auto dev_path = ("/dev/input/" / sys_node.path().filename()).string();
        auto event = gen_udev_base_event(dev_path, sys_path);
        event["ID_INPUT_JOYSTICK"] = "1";
        event[".INPUT_CLASS"] = "joystick";
        event["UNIQ"] = this->get_mac_address();
        events.emplace_back(event);
      }
    }
  }

  if (!sys_nodes.empty()) {
    auto base_path = std::filesystem::path(sys_nodes[0]).parent_path().parent_path();
    if (std::filesystem::exists(base_path / "hidraw")) {
      for (const auto &hidraw_entry : std::filesystem::directory_iterator{base_path / "hidraw"}) {
        auto dev_path = "/dev/" + hidraw_entry.path().filename().string();
        auto sys_path = hidraw_entry.path().string();
        sys_path.erase(0, 4);
        auto event = gen_udev_base_event(dev_path, sys_path);
        event["SUBSYSTEM"] = "hidraw";
        events.emplace_back(event);
      }
    }
  }

  return events;
}

std::vector<Joypad::UdevHwDbEntry> XboxJoypad::get_udev_hw_db_entries() const {
  std::vector<Joypad::UdevHwDbEntry> result;

  for (const auto &sys_entry : this->get_sys_nodes()) {
    for (const auto &sys_node : std::filesystem::directory_iterator{sys_entry}) {
      auto fname = sys_node.path().filename().string();
      if (sys_node.is_directory() &&
          (fname.rfind("event", 0) == 0 || fname.rfind("js", 0) == 0 || fname.rfind("mouse", 0) == 0)) {
        auto dev_path = ("/dev/input/" / sys_node.path().filename()).string();
        Joypad::UdevHwDbEntry entry;
        entry.first = gen_udev_hw_db_filename(dev_path);
        entry.second = {"E:ID_INPUT=1",
                        "E:ID_INPUT_JOYSTICK=1",
                        "E:ID_BUS=bluetooth",
                        "G:seat",
                        "G:uaccess",
                        "Q:seat",
                        "Q:uaccess",
                        "V:1"};
        result.emplace_back(entry);
      }
    }
  }

  return result;
}

} // namespace inputtino
