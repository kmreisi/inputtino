#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <crc32.hpp>
#include <cstring>
#include <endian.h>
#include <filesystem>
#include <fstream>
#include <inputtino/input.hpp>
#include <udev_helpers.hpp>
#include <uhid/protected_types.hpp>
#include <uhid/uhid.hpp>

namespace inputtino {

static uint32_t sign_crc32(uint32_t seed, const unsigned char *buffer, size_t length) {
  auto crc = CRC32(buffer, length, seed);
  return htole32(crc);
}

static void send_report(DS4JoypadState &state) {
  {
    // Same 0.33us-derived unit as pack_dualshock4_input_report's sensor timestamp.
    auto now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
                   .count();
    state.current_state.payload.sensor_timestamp = htole16(static_cast<uint16_t>((static_cast<uint64_t>(now) * 3) / 16));
  }

  auto &report = state.current_state;
  auto crc_len = sizeof(report) - sizeof(report.crc32);
  report.crc32 = sign_crc32(uhid::PS_INPUT_CRC32, reinterpret_cast<const unsigned char *>(&report), crc_len);

  struct uhid_event ev {};
  ev.type = UHID_INPUT2;
  std::memcpy(ev.u.input2.data, &report, sizeof(report));
  ev.u.input2.size = sizeof(report);

  // Serialise access to state.dev between the control thread and the report-pump thread.
  std::lock_guard<std::mutex> lock(state.mtx);
  if (state.dev) {
    state.dev->send(ev);
  }
}

static void on_uhid_event(std::shared_ptr<DS4JoypadState> state, uhid_event ev, int fd) {
  switch (ev.type) {
  case UHID_GET_REPORT: {
    uhid_event answer{};
    answer.type = UHID_GET_REPORT_REPLY;
    answer.u.get_report_reply.id = ev.u.get_report.id;
    answer.u.get_report_reply.err = 0;
    switch (ev.u.get_report.rnum) {
    case uhid::DS4_CALIBRATION_BT: {
      std::copy(std::begin(uhid::ds4_calibration_info_bt),
                std::end(uhid::ds4_calibration_info_bt),
                &answer.u.get_report_reply.data[0]);
      answer.u.get_report_reply.size = sizeof(uhid::ds4_calibration_info_bt);
      break;
    }
    case uhid::DS4_PAIRING_INFO: {
      std::copy(std::begin(uhid::ds4_pairing_info), std::end(uhid::ds4_pairing_info), &answer.u.get_report_reply.data[0]);
      std::reverse_copy(state->mac.bytes.begin(), state->mac.bytes.end(), &answer.u.get_report_reply.data[1]);
      answer.u.get_report_reply.size = sizeof(uhid::ds4_pairing_info);
      break;
    }
    case uhid::DS4_FIRMWARE_INFO: {
      std::copy(std::begin(uhid::ds4_firmware_info), std::end(uhid::ds4_firmware_info), &answer.u.get_report_reply.data[0]);
      answer.u.get_report_reply.size = sizeof(uhid::ds4_firmware_info);
      break;
    }
    default:
      answer.u.get_report_reply.err = -EINVAL;
      break;
    }

    if (answer.u.get_report_reply.err == 0) {
      // CRC32-sign the feature reply, same convention as PS5Joypad's Bluetooth feature reports.
      auto end_of_msg = answer.u.get_report_reply.size - 4;
      auto crc = sign_crc32(uhid::PS_FEATURE_CRC32, &answer.u.get_report_reply.data[0], end_of_msg);
      std::copy(reinterpret_cast<unsigned char *>(&crc),
                reinterpret_cast<unsigned char *>(&crc) + 4,
                &answer.u.get_report_reply.data[end_of_msg]);
    }

    auto res = uhid::uhid_write(fd, &answer);
    (void)res;
    break;
  }
  case UHID_OUTPUT: {
    if (ev.u.output.size < sizeof(uhid::ds4_output_report_bt)) {
      break;
    }
    auto *report = reinterpret_cast<const uhid::ds4_output_report_bt *>(ev.u.output.data);
    const auto &common = report->common;

    if (common.valid_flag0 & uhid::DS4_FLAG0_RUMBLE) {
      if (state->on_rumble) {
        auto left = (common.motor_left / 255.0f) * 0xFFFF;
        auto right = (common.motor_right / 255.0f) * 0xFFFF;
        (*state->on_rumble)(static_cast<int>(left), static_cast<int>(right));
      }
    } else if (common.valid_flag0 == 0 && common.valid_flag1 == 0) {
      if (state->on_rumble) {
        (*state->on_rumble)(0, 0);
      }
    }

    if (common.valid_flag0 & uhid::DS4_FLAG0_LIGHTBAR) {
      if (state->on_led) {
        (*state->on_led)(common.lightbar_red, common.lightbar_green, common.lightbar_blue);
      }
    }
    break;
  }
  default:
    break;
  }
}

DS4Joypad::DS4Joypad(uint16_t vendor_id, const Mac &mac) : _state(std::make_shared<DS4JoypadState>()) {
  this->_state->mac = mac;
  this->_state->vendor_id = vendor_id;
  this->_state->current_state.touch.points[0].contact = 1;
  this->_state->current_state.touch.points[1].contact = 1;
}

DS4Joypad::~DS4Joypad() {
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

Result<DS4Joypad> DS4Joypad::create(const DeviceDefinition &device) {
  auto def = uhid::DeviceDefinition{
      .name = device.name,
      .phys = device.device_phys,
      .uniq = device.device_uniq,
      .bus = BUS_BLUETOOTH,
      .vendor = static_cast<uint32_t>(device.vendor_id),
      .product = static_cast<uint32_t>(device.product_id),
      .version = static_cast<uint32_t>(device.version),
      .country = 0,
      .report_description = {&uhid::ds4_rdesc_bt[0], &uhid::ds4_rdesc_bt[0] + sizeof(uhid::ds4_rdesc_bt)}};

  auto mac = def.uniq.empty() ? Mac::generate() : Mac::parse(def.uniq);
  if (!mac) {
    return Error(mac.getErrorMessage());
  }
  auto joypad = DS4Joypad(device.vendor_id, *mac);

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

    joypad._send_input_thread = uhid_joypad::start_report_pump<DS4JoypadState>(
        joypad._state,
        [](DS4JoypadState &s) { send_report(s); },
        std::chrono::milliseconds(10));

    uhid_joypad::wait_for_sys_nodes([&joypad]() { return joypad.get_sys_nodes(); });
    return joypad;
  }
  return Error(dev.getErrorMessage());
}

static int scale_value(int input, int input_start, int input_end, int output_start, int output_end) {
  auto slope = 1.0 * (output_end - output_start) / (input_end - input_start);
  return output_start + std::round(slope * (input - input_start));
}

std::string DS4Joypad::get_mac_address() const {
  return _state->mac.to_string();
}

std::vector<std::string> DS4Joypad::get_sys_nodes() const {
  return uhid::find_uhid_sys_nodes(_state->vendor_id, _state->mac);
}

std::vector<std::string> DS4Joypad::get_nodes() const {
  return uhid::sys_nodes_to_dev_paths(get_sys_nodes());
}

void DS4Joypad::set_pressed_buttons(unsigned int pressed) {
  auto &payload = this->_state->current_state.payload;
  {
    payload.buttons[0] = uhid::HAT_NEUTRAL;
    payload.buttons[1] &= (uhid::DS4_L2 | uhid::DS4_R2);
    payload.buttons[2] = 0;
  }
  {
    if (DPAD_UP & pressed) {
      if (DPAD_LEFT & pressed) {
        payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_NW;
      } else if (DPAD_RIGHT & pressed) {
        payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_NE;
      } else {
        payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_N;
      }
    } else if (DPAD_DOWN & pressed) {
      if (DPAD_LEFT & pressed) {
        payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_SW;
      } else if (DPAD_RIGHT & pressed) {
        payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_SE;
      } else {
        payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_S;
      }
    } else if (DPAD_LEFT & pressed) {
      payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_W;
    } else if (DPAD_RIGHT & pressed) {
      payload.buttons[0] = (payload.buttons[0] & 0xF0) | uhid::HAT_E;
    }

    if (X & pressed)
      payload.buttons[0] |= uhid::DS4_SQUARE;
    if (Y & pressed)
      payload.buttons[0] |= uhid::DS4_TRIANGLE;
    if (A & pressed)
      payload.buttons[0] |= uhid::DS4_CROSS;
    if (B & pressed)
      payload.buttons[0] |= uhid::DS4_CIRCLE;
    if (LEFT_BUTTON & pressed)
      payload.buttons[1] |= uhid::DS4_L1;
    if (RIGHT_BUTTON & pressed)
      payload.buttons[1] |= uhid::DS4_R1;
    if (LEFT_STICK & pressed)
      payload.buttons[1] |= uhid::DS4_L3;
    if (RIGHT_STICK & pressed)
      payload.buttons[1] |= uhid::DS4_R3;
    if (START & pressed)
      payload.buttons[1] |= uhid::DS4_OPTIONS;
    if (BACK & pressed)
      payload.buttons[1] |= uhid::DS4_SHARE;
    if (TOUCHPAD_FLAG & pressed)
      payload.buttons[2] |= uhid::DS4_TOUCHPAD;
    if (HOME & pressed)
      payload.buttons[2] |= uhid::DS4_PS_HOME;
  }
  send_report(*this->_state);
}

void DS4Joypad::set_triggers(int16_t left, int16_t right) {
  auto &payload = this->_state->current_state.payload;
  payload.l2_trigger = scale_value(left, 0, 255, uhid::DS4_AXIS_MIN, uhid::DS4_AXIS_MAX);
  payload.r2_trigger = scale_value(right, 0, 255, uhid::DS4_AXIS_MIN, uhid::DS4_AXIS_MAX);

  if (left == 0)
    payload.buttons[1] &= ~uhid::DS4_L2;
  else
    payload.buttons[1] |= uhid::DS4_L2;

  if (right == 0)
    payload.buttons[1] &= ~uhid::DS4_R2;
  else
    payload.buttons[1] |= uhid::DS4_R2;

  send_report(*this->_state);
}

void DS4Joypad::set_stick(Joypad::STICK_POSITION stick_type, short x, short y) {
  auto &payload = this->_state->current_state.payload;
  switch (stick_type) {
  case RS:
    payload.rx = scale_value(x, -32768, 32767, uhid::DS4_AXIS_MIN, uhid::DS4_AXIS_MAX);
    payload.ry = scale_value(-y, -32768, 32767, uhid::DS4_AXIS_MIN, uhid::DS4_AXIS_MAX);
    break;
  case LS:
    payload.x = scale_value(x, -32768, 32767, uhid::DS4_AXIS_MIN, uhid::DS4_AXIS_MAX);
    payload.y = scale_value(-y, -32768, 32767, uhid::DS4_AXIS_MIN, uhid::DS4_AXIS_MAX);
    break;
  }
  send_report(*this->_state);
}

void DS4Joypad::set_on_rumble(const std::function<void(int, int)> &callback) {
  this->_state->on_rumble = callback;
}

void DS4Joypad::set_on_led(const std::function<void(int, int, int)> &callback) {
  this->_state->on_led = callback;
}

void DS4Joypad::place_finger(int finger_nr, uint16_t x, uint16_t y) {
  if (finger_nr > 1) {
    return;
  }
  auto &points = this->_state->current_state.touch.points;
  // DS4's native touchpad is 1920x942 (shorter than the 1920x1080 convention
  // shared by Joypad::touchpad_width/height), so rescale y proportionally.
  auto native_y = static_cast<uint16_t>(std::lround(y * (uhid::DS4_TOUCHPAD_HEIGHT - 1) / double(Joypad::touchpad_height - 1)));

  if (points[finger_nr].contact == 1) {
    points[finger_nr].id = ++this->_state->last_touch_id;
  }
  points[finger_nr].contact = 0;
  points[finger_nr].x_lo = static_cast<uint8_t>(x & 0x00FF);
  points[finger_nr].x_hi = static_cast<uint8_t>((x & 0x0F00) >> 8);
  points[finger_nr].y_lo = static_cast<uint8_t>(native_y & 0x000F);
  points[finger_nr].y_hi = static_cast<uint8_t>((native_y & 0x0FF0) >> 4);

  send_report(*this->_state);
}

void DS4Joypad::release_finger(int finger_nr) {
  if (finger_nr > 1) {
    return;
  }
  if (this->_state->last_touch_id >= 0x7E) {
    this->_state->last_touch_id = 0;
  }
  this->_state->current_state.touch.points[finger_nr].contact = 1;
  send_report(*this->_state);
}

std::vector<Joypad::UdevEvent> DS4Joypad::get_udev_events() const {
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

        std::ifstream name_file(std::filesystem::path(sys_entry) / "name");
        std::string name;
        std::getline(name_file, name);
        if (name.find("Touchpad") != std::string::npos) {
          event["ID_INPUT_TOUCHPAD"] = "1";
          event[".INPUT_CLASS"] = "mouse";
          event["ID_INPUT_TOUCHPAD_INTEGRATION"] = "internal";
        } else {
          event["ID_INPUT_JOYSTICK"] = "1";
          event[".INPUT_CLASS"] = "joystick";
          event["UNIQ"] = this->get_mac_address();
        }
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

std::vector<Joypad::UdevHwDbEntry> DS4Joypad::get_udev_hw_db_entries() const {
  std::vector<Joypad::UdevHwDbEntry> result;

  for (const auto &sys_entry : this->get_sys_nodes()) {
    for (const auto &sys_node : std::filesystem::directory_iterator{sys_entry}) {
      auto fname = sys_node.path().filename().string();
      if (sys_node.is_directory() &&
          (fname.rfind("event", 0) == 0 || fname.rfind("js", 0) == 0 || fname.rfind("mouse", 0) == 0)) {
        auto dev_path = ("/dev/input/" / sys_node.path().filename()).string();
        Joypad::UdevHwDbEntry entry;
        entry.first = gen_udev_hw_db_filename(dev_path);

        std::ifstream name_file(std::filesystem::path(sys_entry) / "name");
        std::string name;
        std::getline(name_file, name);
        if (name.find("Touchpad") != std::string::npos) {
          entry.second = {"E:ID_INPUT=1",
                          "E:ID_INPUT_TOUCHPAD=1",
                          "E:ID_BUS=usb",
                          "G:seat",
                          "G:uaccess",
                          "Q:seat",
                          "Q:uaccess",
                          "V:1"};
        } else {
          entry.second = {"E:ID_INPUT=1",
                          "E:ID_INPUT_JOYSTICK=1",
                          "E:ID_BUS=usb",
                          "G:seat",
                          "G:uaccess",
                          "Q:seat",
                          "Q:uaccess",
                          "V:1"};
        }
        result.emplace_back(entry);
      }
    }
  }

  return result;
}

} // namespace inputtino
