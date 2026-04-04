#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <inputtino/input.hpp>
#include <uhid/protected_types.hpp>
#include <uhid/switch.hpp>
#include <uhid/uhid.hpp>

namespace inputtino {

namespace {

std::string lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string uppercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

// Real Bluetooth Pro Controller 0x30 reports on the host consistently show:
// - bat_con = 0x60
// - button_status = 00 80 00 at idle
//
// Mirror that steady-state shape as closely as possible so hid-nintendo sees
// reports that look like a real controller rather than a synthetic
// "full+charging" device with an all-zero button prefix.
constexpr uint8_t SWITCH_BATTERY_HIGH_BT = 0x60;
constexpr uint8_t SWITCH_BUTTON_STATUS1_BASE = 0x80;

// Map signed 16-bit stick axis to 12-bit Switch range [0, 4095] centered at 2048.
int scale_axis(short value) {
  auto slope = static_cast<double>(uhid::SWITCH_AXIS_MAX) / 65535.0;
  return static_cast<int>(std::round(slope * (static_cast<int>(value) + 32768)));
}

// Pack two 12-bit values into 3 bytes: X in bits 0-11, Y in bits 12-23.
void pack_stick(uint8_t out[3], uint16_t x, uint16_t y) {
  out[0] = static_cast<uint8_t>(x & 0xFF);
  out[1] = static_cast<uint8_t>(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
  out[2] = static_cast<uint8_t>((y >> 4) & 0xFF);
}

// Write a 12-bit XY pair into 3 bytes at data+offset.
void write12(uint8_t *data, int offset, uint16_t x, uint16_t y) {
  data[offset + 0] = static_cast<uint8_t>(x & 0xFF);
  data[offset + 1] = static_cast<uint8_t>(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
  data[offset + 2] = static_cast<uint8_t>((y >> 4) & 0xFF);
}

// Left stick factory calibration at 0x603D: hid-nintendo parses as
// (max_above, center, min_below).
void fill_left_stick_calibration(uint8_t *data) {
  write12(data, 0, 2047, 2047); // max_above
  write12(data, 3, 2048, 2048); // center
  write12(data, 6, 2048, 2048); // min_below
}

// Right stick factory calibration at 0x6046: hid-nintendo parses as
// (center, min_below, max_above) — different order from left stick.
void fill_right_stick_calibration(uint8_t *data) {
  write12(data, 0, 2048, 2048); // center
  write12(data, 3, 2048, 2048); // min_below
  write12(data, 6, 2047, 2047); // max_above
}

void fill_imu_calibration(uint8_t *data) {
  auto write_le16 = [&](int index, int16_t value) {
    data[index] = static_cast<uint8_t>(value & 0xFF);
    data[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  };

  for (int i = 0; i < 3; ++i) {
    write_le16(i * 2, uhid::SWITCH_IMU_ACCEL_OFFSET);
    write_le16(6 + i * 2, uhid::SWITCH_IMU_GYRO_OFFSET);
    write_le16(12 + i * 2, uhid::SWITCH_IMU_ACCEL_SCALE_CAL);
    write_le16(18 + i * 2, uhid::SWITCH_IMU_GYRO_SCALE_CAL);
  }
}

int decode_rumble_high_amplitude(uint8_t encoded) {
  // encoded ∈ [0x00, 0xC8] maps linearly to amplitude ∈ [0, 65535]
  return static_cast<int>(std::lround(encoded * (65535.0 / 0xC8)));
}

int decode_rumble_low_amplitude(uint16_t encoded) {
  // Bits 7:0 range from 0x40 to 0x72 (→ base index 0..50); bit 15 is a half-step flag.
  // Combined index ∈ [0, 100] maps linearly to amplitude ∈ [0, 65535].
  int index = (static_cast<int>(encoded & 0xFF) - 0x40) * 2 + ((encoded >> 15) & 1);
  return static_cast<int>(std::lround(std::clamp(index, 0, 100) * (65535.0 / 100)));
}

std::pair<int, int> decode_rumble_block(const uhid::switch_rumble_data &rumble) {
  auto high_amplitude = decode_rumble_high_amplitude(rumble.amp_high);
  auto low_amplitude = decode_rumble_low_amplitude(
      static_cast<uint16_t>((rumble.freq_low & 0x80) << 8) | rumble.amp_low);
  return {low_amplitude, high_amplitude};
}

void fill_reply_prefix(SwitchJoypadState &state, uhid::switch_standard_input_prefix &prefix, uint8_t report_id) {
  prefix.report_id = report_id;
  prefix.timer = state.timer++;
  prefix.bat_con = SWITCH_BATTERY_HIGH_BT;
  prefix.vibrator_input = 0;
  std::copy(std::begin(state.buttons), std::end(state.buttons), std::begin(prefix.button_status));
  pack_stick(prefix.left_stick, state.lx, state.ly);
  pack_stick(prefix.right_stick, state.rx, state.ry);
}

void send_report(SwitchJoypadState &state) {
  std::lock_guard<std::mutex> lock(state.mtx);

  if (state.report_mode != uhid::SWITCH_REPORT_MODE_STANDARD_FULL) {
    return;
  }

  uhid::switch_input_report report{};
  report.prefix.report_id = uhid::SWITCH_INPUT_REPORT_STANDARD_FULL;
  report.prefix.timer = state.timer++;
  report.prefix.bat_con = SWITCH_BATTERY_HIGH_BT;
  report.prefix.vibrator_input = 0;
  std::copy(std::begin(state.buttons), std::end(state.buttons), std::begin(report.prefix.button_status));
  pack_stick(report.prefix.left_stick, state.lx, state.ly);
  pack_stick(report.prefix.right_stick, state.rx, state.ry);
  std::copy(std::begin(state.imu), std::end(state.imu), std::begin(report.imu));

  struct uhid_event ev{};
  ev.type = UHID_INPUT2;
  std::copy(reinterpret_cast<unsigned char *>(&report),
            reinterpret_cast<unsigned char *>(&report) + sizeof(report),
            &ev.u.input2.data[0]);
  ev.u.input2.size = sizeof(report);
  state.dev->send(ev);
}

void send_subcmd_reply(
    SwitchJoypadState &state, uint8_t ack, uint8_t subcmd_id, const uint8_t *payload, size_t payload_size) {
  std::lock_guard<std::mutex> lock(state.mtx);

  uhid::switch_subcmd_reply_report report{};
  fill_reply_prefix(state, report.prefix, uhid::SWITCH_INPUT_REPORT_SUBCOMMAND_REPLY);
  report.ack = ack;
  report.subcmd_id = subcmd_id;
  if (payload && payload_size > 0) {
    std::copy(payload, payload + std::min(payload_size, sizeof(report.data)), std::begin(report.data));
  }

  struct uhid_event ev {};
  ev.type = UHID_INPUT2;
  std::copy(reinterpret_cast<unsigned char *>(&report),
            reinterpret_cast<unsigned char *>(&report) + sizeof(report),
            &ev.u.input2.data[0]);
  ev.u.input2.size = sizeof(report);
  state.dev->send(ev);
}

std::string generate_mac_string() {
  auto rand = std::bind(std::uniform_int_distribution<unsigned char>{0, 0xFF},
                        std::default_random_engine{std::random_device()()});
  std::ostringstream ss;
  for (int i = 0; i < 6; ++i) {
    if (i > 0) ss << ':';
    ss << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
       << static_cast<unsigned int>(rand());
  }
  return ss.str();
}

void handle_spi_flash_read(SwitchJoypadState &state, const uint8_t *request_data) {
  uint32_t address = static_cast<uint32_t>(request_data[0]) | (static_cast<uint32_t>(request_data[1]) << 8) |
                     (static_cast<uint32_t>(request_data[2]) << 16) | (static_cast<uint32_t>(request_data[3]) << 24);
  uint8_t size = request_data[4];

  uint8_t payload[35] = {};
  payload[0] = request_data[0];
  payload[1] = request_data[1];
  payload[2] = request_data[2];
  payload[3] = request_data[3];
  payload[4] = size;

  switch (address) {
  case uhid::JC_CAL_USR_LEFT_MAGIC_ADDR:
  case uhid::JC_CAL_USR_RIGHT_MAGIC_ADDR:
  case uhid::JC_IMU_CAL_USR_MAGIC_ADDR:
    payload[5] = 0x00;
    payload[6] = 0x00;
    break;
  case uhid::JC_CAL_FCT_DATA_LEFT_ADDR:
    fill_left_stick_calibration(&payload[5]);
    // SDL reads both sticks in one 18-byte SPI read from 0x603D.
    // The kernel reads them separately (9 bytes each from 0x603D and 0x6046).
    // Fill the right stick data at offset +9 so both paths work.
    if (size >= 18) {
      fill_right_stick_calibration(&payload[5 + 9]);
    }
    break;
  case uhid::JC_CAL_FCT_DATA_RIGHT_ADDR:
    fill_right_stick_calibration(&payload[5]);
    break;
  case uhid::JC_IMU_CAL_FCT_DATA_ADDR:
    fill_imu_calibration(&payload[5]);
    break;
  default:
    break;
  }

  send_subcmd_reply(state,
                    uhid::SWITCH_ACK_SPI_FLASH_READ,
                    uhid::SWITCH_SUBCMD_SPI_FLASH_READ,
                    payload,
                    sizeof(payload));
}

void handle_output_report(std::shared_ptr<SwitchJoypadState> state, const uint8_t *data, size_t size) {
  if (!data || size == 0) {
    return;
  }

  auto report_id = data[0];
  if (report_id == uhid::SWITCH_OUTPUT_REPORT_RUMBLE_ONLY) {
    if (size >= sizeof(uhid::switch_output_report_rumble_only) && state->on_rumble) {
      const auto *report = reinterpret_cast<const uhid::switch_output_report_rumble_only *>(data);
      auto [low_left, high_left] = decode_rumble_block(report->left);
      auto [low_right, high_right] = decode_rumble_block(report->right);
      (*state->on_rumble)(std::max(low_left, low_right), std::max(high_left, high_right));
    }
    return;
  }
  if (report_id != uhid::SWITCH_OUTPUT_REPORT_RUMBLE_AND_SUBCOMMAND) {
    return;
  }
  if (size < sizeof(uhid::switch_output_report_rumble_subcmd)) {
    return;
  }

  const auto *report = reinterpret_cast<const uhid::switch_output_report_rumble_subcmd *>(data);
  auto subcmd_id = report->subcmd_id;
  const uint8_t *subcmd_data = data + sizeof(uhid::switch_output_report_rumble_subcmd);

  switch (subcmd_id) {
  case uhid::SWITCH_SUBCMD_REQ_DEV_INFO: {
    uint8_t payload[35] = {};
    payload[0] = 0x04;
    payload[1] = 0x33;
    payload[2] = uhid::JOYCON_CTLR_TYPE_PRO;
    payload[3] = 0x02;
    std::copy(std::begin(state->mac_raw), std::end(state->mac_raw), &payload[4]);
    send_subcmd_reply(*state, uhid::SWITCH_ACK_DEV_INFO, subcmd_id, payload, sizeof(payload));
    break;
  }
  case uhid::SWITCH_SUBCMD_SET_INPUT_REPORT_MODE: {
    std::lock_guard<std::mutex> lock(state->mtx);
    state->report_mode = subcmd_data[0];
    }
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  case uhid::SWITCH_SUBCMD_ENABLE_IMU: {
    std::lock_guard<std::mutex> lock(state->mtx);
    state->imu_enabled = subcmd_data[0] != 0;
    }
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  case uhid::SWITCH_SUBCMD_ENABLE_VIBRATION: {
    std::lock_guard<std::mutex> lock(state->mtx);
    state->vibration_enabled = subcmd_data[0] != 0;
    }
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  case uhid::SWITCH_SUBCMD_SPI_FLASH_READ:
    handle_spi_flash_read(*state, subcmd_data);
    break;
  case uhid::SWITCH_SUBCMD_SET_PLAYER_LIGHTS:
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  case uhid::SWITCH_SUBCMD_GET_PLAYER_LIGHTS: {
    uint8_t payload[35] = {};
    payload[0] = 0x01;
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, payload, sizeof(payload));
    break;
  }
  case uhid::SWITCH_SUBCMD_SET_HOME_LIGHT:
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  default:
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  }

  if (state->on_rumble) {
    (*state->on_rumble)(0, 0);
  }
}

void on_uhid_event(std::shared_ptr<SwitchJoypadState> state, uhid_event ev, int fd) {
  switch (ev.type) {
  case UHID_OUTPUT:
    handle_output_report(state, ev.u.output.data, ev.u.output.size);
    break;
  case UHID_GET_REPORT: {
    uhid_event answer{};
    answer.type = UHID_GET_REPORT_REPLY;
    answer.u.get_report_reply.id = ev.u.get_report.id;
    answer.u.get_report_reply.err = 0;
    answer.u.get_report_reply.size = 0;
    auto res = uhid::uhid_write(fd, &answer);
    (void)res;
    break;
  }
  case UHID_SET_REPORT: {
    if (ev.u.set_report.rtype == UHID_OUTPUT_REPORT) {
      handle_output_report(state, ev.u.set_report.data, ev.u.set_report.size);
    }

    uhid_event answer{};
    answer.type = UHID_SET_REPORT_REPLY;
    answer.u.set_report_reply.id = ev.u.set_report.id;
    answer.u.set_report_reply.err = 0;
    auto res = uhid::uhid_write(fd, &answer);
    (void)res;
    break;
  }
  default:
    break;
  }
}

} // namespace

SwitchJoypad::SwitchJoypad(std::string mac) : _state(std::make_shared<SwitchJoypadState>()) {
  this->_state->mac = mac;
  // Parse mac string into mac_raw once; HID payload site uses mac_raw directly.
  std::stringstream ss(mac);
  for (int i = 0; i < 6; ++i) {
    unsigned int v = 0;
    ss >> std::hex >> v;
    this->_state->mac_raw[i] = static_cast<unsigned char>(v);
    if (i < 5) ss.ignore(1, ':');
  }
  // buttons[1] bit 7 mirrors a real Pro Controller's steady-state report.
  this->_state->buttons[1] = SWITCH_BUTTON_STATUS1_BASE;
  // lx/ly/rx/ry default-initialise to SWITCH_AXIS_CENTER via the struct definition.
}

SwitchJoypad::~SwitchJoypad() {
  if (this->_state && this->_state->dev) {
    this->_state->stop_repeat_thread = true;
    if (this->_send_input_thread.joinable()) {
      this->_send_input_thread.join();
    }
    this->_state->dev->stop_thread();
    this->_state->dev.reset();
  }
}

Result<SwitchJoypad> SwitchJoypad::create(const DeviceDefinition &device) {
  std::string mac = device.device_uniq.empty() ? generate_mac_string() : device.device_uniq;
  auto joypad = SwitchJoypad(mac);
  joypad._state->vendor_id = device.vendor_id;
  joypad._state->product_id = device.product_id;

  auto def = uhid::DeviceDefinition{
      .name = device.name,
      .phys = device.device_phys.empty() ? "bluetooth" : device.device_phys,
      .uniq = mac,
      .bus = BUS_BLUETOOTH,
      .vendor = static_cast<uint32_t>(device.vendor_id),
      .product = static_cast<uint32_t>(device.product_id),
      .version = static_cast<uint32_t>(device.version),
      .country = 0,
      .report_description = {&uhid::switch_rdesc_bt[0], &uhid::switch_rdesc_bt[0] + sizeof(uhid::switch_rdesc_bt)}};

  auto dev =
      uhid::Device::create(def, [state = joypad._state](uhid_event ev, int fd) { on_uhid_event(state, ev, fd); });
  if (!dev) {
    return Error(dev.getErrorMessage());
  }

  joypad._state->dev = std::make_shared<uhid::Device>(std::move(*dev));
  joypad._send_input_thread = std::thread([state = joypad._state]() {
    while (!state->stop_repeat_thread) {
      send_report(*state);
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  });
  joypad._send_input_thread.detach();

  // Keep the node-discovery path simple and PS5-like by only returning the
  // device once the kernel has exposed its input nodes in sysfs.
  for (int attempt = 0; attempt < 20; ++attempt) {
    if (!joypad.get_sys_nodes().empty()) {
      return joypad;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return joypad;
}

const std::string &SwitchJoypad::get_mac_address() const {
  return _state->mac;
}

std::vector<std::string> SwitchJoypad::get_sys_nodes() const {
  const std::string base_path = "/sys/devices/virtual/misc/uhid";
  // Normalize both sides to uppercase so discovery works regardless of whether
  // the caller supplied upper- or lowercase in device_uniq.
  const auto mac = uppercase(_state->mac);
  std::ostringstream target_id;
  target_id << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
            << static_cast<unsigned int>(_state->vendor_id);

  std::vector<std::string> nodes;
  if (!std::filesystem::exists(base_path)) {
    return nodes;
  }

  for (const auto &uhid_entry : std::filesystem::directory_iterator{base_path}) {
    if (!uhid_entry.is_directory()) {
      continue;
    }

    auto candidate = uhid_entry.path().filename().string();
    if (candidate.find(target_id.str()) == std::string::npos) {
      continue;
    }

    auto input_path = uhid_entry.path() / "input";
    if (!std::filesystem::exists(input_path)) {
      continue;
    }

    for (const auto &dev_entry : std::filesystem::directory_iterator{input_path}) {
      if (!dev_entry.is_directory()) {
        continue;
      }

      auto uniq_path = dev_entry.path() / "uniq";
      if (!std::filesystem::exists(uniq_path)) {
        continue;
      }

      std::ifstream uniq_file{uniq_path};
      std::string uniq_value;
      std::getline(uniq_file, uniq_value);
      if (uppercase(uniq_value) == mac) {
        nodes.push_back(dev_entry.path().string());
      }
    }
  }

  return nodes;
}

std::vector<std::string> SwitchJoypad::get_nodes() const {
  std::vector<std::string> nodes;
  auto sys_nodes = get_sys_nodes();
  if (sys_nodes.empty() || !std::filesystem::exists("/sys/class/input")) {
    return nodes;
  }

  std::vector<std::filesystem::path> canonical_sys_nodes;
  canonical_sys_nodes.reserve(sys_nodes.size());
  for (const auto &sys_node : sys_nodes) {
    canonical_sys_nodes.push_back(std::filesystem::weakly_canonical(sys_node));
  }

  for (const auto &entry : std::filesystem::directory_iterator{"/sys/class/input"}) {
    const auto name = entry.path().filename().string();
    if (!(name.rfind("event", 0) == 0 || name.rfind("js", 0) == 0)) {
      continue;
    }

    auto device_path = std::filesystem::weakly_canonical(entry.path() / "device");
    for (const auto &sys_node : canonical_sys_nodes) {
      if (device_path == sys_node) {
        nodes.push_back((std::filesystem::path("/dev/input") / name).string());
        break;
      }
    }
  }
  return nodes;
}

void SwitchJoypad::set_pressed_buttons(unsigned int pressed) {
  {
    std::lock_guard<std::mutex> lock(this->_state->mtx);
    auto &buttons = this->_state->buttons;
    buttons[0] = 0;
    buttons[1] = SWITCH_BUTTON_STATUS1_BASE;
    buttons[2] = 0;

    if (Y & pressed)
      buttons[0] |= 0x01;
    if (X & pressed)
      buttons[0] |= 0x02;
    if (B & pressed)
      buttons[0] |= 0x04;
    if (A & pressed)
      buttons[0] |= 0x08;
    if (RIGHT_BUTTON & pressed)
      buttons[0] |= 0x40;

    if (BACK & pressed)
      buttons[1] |= 0x01;
    if (START & pressed)
      buttons[1] |= 0x02;
    if (RIGHT_STICK & pressed)
      buttons[1] |= 0x04;
    if (LEFT_STICK & pressed)
      buttons[1] |= 0x08;
    if (HOME & pressed)
      buttons[1] |= 0x10;
    if (MISC_FLAG & pressed)
      buttons[1] |= 0x20;

    if (DPAD_DOWN & pressed)
      buttons[2] |= 0x01;
    if (DPAD_UP & pressed)
      buttons[2] |= 0x02;
    if (DPAD_RIGHT & pressed)
      buttons[2] |= 0x04;
    if (DPAD_LEFT & pressed)
      buttons[2] |= 0x08;
    if (LEFT_BUTTON & pressed)
      buttons[2] |= 0x40;
  }
  send_report(*this->_state);
}

void SwitchJoypad::set_triggers(int16_t left, int16_t right) {
  {
    std::lock_guard<std::mutex> lock(this->_state->mtx);
    auto &buttons = this->_state->buttons;
    if (left > 0) {
      buttons[2] |= 0x80;
    } else {
      buttons[2] &= ~0x80;
    }
    if (right > 0) {
      buttons[0] |= 0x80;
    } else {
      buttons[0] &= ~0x80;
    }
  }
  send_report(*this->_state);
}

void SwitchJoypad::set_stick(Joypad::STICK_POSITION stick_type, short x, short y) {
  auto scaled_x = static_cast<uint16_t>(scale_axis(x));
  auto scaled_y = static_cast<uint16_t>(scale_axis(y));
  {
    std::lock_guard<std::mutex> lock(this->_state->mtx);
    if (stick_type == LS) {
      this->_state->lx = scaled_x;
      this->_state->ly = scaled_y;
    } else {
      this->_state->rx = scaled_x;
      this->_state->ry = scaled_y;
    }
  }
  send_report(*this->_state);
}

void SwitchJoypad::set_motion(MOTION_TYPE type, float x, float y, float z) {
  int16_t sx = 0;
  int16_t sy = 0;
  int16_t sz = 0;

  if (type == ACCELERATION) {
    sx = static_cast<int16_t>(
        std::clamp(std::lround((x / uhid::SWITCH_STANDARD_GRAVITY_CONST) * uhid::SWITCH_ACCEL_SCALE),
                   static_cast<long>(INT16_MIN),
                   static_cast<long>(INT16_MAX)));
    sy = static_cast<int16_t>(
        std::clamp(std::lround((y / uhid::SWITCH_STANDARD_GRAVITY_CONST) * uhid::SWITCH_ACCEL_SCALE),
                   static_cast<long>(INT16_MIN),
                   static_cast<long>(INT16_MAX)));
    sz = static_cast<int16_t>(
        std::clamp(std::lround((z / uhid::SWITCH_STANDARD_GRAVITY_CONST) * uhid::SWITCH_ACCEL_SCALE),
                   static_cast<long>(INT16_MIN),
                   static_cast<long>(INT16_MAX)));
  } else {
    sx = static_cast<int16_t>(std::clamp(std::lround(x * uhid::SWITCH_GYRO_SCALE),
                                         static_cast<long>(INT16_MIN),
                                         static_cast<long>(INT16_MAX)));
    sy = static_cast<int16_t>(std::clamp(std::lround(y * uhid::SWITCH_GYRO_SCALE),
                                         static_cast<long>(INT16_MIN),
                                         static_cast<long>(INT16_MAX)));
    sz = static_cast<int16_t>(std::clamp(std::lround(z * uhid::SWITCH_GYRO_SCALE),
                                         static_cast<long>(INT16_MIN),
                                         static_cast<long>(INT16_MAX)));
  }

  {
    std::lock_guard<std::mutex> lock(this->_state->mtx);
    for (auto &sample : this->_state->imu) {
      if (type == ACCELERATION) {
        sample.accel[0] = sx;
        sample.accel[1] = sy;
        sample.accel[2] = sz;
      } else {
        sample.gyro[0] = sx;
        sample.gyro[1] = sy;
        sample.gyro[2] = sz;
      }
    }

    if (type == ACCELERATION && !this->_state->imu_enabled) {
      return;
    }

    if (type == GYROSCOPE && !this->_state->imu_enabled) {
      return;
    }
  }
  send_report(*this->_state);
}

void SwitchJoypad::set_on_rumble(const std::function<void(int low_freq, int high_freq)> &callback) {
  this->_state->on_rumble = callback;
}

} // namespace inputtino
