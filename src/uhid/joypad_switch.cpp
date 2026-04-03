#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
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

// Real Bluetooth Pro Controller 0x30 reports on the host consistently show:
// - bat_con = 0x60
// - button_status = 00 80 00 at idle
//
// Mirror that steady-state shape as closely as possible so hid-nintendo sees
// reports that look like a real controller rather than a synthetic
// "full+charging" device with an all-zero button prefix.
constexpr uint8_t SWITCH_BATTERY_HIGH_BT = 0x60;
constexpr uint8_t SWITCH_BUTTON_STATUS1_BASE = 0x80;
constexpr uint8_t SWITCH_OUTPUT_REPORT_RUMBLE_ONLY = 0x10;

int scale_axis(short value) {
  auto normalized = static_cast<float>(value) / 32767.0f;
  auto scaled = static_cast<int>(std::lround(uhid::SWITCH_AXIS_CENTER + normalized * (uhid::SWITCH_AXIS_SPAN / 2.0f)));
  return std::clamp(scaled, uhid::SWITCH_AXIS_MIN, uhid::SWITCH_AXIS_MAX);
}

void pack_stick(uint8_t out[3], int x, int y) {
  out[0] = static_cast<uint8_t>(x & 0xFF);
  out[1] = static_cast<uint8_t>(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
  out[2] = static_cast<uint8_t>((y >> 4) & 0xFF);
}

void fill_stick_calibration(uint8_t *data) {
  const uint16_t center_x = uhid::SWITCH_STICK_CENTER;
  const uint16_t center_y = uhid::SWITCH_STICK_CENTER;
  const uint16_t max_x = uhid::SWITCH_STICK_RANGE;
  const uint16_t max_y = uhid::SWITCH_STICK_RANGE;
  const uint16_t min_x = uhid::SWITCH_STICK_RANGE;
  const uint16_t min_y = uhid::SWITCH_STICK_RANGE;

  data[0] = max_x & 0xFF;
  data[1] = ((max_x >> 8) & 0x0F) | ((max_y & 0x0F) << 4);
  data[2] = (max_y >> 4) & 0xFF;
  data[3] = center_x & 0xFF;
  data[4] = ((center_x >> 8) & 0x0F) | ((center_y & 0x0F) << 4);
  data[5] = (center_y >> 4) & 0xFF;
  data[6] = min_x & 0xFF;
  data[7] = ((min_x >> 8) & 0x0F) | ((min_y & 0x0F) << 4);
  data[8] = (min_y >> 4) & 0xFF;
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
  static constexpr std::array<std::pair<uint16_t, uint8_t>, 101> table = {{
      {0, 0x00},     {514, 0x02},   {775, 0x04},   {921, 0x06},   {1096, 0x08},  {1303, 0x0A},  {1550, 0x0C},
      {1843, 0x0E},  {2192, 0x10},  {2606, 0x12},  {3100, 0x14},  {3686, 0x16},  {4383, 0x18},  {5213, 0x1A},
      {6199, 0x1C},  {7372, 0x1E},  {7698, 0x20},  {8039, 0x22},  {8395, 0x24},  {8767, 0x26},  {9155, 0x28},
      {9560, 0x2A},  {9984, 0x2C},  {10426, 0x2E}, {10887, 0x30}, {11369, 0x32}, {11873, 0x34}, {12398, 0x36},
      {12947, 0x38}, {13520, 0x3A}, {14119, 0x3C}, {14744, 0x3E}, {15067, 0x40}, {15397, 0x42}, {15734, 0x44},
      {16079, 0x46}, {16431, 0x48}, {16790, 0x4A}, {17158, 0x4C}, {17534, 0x4E}, {17918, 0x50}, {18310, 0x52},
      {18711, 0x54}, {19121, 0x56}, {19540, 0x58}, {19967, 0x5A}, {20405, 0x5C}, {20851, 0x5E}, {21308, 0x60},
      {21775, 0x62}, {22251, 0x64}, {22739, 0x66}, {23236, 0x68}, {23745, 0x6A}, {24265, 0x6C}, {24797, 0x6E},
      {25340, 0x70}, {25894, 0x72}, {26462, 0x74}, {27041, 0x76}, {27633, 0x78}, {28238, 0x7A}, {28856, 0x7C},
      {29488, 0x7E}, {30134, 0x80}, {30794, 0x82}, {31468, 0x84}, {32157, 0x86}, {32861, 0x88}, {33581, 0x8A},
      {34316, 0x8C}, {35068, 0x8E}, {35836, 0x90}, {36620, 0x92}, {37422, 0x94}, {38242, 0x96}, {39079, 0x98},
      {39935, 0x9A}, {40809, 0x9C}, {41703, 0x9E}, {42616, 0xA0}, {43549, 0xA2}, {44503, 0xA4}, {45477, 0xA6},
      {46473, 0xA8}, {47491, 0xAA}, {48531, 0xAC}, {49593, 0xAE}, {50679, 0xB0}, {51789, 0xB2}, {52923, 0xB4},
      {54082, 0xB6}, {55266, 0xB8}, {56476, 0xBA}, {57713, 0xBC}, {58977, 0xBE}, {60268, 0xC0}, {61588, 0xC2},
      {62936, 0xC4}, {64315, 0xC6}, {65535, 0xC8},
  }};
  for (const auto &[amplitude, code] : table) {
    if (code == encoded) {
      return amplitude;
    }
  }
  return 0;
}

int decode_rumble_low_amplitude(uint16_t encoded) {
  static constexpr std::array<std::pair<uint16_t, uint16_t>, 101> table = {{
      {0, 0x0040},     {514, 0x8040},   {775, 0x0041},   {921, 0x8041},   {1096, 0x0042},  {1303, 0x8042},
      {1550, 0x0043},  {1843, 0x8043},  {2192, 0x0044},  {2606, 0x8044},  {3100, 0x0045},  {3686, 0x8045},
      {4383, 0x0046},  {5213, 0x8046},  {6199, 0x0047},  {7372, 0x8047},  {7698, 0x0048},  {8039, 0x8048},
      {8395, 0x0049},  {8767, 0x8049},  {9155, 0x004A},  {9560, 0x804A},  {9984, 0x004B},  {10426, 0x804B},
      {10887, 0x004C}, {11369, 0x804C}, {11873, 0x004D}, {12398, 0x804D}, {12947, 0x004E}, {13520, 0x804E},
      {14119, 0x004F}, {14744, 0x804F}, {15067, 0x0050}, {15397, 0x8050}, {15734, 0x0051}, {16079, 0x8051},
      {16431, 0x0052}, {16790, 0x8052}, {17158, 0x0053}, {17534, 0x8053}, {17918, 0x0054}, {18310, 0x8054},
      {18711, 0x0055}, {19121, 0x8055}, {19540, 0x0056}, {19967, 0x8056}, {20405, 0x0057}, {20851, 0x8057},
      {21308, 0x0058}, {21775, 0x8058}, {22251, 0x0059}, {22739, 0x8059}, {23236, 0x005A}, {23745, 0x805A},
      {24265, 0x005B}, {24797, 0x805B}, {25340, 0x005C}, {25894, 0x805C}, {26462, 0x005D}, {27041, 0x805D},
      {27633, 0x005E}, {28238, 0x805E}, {28856, 0x005F}, {29488, 0x805F}, {30134, 0x0060}, {30794, 0x8060},
      {31468, 0x0061}, {32157, 0x8061}, {32861, 0x0062}, {33581, 0x8062}, {34316, 0x0063}, {35068, 0x8063},
      {35836, 0x0064}, {36620, 0x8064}, {37422, 0x0065}, {38242, 0x8065}, {39079, 0x0066}, {39935, 0x8066},
      {40809, 0x0067}, {41703, 0x8067}, {42616, 0x0068}, {43549, 0x8068}, {44503, 0x0069}, {45477, 0x8069},
      {46473, 0x006A}, {47491, 0x806A}, {48531, 0x006B}, {49593, 0x806B}, {50679, 0x006C}, {51789, 0x806C},
      {52923, 0x006D}, {54082, 0x806D}, {55266, 0x006E}, {56476, 0x806E}, {57713, 0x006F}, {58977, 0x806F},
      {60268, 0x0070}, {61588, 0x8070}, {62936, 0x0071}, {64315, 0x8071}, {65535, 0x0072},
  }};
  for (const auto &[amplitude, code] : table) {
    if (code == encoded) {
      return amplitude;
    }
  }
  return 0;
}

std::pair<int, int> decode_rumble_block(const uint8_t *data) {
  auto high_amplitude = decode_rumble_high_amplitude(data[1]);
  auto low_amplitude = decode_rumble_low_amplitude(static_cast<uint16_t>((data[2] & 0x80) << 8) | data[3]);
  return {low_amplitude, high_amplitude};
}

void fill_reply_prefix(SwitchJoypadState &state, uhid::switch_standard_input_prefix &prefix, uint8_t report_id) {
  prefix.report_id = report_id;
  prefix.timer = state.timer++;
  prefix.bat_con = SWITCH_BATTERY_HIGH_BT;
  prefix.vibrator_input = 0;
  std::copy(std::begin(state.current_state.prefix.button_status),
            std::end(state.current_state.prefix.button_status),
            std::begin(prefix.button_status));
  std::copy(std::begin(state.current_state.prefix.left_stick),
            std::end(state.current_state.prefix.left_stick),
            std::begin(prefix.left_stick));
  std::copy(std::begin(state.current_state.prefix.right_stick),
            std::end(state.current_state.prefix.right_stick),
            std::begin(prefix.right_stick));
}

void send_report(SwitchJoypadState &state) {
  if (state.report_mode != uhid::SWITCH_REPORT_MODE_STANDARD_FULL) {
    return;
  }

  state.current_state.prefix.report_id = uhid::SWITCH_INPUT_REPORT_STANDARD_FULL;
  state.current_state.prefix.timer = state.timer++;
  state.current_state.prefix.bat_con = SWITCH_BATTERY_HIGH_BT;
  state.current_state.prefix.vibrator_input = 0;

  struct uhid_event ev {};
  ev.type = UHID_INPUT2;
  std::copy(reinterpret_cast<unsigned char *>(&state.current_state),
            reinterpret_cast<unsigned char *>(&state.current_state) + sizeof(state.current_state),
            &ev.u.input2.data[0]);
  ev.u.input2.size = sizeof(state.current_state);
  state.dev->send(ev);
}

void send_subcmd_reply(
    SwitchJoypadState &state, uint8_t ack, uint8_t subcmd_id, const uint8_t *payload, size_t payload_size) {
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

std::array<unsigned char, 6> parse_mac_address(const std::string &uniq) {
  std::array<unsigned char, 6> mac_address = {};
  std::stringstream ss(uniq);
  for (int i = 0; i < 6; ++i) {
    unsigned int value = 0;
    ss >> std::hex >> value;
    mac_address[i] = static_cast<unsigned char>(value);
    if (i < 5) {
      ss.ignore(1, ':');
    }
  }
  return mac_address;
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
  case uhid::JC_CAL_FCT_DATA_RIGHT_ADDR:
    fill_stick_calibration(&payload[5]);
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
  if (report_id == SWITCH_OUTPUT_REPORT_RUMBLE_ONLY) {
    if (size >= 10 && state->on_rumble) {
      auto [low_left, high_left] = decode_rumble_block(&data[2]);
      auto [low_right, high_right] = decode_rumble_block(&data[6]);
      (*state->on_rumble)(std::max(low_left, low_right), std::max(high_left, high_right));
    }
    return;
  }
  if (report_id != uhid::SWITCH_OUTPUT_REPORT_RUMBLE_AND_SUBCOMMAND) {
    return;
  }
  if (size < 11) {
    return;
  }

  auto subcmd_id = data[10];
  const uint8_t *subcmd_data = &data[11];

  switch (subcmd_id) {
  case uhid::SWITCH_SUBCMD_REQ_DEV_INFO: {
    uint8_t payload[35] = {};
    payload[0] = 0x04;
    payload[1] = 0x33;
    payload[2] = uhid::JOYCON_CTLR_TYPE_PRO;
    payload[3] = 0x02;
    std::copy(std::begin(state->mac_address), std::end(state->mac_address), &payload[4]);
    send_subcmd_reply(*state, uhid::SWITCH_ACK_DEV_INFO, subcmd_id, payload, sizeof(payload));
    break;
  }
  case uhid::SWITCH_SUBCMD_SET_INPUT_REPORT_MODE:
    state->report_mode = subcmd_data[0];
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  case uhid::SWITCH_SUBCMD_ENABLE_IMU:
    state->imu_enabled = subcmd_data[0] != 0;
    send_subcmd_reply(*state, uhid::SWITCH_ACK, subcmd_id, nullptr, 0);
    break;
  case uhid::SWITCH_SUBCMD_ENABLE_VIBRATION:
    state->vibration_enabled = subcmd_data[0] != 0;
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

SwitchJoypad::SwitchJoypad(std::array<unsigned char, 6> mac_address) : _state(std::make_shared<SwitchJoypadState>()) {
  std::copy(mac_address.begin(), mac_address.end(), this->_state->mac_address);
  this->_state->current_state.prefix.bat_con = SWITCH_BATTERY_HIGH_BT;
  this->_state->current_state.prefix.button_status[1] = SWITCH_BUTTON_STATUS1_BASE;
  pack_stick(this->_state->current_state.prefix.left_stick, uhid::SWITCH_AXIS_CENTER, uhid::SWITCH_AXIS_CENTER);
  pack_stick(this->_state->current_state.prefix.right_stick, uhid::SWITCH_AXIS_CENTER, uhid::SWITCH_AXIS_CENTER);
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
  auto mac_address = device.device_uniq.empty() ? generate_mac_address() : parse_mac_address(device.device_uniq);
  auto joypad = SwitchJoypad(mac_address);
  joypad._state->vendor_id = device.vendor_id;
  joypad._state->product_id = device.product_id;

  auto def = uhid::DeviceDefinition{
      .name = device.name,
      .phys = device.device_phys.empty() ? "bluetooth" : device.device_phys,
      .uniq = device.device_uniq.empty() ? joypad.get_mac_address() : device.device_uniq,
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

std::string SwitchJoypad::get_mac_address() const {
  std::stringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(_state->mac_address[0]) << ":"
         << std::setw(2) << static_cast<unsigned int>(_state->mac_address[1]) << ":" << std::setw(2)
         << static_cast<unsigned int>(_state->mac_address[2]) << ":" << std::setw(2)
         << static_cast<unsigned int>(_state->mac_address[3]) << ":" << std::setw(2)
         << static_cast<unsigned int>(_state->mac_address[4]) << ":" << std::setw(2)
         << static_cast<unsigned int>(_state->mac_address[5]);
  return stream.str();
}

std::vector<std::string> SwitchJoypad::get_sys_nodes() const {
  const std::string base_path = "/sys/devices/virtual/misc/uhid";
  // get_mac_address() formats the synthetic device MAC in lowercase hex,
  // while the kernel exposes the corresponding input-node UNIQ value in
  // uppercase on this Switch path. Normalize both sides so discovery matches
  // the same controller regardless of that presentation-only case difference.
  const auto target_uniq = lowercase(get_mac_address());
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
      if (lowercase(uniq_value) == target_uniq) {
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
  auto &buttons = this->_state->current_state.prefix.button_status;
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

  send_report(*this->_state);
}

void SwitchJoypad::set_triggers(int16_t left, int16_t right) {
  auto &buttons = this->_state->current_state.prefix.button_status;
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

  send_report(*this->_state);
}

void SwitchJoypad::set_stick(Joypad::STICK_POSITION stick_type, short x, short y) {
  auto scaled_x = scale_axis(x);
  auto scaled_y = scale_axis(-y);
  if (stick_type == LS) {
    pack_stick(this->_state->current_state.prefix.left_stick, scaled_x, scaled_y);
  } else {
    pack_stick(this->_state->current_state.prefix.right_stick, scaled_x, scaled_y);
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

  for (auto &sample : this->_state->current_state.imu) {
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

  send_report(*this->_state);
}

void SwitchJoypad::set_on_rumble(const std::function<void(int low_freq, int high_freq)> &callback) {
  this->_state->on_rumble = callback;
}

} // namespace inputtino
