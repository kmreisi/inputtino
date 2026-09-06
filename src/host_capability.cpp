// Runtime probes for host-level input capabilities plus the runtime joypad
// factory, used by callers that want to pick between uhid- and uinput-backed
// virtual devices based on what the kernel actually exposes here.

#include <fcntl.h>
#include <inputtino/input.hpp>
#include <iostream>
#include <memory>
#include <unistd.h>

namespace inputtino {

bool is_uhid_supported() {
  // Cached once per process — UHID node presence doesn't change at
  // runtime on any sane host, and probing on every controller
  // creation would add ~200us of open/close per join.
  static const bool supported = []() {
    int fd = open("/dev/uhid", O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      return false;
    }
    close(fd);
    return true;
  }();
  return supported;
}

namespace {

// Wrap a concrete Result<T> into a Result<unique_ptr<Joypad>>, moving the
// created pad onto the heap and up-casting to the Joypad base.
template <typename T> Result<std::unique_ptr<Joypad>> as_joypad(Result<T> created) {
  if (!created) {
    return Error(created.getErrorMessage());
  }
  return std::unique_ptr<Joypad>(std::make_unique<T>(std::move(*created)));
}

} // namespace

DeviceDefinition Joypad::default_definition(Joypad::TYPE kind) {
  switch (kind) {
  case Joypad::TYPE::XBOX:
    return {.name = "Wolf X-Box One (virtual) pad", .vendor_id = 0x045E, .product_id = 0x02EA, .version = 0x0408};
  case Joypad::TYPE::PS:
    return {.name = "Wolf DualSense (virtual) pad", .vendor_id = 0x054C, .product_id = 0x0CE6, .version = 0x8111};
  case Joypad::TYPE::JOYCON_LEFT:
    // hid-nintendo keys the Joy-Con side off the product id (0x2006 = left).
    return {.name = "Wolf Joy-Con (L) (virtual) pad", .vendor_id = 0x057e, .product_id = 0x2006, .version = 0x8111};
  case Joypad::TYPE::JOYCON_RIGHT:
    return {.name = "Wolf Joy-Con (R) (virtual) pad", .vendor_id = 0x057e, .product_id = 0x2007, .version = 0x8111};
  case Joypad::TYPE::NINTENDO:
    return {.name = "Wolf Nintendo (virtual) pad", .vendor_id = 0x057e, .product_id = 0x2009, .version = 0x8111};
  case Joypad::TYPE::PS4:
    return {.name = "Wolf DualShock 4 (virtual) pad", .vendor_id = 0x054C, .product_id = 0x05C4, .version = 0x0100};
  case Joypad::TYPE::GENERIC:
    return {.name = "Wolf Generic (virtual) pad", .vendor_id = 0x1209, .product_id = 0x0001, .version = 0x0100};
  }
  return {};
}

Result<std::unique_ptr<Joypad>> Joypad::create(Joypad::TYPE kind, bool prefer_uhid) {
  return create(kind, default_definition(kind), prefer_uhid);
}

Result<std::unique_ptr<Joypad>> Joypad::create(Joypad::TYPE kind, const DeviceDefinition &device, bool prefer_uhid) {
  switch (kind) {
  case Joypad::TYPE::XBOX:
#ifdef INPUTTINO_USE_UHID
    if (prefer_uhid) {
      // The rich uhid pad is a real Xbox Wireless Controller identity
      // (0x045E:0x0B13, Series w/ Share button), distinct from the uinput
      // fallback's older xpad-style USB id — the factory owns that identity
      // rather than trusting the caller's vendor/product here, same as
      // PS5Joypad/SwitchJoypad's uhid vs uinput ids never diverging by chance.
      auto uhid_device = device;
      uhid_device.vendor_id = 0x045E;
      uhid_device.product_id = 0x0B13;
      uhid_device.version = 0x0513;
      if (auto pad = XboxJoypad::create(uhid_device)) {
        return std::unique_ptr<Joypad>(std::make_unique<XboxJoypad>(std::move(*pad)));
      } else {
        std::cerr << "inputtino: uhid Xbox joypad creation failed (" << pad.getErrorMessage()
                  << "), falling back to the uinput backend" << std::endl;
      }
    }
#else
    (void)prefer_uhid;
#endif
    return as_joypad(XboxOneJoypad::create(device));
  case Joypad::TYPE::PS:
#ifdef INPUTTINO_USE_UHID
    if (prefer_uhid) {
      if (auto pad = PS5Joypad::create(device)) {
        return std::unique_ptr<Joypad>(std::make_unique<PS5Joypad>(std::move(*pad)));
      } else {
        std::cerr << "inputtino: uhid PS5 joypad creation failed (" << pad.getErrorMessage()
                  << "), falling back to the uinput backend" << std::endl;
      }
    }
#else
    (void)prefer_uhid;
#endif
    return as_joypad(PS5JoypadUinput::create(device));
  // Pro Controller and the two Joy-Cons are the same hid-nintendo pad; the
  // SwitchJoypad picks Pro / L / R from device.product_id (see default_definition).
  case Joypad::TYPE::JOYCON_LEFT:
  case Joypad::TYPE::JOYCON_RIGHT:
  case Joypad::TYPE::NINTENDO:
#ifdef INPUTTINO_USE_UHID
    if (prefer_uhid) {
      if (auto pad = SwitchJoypad::create(device)) {
        return std::unique_ptr<Joypad>(std::make_unique<SwitchJoypad>(std::move(*pad)));
      } else {
        std::cerr << "inputtino: uhid Switch joypad creation failed (" << pad.getErrorMessage()
                  << "), falling back to the uinput backend" << std::endl;
      }
    }
#else
    (void)prefer_uhid;
#endif
    return as_joypad(SwitchJoypadUinput::create(device));
  case Joypad::TYPE::PS4:
#ifdef INPUTTINO_USE_UHID
    if (prefer_uhid) {
      if (auto pad = DS4Joypad::create(device)) {
        return std::unique_ptr<Joypad>(std::make_unique<DS4Joypad>(std::move(*pad)));
      } else {
        std::cerr << "inputtino: uhid DS4 joypad creation failed (" << pad.getErrorMessage()
                  << "), falling back to the uinput backend" << std::endl;
      }
    }
#else
    (void)prefer_uhid;
#endif
    // No dedicated uinput DS4 backend; PS5JoypadUinput's generic PS-family
    // layout (BTN_TR2/TL2 for L2/R2 as buttons) is already DS4-shaped.
    return as_joypad(PS5JoypadUinput::create(device));
  case Joypad::TYPE::GENERIC:
    return as_joypad(GenericJoypad::create(device));
  }
  return Error("Joypad::create: unknown Joypad::TYPE");
}

} // namespace inputtino
