#include <catch2/catch_test_macros.hpp>
#include <inputtino/input.hpp>

using namespace inputtino;

template <typename T> static T unwrap_or_skip(Result<T> created, const char *what) {
  if (!created) {
    SKIP(std::string("Skipping because ") + what + " could not be created on this host: " + created.getErrorMessage());
  }
  return std::move(*created);
}

// These cover the runtime backend selection, the basic uinput joypad backends
// and the self-describing udev metadata. They deliberately avoid the SDL/HIDAPI
// round-trip (see testUHID/testJoypads) so they stay deterministic.

TEST_CASE("is_uhid_supported is stable across calls", "[Factory]") {
  const bool supported = is_uhid_supported();
  REQUIRE(is_uhid_supported() == supported);
}

TEST_CASE("uinput DualSense pad: no rich features, safe no-ops, self-describes", "[Factory]") {
  auto joypad = unwrap_or_skip(PS5JoypadUinput::create(), "the uinput DualSense");

  // The uinput backend has no motion; the rich setters inherited from the
  // Joypad base must be safe no-ops rather than crash.
  REQUIRE_FALSE(joypad.supports_motion());
  joypad.set_motion(Joypad::ACCELERATION, 1.f, 2.f, 3.f);
  joypad.set_battery(Joypad::BATTERY_FULL, 100);
  joypad.place_finger(0, 10, 10);
  joypad.release_finger(0);

  // It still presents at least one joystick node to udev.
  auto events = joypad.get_udev_events();
  REQUIRE_FALSE(events.empty());
  bool has_joystick = false;
  for (const auto &ev : events) {
    auto it = ev.find("ID_INPUT_JOYSTICK");
    if (it != ev.end() && it->second == "1") {
      has_joystick = true;
    }
  }
  REQUIRE(has_joystick);
  REQUIRE_FALSE(joypad.get_udev_hw_db_entries().empty());
}

TEST_CASE("uinput Nintendo pad: no motion, self-describes", "[Factory]") {
  auto joypad = unwrap_or_skip(SwitchJoypadUinput::create(), "the uinput Nintendo pad");

  REQUIRE_FALSE(joypad.supports_motion());
  joypad.set_motion(Joypad::GYROSCOPE, 0.f, 0.f, 0.f); // no-op, must not crash
  REQUIRE_FALSE(joypad.get_udev_events().empty());
}

TEST_CASE("Joypad::create returns a usable pad through the Joypad base", "[Factory]") {
  // prefer_uhid = false always yields the uinput backend, which has no motion.
  auto ps = Joypad::create(JoypadKind::PS, /* prefer_uhid */ false);
  if (!ps) {
    SKIP(std::string("Skipping because the uinput PS backend is unavailable on this host: ") + ps.getErrorMessage());
  }
  REQUIRE_FALSE((*ps)->supports_motion());
  REQUIRE_FALSE((*ps)->get_udev_events().empty());

  auto nintendo = Joypad::create(JoypadKind::NINTENDO, /* prefer_uhid */ false);
  if (!nintendo) {
    SKIP(std::string("Skipping because the uinput Nintendo backend is unavailable on this host: ") +
         nintendo.getErrorMessage());
  }
  REQUIRE_FALSE((*nintendo)->supports_motion());

  auto xbox = Joypad::create(JoypadKind::XBOX);
  if (!xbox) {
    SKIP(std::string("Skipping because the uinput Xbox backend is unavailable on this host: ") + xbox.getErrorMessage());
  }
  REQUIRE_FALSE((*xbox)->supports_motion());
  REQUIRE_FALSE((*xbox)->get_udev_events().empty());
}

#ifdef USE_UHID
TEST_CASE("Joypad::create picks the rich uhid backend when preferred", "[Factory][UHID]") {
  if (!is_uhid_supported()) {
    SKIP("This host has no accessible /dev/uhid");
  }

  // The uhid DualSense / Switch Pro pads forward motion; that's the observable
  // difference from their uinput fallbacks.
  auto ps = Joypad::create(JoypadKind::PS, /* prefer_uhid */ true);
  REQUIRE(ps);
  REQUIRE((*ps)->supports_motion());

  auto nintendo = Joypad::create(JoypadKind::NINTENDO, /* prefer_uhid */ true);
  REQUIRE(nintendo);
  REQUIRE((*nintendo)->supports_motion());
}
#endif
