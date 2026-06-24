#pragma once

// Shared plumbing for the uhid joypad backends (PS5 / Switch Pro). Both create a
// uhid device, run a report-pump thread, guard the device with a mutex, and can
// rebuild it in place (lobby hand-off). Only the report layout (send_report) and
// the pump cadence differ, so the lifecycle lives here once.

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <uhid/uhid.hpp>
#include <vector>

namespace inputtino::uhid_joypad {

/**
 * State common to every uhid joypad. Concrete joypad states inherit this and add
 * their report-specific fields. `mtx` serialises access to `dev` between the
 * report-pump thread, the client-driven control thread (set_motion / buttons),
 * and recreate_device().
 */
struct CommonState {
  std::shared_ptr<uhid::Device> dev;
  uhid::DeviceDefinition def = {};
  std::mutex mtx;
  bool stop_repeat_thread = false;
};

/**
 * Block until the kernel has exposed the device's sysfs input/hidraw nodes (or we
 * give up after max_attempts). Callers that read get_udev_events()/get_nodes()
 * straight after create()/recreate() (e.g. plugging the pad into a container)
 * would otherwise see an empty device and it wouldn't enumerate.
 */
inline void wait_for_sys_nodes(const std::function<std::vector<std::string>()> &get_sys_nodes,
                               int max_attempts = 20,
                               std::chrono::milliseconds interval = std::chrono::milliseconds(50)) {
  for (int attempt = 0; attempt < max_attempts && get_sys_nodes().empty(); ++attempt) {
    std::this_thread::sleep_for(interval);
  }
}

/**
 * Start the report-pump thread: re-sends the HID input report every `interval` so
 * the device stays "live" for readers even when its state hasn't changed. The
 * thread holds a shared_ptr to the state so it stays valid across joypad moves,
 * and exits once state->stop_repeat_thread is set.
 */
template <typename State>
std::thread start_report_pump(std::shared_ptr<State> state,
                              std::function<void(State &)> send,
                              std::chrono::milliseconds interval) {
  return std::thread([state, send = std::move(send), interval]() {
    while (!state->stop_repeat_thread) {
      send(*state);
      std::this_thread::sleep_for(interval);
    }
  });
}

/**
 * Rebuild the uhid device in place (UHID_DESTROY + re-create with the same
 * definition, so MAC/uniq and the /dev/input/* layout stay stable). Used for the
 * lobby hand-off where another container takes over the pad.
 *
 * Ordering matters: the report pump and the device's own event listener are
 * stopped+joined *before* taking `mtx` (on_uhid_event can take mtx, so holding it
 * while joining would deadlock), then the device swap happens under `mtx` so a
 * concurrent client-driven send_report() can't dereference a null `dev`.
 */
template <typename State>
void recreate_device(std::shared_ptr<State> &state,
                     std::thread &pump_thread,
                     const std::function<void(const uhid_event &, int)> &on_event,
                     std::function<void(State &)> send,
                     std::chrono::milliseconds interval,
                     const std::function<std::vector<std::string>()> &get_sys_nodes,
                     const char *what) {
  if (!state || !state->dev) {
    return;
  }

  state->stop_repeat_thread = true;
  if (pump_thread.joinable()) {
    pump_thread.join();
  }
  state->dev->stop_thread();

  {
    std::lock_guard<std::mutex> lock(state->mtx);
    state->dev.reset();
    auto dev = uhid::Device::create(state->def, on_event);
    if (!dev) {
      fprintf(stderr, "Unable to recreate %s joypad device: %s\n", what, dev.getErrorMessage().c_str());
      return;
    }
    state->dev = std::make_shared<uhid::Device>(std::move(*dev));
  }

  state->stop_repeat_thread = false;
  pump_thread = start_report_pump<State>(state, std::move(send), interval);
  wait_for_sys_nodes(get_sys_nodes);
}

} // namespace inputtino::uhid_joypad
