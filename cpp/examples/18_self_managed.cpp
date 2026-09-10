// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, a loop that keeps the lifecycle in Running on its own
//
// create -> loop { read the lifecycle -> fill in the missing step -> send the next waypoint }
// The switch sits inside the try rather than the catch, so a failed recovery call lands in the
// same handler, the first cycle performs the initial connect from Disconnected, and a lifecycle
// change that arrives without an exception (auto-reconnect finishing, for one) is picked up too
//
// Run as: 18_self_managed [right|left] [interface=can0]
// Needs a real hand. auto_home is on, so the first run() drives every finger into its hard stop:
// clear the space around the hand before running. Ctrl-C leaves the loop and the HandManager
// destructor confirms the quick stop on the way out

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <thread>
#include <vector>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// Paste a trajectory exported from the GUI sequence tab here. The abduction joints
// (thumb j2 and the j1 of every other finger) stay at 0
struct Waypoint {
  std::array<double, kActiveJointCount> target;   // rad
  int hold_ms;
};

const std::vector<Waypoint> kSequence = {
    {{0.00, 0.00, 0.00, 0.00,   // thumb   j0 j1 j2 j3
      0.00, 0.00, 0.00,         // index   j1 j2 j3
      0.00, 0.00, 0.00,         // middle
      0.00, 0.00, 0.00,         // ring
      0.00, 0.00, 0.00},        // baby
     1000},
    {{0.15, 0.25, 0.00, 0.20,
      0.00, 0.30, 0.25,
      0.00, 0.30, 0.25,
      0.00, 0.30, 0.25,
      0.00, 0.30, 0.25},
     800},
    {{0.30, 0.50, 0.00, 0.40,
      0.00, 0.60, 0.50,
      0.00, 0.60, 0.50,
      0.00, 0.60, 0.50,
      0.00, 0.60, 0.50},
     1200},
    {{0.15, 0.25, 0.00, 0.20,
      0.00, 0.30, 0.25,
      0.00, 0.30, 0.25,
      0.00, 0.30, 0.25,
      0.00, 0.30, 0.25},
     800},
};

std::atomic<bool> g_shutdown{false};

// Picks the waypoint the elapsed time falls into, wrapping back to the first one.
// Driving it from the clock rather than a counter keeps the trajectory on time when a
// cycle is late
JointPositionCommand next_command()
{
  static const auto start = std::chrono::steady_clock::now();

  int total_ms = 0;
  for (const Waypoint& point : kSequence) total_ms += point.hold_ms;

  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start)
                           .count();

  JointPositionCommand command;
  int offset_ms = static_cast<int>(elapsed % total_ms);
  for (const Waypoint& point : kSequence) {
    if (offset_ms < point.hold_ms) {
      command.target = point.target;
      return command;
    }
    offset_ms -= point.hold_ms;
  }
  command.target = kSequence.back().target;
  return command;
}

// 50 Hz on an absolute deadline, so a slow cycle does not push every later one
void wait_next_period()
{
  static auto next = std::chrono::steady_clock::now();
  next += std::chrono::milliseconds(20);
  std::this_thread::sleep_until(next);
}
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate        = 500;    // Hz
    config.auto_home           = true;   // run() homes before it enables control
    config.auto_reconnect      = true;   // the control loop rebuilds a dropped link by itself
    config.auto_reconnect_home = true;

    // The destructor confirms the quick stop and closes the link, so there is no
    // stop()/disconnect() pair at the end
    HandManager manager;
    Hand hand = manager.create(config);
    hand.set_max_effort(600.0);          // percent of rated current, independent of the lifecycle

    while (!g_shutdown.load()) {
      try {
        switch (hand.get_diagnostics().lifecycle) {
          case HandLifecycle::Faulted:      hand.reconnect();  [[fallthrough]];
          case HandLifecycle::Disconnected: hand.connect();    [[fallthrough]];
          case HandLifecycle::Connected:
          case HandLifecycle::Stopped:      hand.run();        break;
          case HandLifecycle::Running:      break;
        }
        hand.set_command(next_command());
      } catch (const Exception&) {
        // The SDK logged the cause before throwing. Skipping this cycle is enough:
        // the switch on the next one performs whatever step is still missing
      }
      wait_next_period();
    }
  } catch (const Exception&) {
    // Never started. The config was rejected, or the hand could not be created
    return 1;
  }
  return 0;
}
