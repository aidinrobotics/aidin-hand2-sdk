// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, two hands under one HandManager
//
// One manager can own several HandCore instances. Each hand gets its own control and
// communication loop on its own CAN interface, so they advance independently — the cycle
// counters printed below drift apart, which is the point.
//
// create() twice with configs that differ in interface_name and hand_side is the whole of it.
// destroy_all() releases both, and the destructor would do the same.
//
// Run as: 02_two_hands [left_interface=can0] [right_interface=can1]
// Needs two hands, one per interface. Nothing moves: no run(), no home(), no set_command().

#include <atomic>
#include <chrono>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the poll loop to stop. The HandManager destructor is what closes both
// links, so the point is to leave the loop and reach that destructor.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const std::string left_interface  = argc > 1 ? argv[1] : "can0";
  const std::string right_interface = argc > 2 ? argv[2] : "can1";

  try {
    // 1) Two configs. The side tells the SDK which CAN IDs to expect — left answers on 0x2xx
    //    and right on 0x1xx — so a swapped pair shows up as no data rather than as odd motion.
    HandConfig left_config{left_interface, HandSide::Left};
    left_config.control_rate = 500;    // Hz
    left_config.auto_home    = false;  // nothing here asks for control

    HandConfig right_config{right_interface, HandSide::Right};
    right_config.control_rate = 500;
    right_config.auto_home    = false;

    // 2) One manager, two HandCore instances. Nothing is shared between them.
    HandManager manager;
    Hand left  = manager.create(left_config);
    Hand right = manager.create(right_config);

    std::printf("created two hands: %s (left) and %s (right)\n\n",
                left_interface.c_str(), right_interface.c_str());

    // 3) Each connect() starts that hand's own loop. Bringing one up does not touch the other,
    //    which is why the counters below do not stay in step.
    left.connect();
    right.connect();
    std::printf("both connected\n\n");

    // 4) Poll for 3 s. Each hand answers from its own buffers.
    for (int i = 0; i < 30 && !g_shutdown.load(); ++i) {
      const Diagnostics left_diagnostics  = left.get_diagnostics();
      const Diagnostics right_diagnostics = right.get_diagnostics();
      const HandState   left_state        = left.get_state();
      const HandState   right_state       = right.get_state();

      std::printf("\r\033[K left %s cycles %llu  |  right %s cycles %llu"
                  "  |  thumb a0 %.0f / %.0f cnt",
                  to_string(left_diagnostics.lifecycle),
                  static_cast<unsigned long long>(left_diagnostics.control_cycles),
                  to_string(right_diagnostics.lifecycle),
                  static_cast<unsigned long long>(right_diagnostics.control_cycles),
                  left_state.actuators.position_count[0],
                  right_state.actuators.position_count[0]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 5) A fault on one hand is that hand's fault. The other keeps running, so an application
    //    driving a pair has to read both lifecycles rather than one.
    std::printf("left lifecycle  = %s\n", to_string(left.get_diagnostics().lifecycle));
    std::printf("right lifecycle = %s\n\n", to_string(right.get_diagnostics().lifecycle));

    // 6) destroy_all() confirms the quick stop on each hand and closes both links.
    manager.destroy_all();
    std::printf("destroy_all() released both hands\n");
  } catch (const Exception& error) {
    // The message names the interface that failed, which is what tells the two apart here.
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
