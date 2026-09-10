// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, calling the lifecycle by hand to see which calls the state accepts
//
// Pick a transition by number and watch what happens. Every call is offered from every state,
// so the refusals are as much the point as the transitions: run() before connect(), reconnect()
// when nothing has faulted, set_command() before homing. The reason the SDK gives is printed
// as it comes back.
//
// The bottom line refreshes about five times a second, so the state is live even while nothing
// is typed. Pull the CAN cable out and it turns to Faulted on its own about 100 ms after RX
// stops, and from there reconnect() is the only call that gets you out.
//
// Run as: 03_lifecycle_walkthrough [right|left] [interface=can0]
// Needs a real hand. run() and home() enable the drives — keep the surroundings clear, and
// remember that home() drives every finger into its hard stop.

#include <atomic>
#include <cstdio>
#include <csignal>
#include <string>

#include <poll.h>
#include <unistd.h>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the loop below to stop. The HandManager destructor is what confirms the
// quick stop and closes the link, so the point is to leave the loop and reach that destructor.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate   = 500;    // Hz
    config.auto_home      = false;  // home() is on the menu, so run() should not do it for you
    config.auto_reconnect = false;  // recovery stays manual here, or Faulted would clear itself

    HandManager manager;
    Hand hand = manager.create(config);

    std::printf("\n  1 connect     2 run          3 stop    4 home\n");
    std::printf("  5 reconnect   6 disconnect   7 set_command(Idle)        0 quit\n\n");
    std::printf("Every number is accepted whatever the state is. What the SDK does with it —\n");
    std::printf("the transition or the refusal — is what gets printed.\n\n");

    // stdin is polled instead of read straight through, so the state line below can be redrawn
    // while nothing is being typed. That is what makes a fault show up without a keypress.
    struct pollfd stdin_poll = {STDIN_FILENO, POLLIN, 0};

    while (!g_shutdown.load()) {
      // Read the state fresh every pass: it moves on its own when the control loop faults.
      const Diagnostics diagnostics = hand.get_diagnostics();
      // \r returns to the start of the line and \033[K clears what was there, so this one
      // line is rewritten in place rather than scrolling the history away.
      std::printf("\r\033[K[%s / %s]  cycles %llu  > ",
                  to_string(diagnostics.lifecycle), to_string(diagnostics.homing_state),
                  static_cast<unsigned long long>(diagnostics.control_cycles));
      std::fflush(stdout);

      if (poll(&stdin_poll, 1, 200) <= 0) continue;  // nothing typed — redraw and wait again

      char line[16] = {0};
      if (std::fgets(line, sizeof(line), stdin) == nullptr) break;  // Ctrl-D
      const char choice = line[0];
      if (choice == '0') break;
      if (choice == '\n') continue;

      std::printf("\n");  // leave the live line behind and let this round scroll up

      try {
        switch (choice) {
          case '1':
            // Disconnected only. Called in Connected it is a no-op that logs what it skipped;
            // in Running or Stopped it is refused, because a rebuild has to go through
            // disconnect() first.
            hand.connect();
            std::printf("  connect() returned\n");
            break;

          case '2':
            // Connected or Stopped. It blocks until the drives report Operation Enabled, so a
            // return means the hardware got there. In Faulted it is refused.
            hand.run();
            std::printf("  run() returned — the drives confirmed the enable\n");
            break;

          case '3':
            // Running only. It blocks until the drives reach quick stop, and resets the stored
            // command to Idle. In Connected there is no control to stop, so it is refused.
            hand.stop();
            std::printf("  stop() returned — the drives confirmed the quick stop\n");
            break;

          case '4':
            // Connected, Running or Stopped. It enables the drives if they are not already, so
            // there is no need to call run() first. Every finger travels to its hard stop.
            hand.home();
            std::printf("  home() returned — homing_state is now Succeeded\n");
            break;

          case '5':
            // Faulted only. Outside Faulted it is refused on purpose: a fault should not be
            // cleared without its cause being seen. It resets homing_state to NotRun and does
            // not resume control, so run() is still needed afterwards.
            hand.reconnect();
            std::printf("  reconnect() returned — control has to be asked for again\n");
            break;

          case '6':
            // Anything but Disconnected. It confirms the quick stop before closing, and when it
            // cannot confirm it throws and leaves the socket open rather than dropping a hand
            // that may still hold torque.
            hand.disconnect();
            std::printf("  disconnect() returned\n");
            break;

          case '7':
            // Running only, and every command other than Idle also needs homing_state to be
            // Succeeded. Idle sends an effort of 0 while the drives stay on, so a joint moved
            // by hand still meets some resistance.
            hand.set_command(Idle{});
            std::printf("  set_command(Idle) returned\n");
            break;

          default:
            std::printf("  no such command\n");
            continue;
        }
      } catch (const Exception& error) {
        // This is the interesting half. The code says what kind of failure it is and the
        // message says why, and the SDK has already logged both at error level.
        std::printf("  refused — %s: %s\n", to_string(error.code()), error.what());
      }
    }

    std::printf("\nleaving — the manager destructor stops the hand and closes the link\n");
  } catch (const Exception& error) {
    // Only create() throws out here. Everything inside the loop is caught above so that a
    // refusal does not end the session.
    std::printf("\nfailed to start: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
