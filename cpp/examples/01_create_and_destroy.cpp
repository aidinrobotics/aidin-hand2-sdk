// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, who owns the hand and what a Hand handle actually is
//
// HandManager owns the resources; Hand is a handle that points at them. Copying a Hand does
// not copy anything behind it, and destroy() invalidates every handle that pointed there —
// this example calls a method on a stale handle to show what that looks like.
//
// Nothing moves: no run(), no home(), no set_command(). It does not even connect, because
// ownership is decided before any CAN traffic.
//
// Run as: 01_create_and_destroy [right|left] [interface=can0]
// Runs without a hand attached, since create() validates the config and allocates only.

#include <atomic>
#include <cstdio>
#include <csignal>
#include <string>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT is caught so that Ctrl-C leaves through the destructors below rather than killing
// the process mid-call. Nothing here loops, so the flag is only read on the way out.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  // 1) A config is checked twice: the constructor rejects an empty interface name or a side
  //    outside the enum, and create() rejects a control_rate of 0 or less.
  HandConfig config{argc > 2 ? argv[2] : "can0", side};
  config.control_rate = 500;    // Hz
  config.auto_home    = false;  // nothing here asks for control, so homing never comes up

  std::printf("config: interface=%s  side=%s  control_rate=%d Hz\n\n",
              config.interface_name.c_str(),
              side == HandSide::Left ? "Left" : "Right",
              config.control_rate);

  try {
    // 2) The manager holds the HandCore. It is move-only, so there is exactly one owner.
    HandManager manager;

    Hand hand = manager.create(config);
    std::printf("create() returned a handle, lifecycle = %s\n",
                to_string(hand.get_diagnostics().lifecycle));

    // 3) A copy is a second handle to the same HandCore. There is no second hand and no
    //    second control loop — both names read the same state.
    Hand copy = hand;
    std::printf("copied the handle, and both read lifecycle = %s / %s\n",
                to_string(hand.get_diagnostics().lifecycle),
                to_string(copy.get_diagnostics().lifecycle));

    // 4) destroy() releases the HandCore that this handle points at. If it were connected it
    //    would confirm the quick stop and close the link first, so a disconnect() beforehand
    //    is never required.
    manager.destroy(hand);
    std::printf("\ndestroy() released the HandCore\n");

    // 5) Both handles are stale now, the copy included. A method on either one throws instead
    //    of touching freed resources.
    try {
      (void)copy.get_diagnostics();
      std::printf("the copy still answered, which the SDK is not supposed to allow\n");
    } catch (const Exception& error) {
      std::printf("the copy is invalid too — %s: %s\n", to_string(error.code()), error.what());
    }

    // 6) destroy_all() is the same for everything the manager owns, and the destructor does
    //    it anyway. Calling it with nothing left is harmless.
    manager.destroy_all();
    std::printf("\ndestroy_all() on an empty manager is a no-op\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  std::printf("done — the manager went out of scope with nothing left to release\n");
  return 0;
}
