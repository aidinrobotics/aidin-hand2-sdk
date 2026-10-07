// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, tactile readings and the tactile bias
//
// Prints raw tactile values, takes the bias with set_tactile_bias(), then prints the largest
// difference from the bias per finger and palm region until Ctrl-C. reset_tactile_bias() at the
// end brings the raw values back. Stays in Connected, so nothing moves.
//
// Run as: 15_tactile [right|left] [interface=can0]
// Needs a real hand. Keep your hands off it until the bias is taken.

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the display loop to stop. The HandManager destructor is what closes the
// link, so the point is to leave the loop and reach that destructor.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  // Finger index follows the Finger enum: Thumb, Index, Middle, Ring, Baby
  const std::array<const char*, kFingerCount> finger_name = {"thumb", "index", "middle", "ring", "baby"};

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;  // this example stays in Connected, so homing never comes up

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 1) Raw values before the bias
    const HandState first = hand.get_state();
    std::printf("[example] raw values, %s fingertip taxels 0-5:", finger_name[1]);
    for (std::size_t t = 0; t < 6; ++t) {
      std::printf(" %7d", first.tactile.fingers[1][t]);
    }
    std::printf("\n");
    std::printf("[example] raw values, palm taxels 0-5:        ");
    for (std::size_t t = 0; t < 6; ++t) {
      std::printf(" %7d", first.tactile.palm[t]);
    }
    std::printf("\n\n[example] none of those say whether anything is touching the hand\n\n");

    // 2) Take the bias with nothing in contact. It applies from the next cycle, so wait one
    std::printf("[example] taking the bias — keep your hands off the hand\n");
    hand.set_tactile_bias();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const TactileState bias = hand.get_tactile_bias();
    std::printf("[example] bias taken, %s fingertip taxels 0-5:   ", finger_name[1]);
    for (std::size_t t = 0; t < 6; ++t) {
      std::printf(" %7d", bias.fingers[1][t]);
    }
    std::printf("\n\n");

    // 3) Contact threshold chosen for this example, the SDK does not provide one
    constexpr int kContactThreshold = 200;

    std::printf("[example] touch a fingertip or the palm — Ctrl-C to finish\n");
    std::printf("[example] each column is one finger: peak deviation, then which taxel, then contact\n\n");

    while (!g_shutdown.load()) {
      const HandState state = hand.get_state();

      // 4) Per finger, the taxel with the largest difference
      std::printf("\r\033[K[example] ");
      for (std::size_t f = 0; f < kFingerCount; ++f) {
        int         peak       = 0;
        std::size_t peak_taxel = 0;
        for (std::size_t t = 0; t < kTactileTaxelsPerFinger; ++t) {
          const int deviation = std::abs(state.tactile.fingers[f][t]);
          if (deviation > peak) {
            peak       = deviation;
            peak_taxel = t;
          }
        }
        std::printf("%s %6d t%-2zu %s  ", finger_name[f], peak, peak_taxel,
                    peak > kContactThreshold ? "**" : "  ");
      }

      // 5) Palm, the taxel with the largest difference: palm1 upper 20, palm1 lower 20, palm2 18
      int         palm_peak  = 0;
      std::size_t palm_taxel = 0;
      for (std::size_t t = 0; t < kPalmTactileCount; ++t) {
        const int deviation = std::abs(state.tactile.palm[t]);
        if (deviation > palm_peak) {
          palm_peak  = deviation;
          palm_taxel = t;
        }
      }
      const char* region = palm_taxel < kPalm1UpperCount ? "palm1 upper"
                           : palm_taxel < kPalm1UpperCount + kPalm1LowerCount ? "palm1 lower"
                                                                             : "palm2      ";
      std::printf("| %s %6d t%-2zu %s", region, palm_peak, palm_taxel,
                  palm_peak > kContactThreshold ? "**" : "  ");
      std::fflush(stdout);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 6) Back to raw values, the bias otherwise stays until reset
    hand.reset_tactile_bias();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const HandState raw = hand.get_state();
    std::printf("[example] bias reset, raw again, %s fingertip taxels 0-5:", finger_name[1]);
    for (std::size_t t = 0; t < 6; ++t) {
      std::printf(" %7d", raw.tactile.fingers[1][t]);
    }
    std::printf("\n[example] done — call set_tactile_bias() again whenever nothing is in contact\n");
  } catch (const Exception& error) {
    std::printf("\n[example] failed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
