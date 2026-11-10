// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cctype>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace aidin_hand2
{

// --------------------------------- Constants --------------------------------

// Fixed structural constants of the AIDIN Hand Gen2
inline constexpr std::size_t kActuatorCount          = 16;
inline constexpr std::size_t kJointCount             = 21;
inline constexpr std::size_t kActiveJointCount       = 16;

// The 4-bar coupled q4 of each digit
inline constexpr std::size_t kPassiveJointCount      = 5;

// Fingertip positions as xyz
inline constexpr std::size_t kTaskCount              = 15;

// Thumb 6-D pose with 4 fingertip positions
inline constexpr std::size_t kOrientedTaskCount      = 18;

inline constexpr std::size_t kFingerCount            = 5;
inline constexpr std::size_t kTactileTaxelsPerFinger = 17;
inline constexpr std::size_t kPalmTactileCount       = 58;
inline constexpr std::size_t kPalm1UpperCount        = 20;
inline constexpr std::size_t kPalm1LowerCount        = 20;
inline constexpr std::size_t kPalm2Count             = 18;

// ----------------------------------- Enums ----------------------------------

enum class HandSide {
  Left,
  Right,
};

// Gen2_B carries the 2 mm lead on the thumb d1 and d2 screws, Gen2 the 1 mm lead
enum class HandType {
  Gen2,
  Gen2_B,
};

// "gen2" or "gen2-b" in any letter case, "gen2_b" too; nullopt for anything else
inline std::optional<HandType> parse_hand_type(std::string_view text)
{
  std::string lower(text);
  for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (lower == "gen2") return HandType::Gen2;
  if (lower == "gen2-b" || lower == "gen2_b") return HandType::Gen2_B;
  return std::nullopt;
}

// "gen2" or "gen2-b"
inline const char* to_string(HandType type)
{
  return type == HandType::Gen2_B ? "gen2-b" : "gen2";
}

enum class Finger {
  Thumb,
  Index,
  Middle,
  Ring,
  Baby,
};

}  // namespace aidin_hand2
