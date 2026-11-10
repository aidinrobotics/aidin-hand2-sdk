#pragma once

#include <array>

#include <aidin_hand2/types/description.hpp>

#include "hand_core/kinematics/hand_kinematics_core.hpp"

namespace aidin_hand2
{

// What differs between hand types: the kinematics boundary and the impedance hard stiffness
struct HandModel
{
  HandType type;
  const kinematics::Boundary& kinematics;

  // N·m/rad per active joint at stiffness 1; the long finger j3 follows the pose instead
  std::array<double, kActiveJointCount> hard_stiffness;
};

// The model of a hand type
const HandModel& hand_model_of(HandType type);

}  // namespace aidin_hand2
