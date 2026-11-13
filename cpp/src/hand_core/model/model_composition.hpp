#pragma once

#include <array>

#include <aidin_hand2/types/description.hpp>

#include "hand_core/kinematics/kinematics.hpp"
#include "types/actuator_constants.hpp"

namespace aidin_hand2
{

// What a hand model is composed of, read once where a hand is built
struct ModelComposition {
  const kinematics::Solver& solver;

  // Nm/rad per active joint at stiffness 1; the long finger j3 follows the pose instead
  std::array<double, kActiveJointCount> hard_stiffness;

  ActuatorConstants actuator;
};

// The composition of a hand model
const ModelComposition& composition_of(HandModel model);

// In model/gen2.cpp and model/gen2b.cpp
extern const ModelComposition kGen2Composition;
extern const ModelComposition kGen2BComposition;

}  // namespace aidin_hand2
