#pragma once

#include <array>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/description.hpp>

namespace aidin_hand2
{

// One RT buffer slot, so the controller command and the effort cap reach the loop together
struct HandCommand {
  ControllerCommand controller{Idle{}};
  std::array<double, kActuatorCount> max_effort_pct{};
};

}  // namespace aidin_hand2
