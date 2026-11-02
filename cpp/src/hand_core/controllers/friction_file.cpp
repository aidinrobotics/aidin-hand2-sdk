#include <aidin_hand2/types/config.hpp>

#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include "hand_core/util/json_value.hpp"

namespace aidin_hand2
{

namespace
{

constexpr std::string_view kFrictionFileFormat = "aidin-hand2-friction/1";

[[noreturn]] void reject(const std::string& path, const std::string& why)
{
  throw Exception(ErrorCode::InvalidArgument, "load_friction_file: " + path + ": " + why);
}

}  // namespace

void load_friction_file(const std::string& path, ControllerConfig::JointImpedanceController& config)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) reject(path, "cannot be opened");
  std::ostringstream text;
  text << file.rdbuf();

  const std::optional<JsonValue> root = JsonValue::parse(text.str());
  if (!root || !root->is_object()) reject(path, "is not JSON");
  const JsonValue* format = root->find("format");
  if (format == nullptr || format->as_string() != kFrictionFileFormat) {
    reject(path, "format is not " + std::string(kFrictionFileFormat));
  }
  const JsonValue* actuators = root->find("actuators");
  if (actuators == nullptr || !actuators->is_array()) reject(path, "has no actuators");

  ControllerConfig::JointImpedanceController::CoulombFriction coulomb{};
  for (std::size_t k = 0; k < actuators->size(); ++k) {
    const JsonValue* actuator = actuators->at(k);
    const auto number = [&](const char* key) {
      const JsonValue* value = actuator->find(key);
      const std::optional<double> n = value != nullptr ? value->as_number() : std::nullopt;
      if (!n || !std::isfinite(*n) || *n < 0.0) {
        reject(path, "actuators[" + std::to_string(k) + "]." + key + " is not a number >= 0");
      }
      return *n;
    };
    const double index = number("index");
    if (index != std::floor(index) || index >= static_cast<double>(kActuatorCount)) {
      reject(path, "actuators[" + std::to_string(k) + "].index is not an actuator");
    }
    const auto i = static_cast<std::size_t>(index);
    coulomb.positive[i] = number("coulombPositive");
    coulomb.negative[i] = number("coulombNegative");
    coulomb.viscous[i]  = number("viscous");
  }

  config.coulomb_friction = coulomb;
  config.friction_model   = ControllerConfig::JointImpedanceController::FrictionModel::Coulomb;
}

}  // namespace aidin_hand2
