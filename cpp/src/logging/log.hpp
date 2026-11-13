#pragma once

#include <cstdint>
#include <string>

#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/error.hpp>

// Internal log API. The control loop calls it for the few events it logs directly,
// so a callback sink can run on that thread

namespace aidin_hand2
{

// The HandSide argument prefixes the message with "[left] " or "[right] "
// The code overloads put "[E<code>] " after it, as in "[left] [E404] communication lost"
void log_debug(HandSide side, const std::string& message);

void log_info(HandSide side, const std::string& message);

void log_warn(HandSide side, const std::string& message);
void log_warn(HandSide side, std::uint16_t code, const std::string& message);

void log_critical(HandSide side, const std::string& message);
void log_critical(HandSide side, std::uint16_t code, const std::string& message);

}  // namespace aidin_hand2
