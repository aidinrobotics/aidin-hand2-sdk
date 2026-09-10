#pragma once

#include <string>

#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/error.hpp>

// Internal log API. The control loop calls it for the few events it logs directly,
// so a callback sink can run on that thread

namespace aidin_hand2
{

// The HandSide overloads prefix the message with "[left] " or "[right] "
void log_debug(const std::string& message);
void log_debug(HandSide side, const std::string& message);

void log_info(const std::string& message);
void log_info(HandSide side, const std::string& message);

void log_warn(const std::string& message);
void log_warn(HandSide side, const std::string& message);

void log_critical(const std::string& message);
void log_critical(HandSide side, const std::string& message);

// Writes "[exception] <ErrorCode>: <message>" at error level
void log_exception(ErrorCode code, const std::string& message);
void log_exception(HandSide side, ErrorCode code, const std::string& message);

// Writes "<ErrorCode>: <message>" at warn level
void log_warn_status(ErrorCode code, const std::string& message);

}  // namespace aidin_hand2
