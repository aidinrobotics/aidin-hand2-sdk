// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace aidin_hand2
{

// --------------------------------- Error code -------------------------------

// SDK operation failure, grouped by what the caller should check
enum class ErrorCode {
  None = 0,

  // Fix the call sequence or the arguments
  InvalidArgument,
  WrongCallOrder,

  // Check the environment or the device
  InterfaceUnavailable,
  CommunicationLost,
  HardwareFault,

  // Report the SDK
  ControlLoopFault,
  UnexpectedError,
};

// ErrorCode name for logs
[[nodiscard]] constexpr const char* to_string(ErrorCode code) noexcept
{
  switch (code) {
    case ErrorCode::None: return "None";
    case ErrorCode::InvalidArgument: return "InvalidArgument";
    case ErrorCode::WrongCallOrder: return "WrongCallOrder";
    case ErrorCode::InterfaceUnavailable: return "InterfaceUnavailable";
    case ErrorCode::CommunicationLost: return "CommunicationLost";
    case ErrorCode::HardwareFault: return "HardwareFault";
    case ErrorCode::ControlLoopFault: return "ControlLoopFault";
    case ErrorCode::UnexpectedError: return "UnexpectedError";
  }
  return "UnexpectedError";
}

// --------------------------------- Exception --------------------------------

// Thrown by every public call, carrying the ErrorCode alongside the message
class Exception : public std::runtime_error {
 public:
  Exception(ErrorCode code, std::string message)
  : std::runtime_error(std::move(message)), code_(code)
  {
  }

  [[nodiscard]] ErrorCode code() const noexcept
  {
    return code_;
  }

 private:
  ErrorCode code_{ErrorCode::UnexpectedError};
};

}  // namespace aidin_hand2
