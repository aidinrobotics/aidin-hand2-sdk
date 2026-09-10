#pragma once

#include <string>

#include <aidin_hand2/types/config.hpp>  // HandSide
#include <aidin_hand2/types/error.hpp>

#include "logging/log.hpp"

namespace aidin_hand2
{

// Failure result returned by internal calls
struct Status {
  ErrorCode code{ErrorCode::None};
  std::string message{};

  [[nodiscard]] bool ok() const noexcept
  {
    return code == ErrorCode::None;
  }
};

// Logs the failure before throwing
inline void throw_if_error(const Status& status)
{
  if (!status.ok()) {
    log_exception(status.code, status.message);
    throw Exception(status.code, status.message);
  }
}

inline void throw_if_error(HandSide side, const Status& status)
{
  if (!status.ok()) {
    log_exception(side, status.code, status.message);
    throw Exception(status.code, status.message);
  }
}

[[noreturn]] inline void throw_error(ErrorCode code, std::string message)
{
  log_exception(code, message);
  throw Exception(code, std::move(message));
}

[[noreturn]] inline void throw_error(HandSide side, ErrorCode code, std::string message)
{
  log_exception(side, code, message);
  throw Exception(code, std::move(message));
}

// Rethrows an Exception as is and wraps anything else as UnexpectedError
// operation is the public API name, as in "connect()"
template <class Callable>
auto guard_boundary(const char* operation, Callable&& callable) -> decltype(callable())
{
  try {
    return callable();
  } catch (const Exception&) {
    throw;
  } catch (const std::exception& unexpected) {
    throw_error(ErrorCode::UnexpectedError,
                std::string("Unexpected failure in ") + operation + ": " + unexpected.what());
  } catch (...) {
    throw_error(ErrorCode::UnexpectedError,
                std::string("Unexpected failure in ") + operation + ": unknown exception");
  }
}

}  // namespace aidin_hand2
