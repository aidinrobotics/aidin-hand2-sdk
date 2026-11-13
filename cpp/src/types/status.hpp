#pragma once

#include <cstdint>
#include <string>

#include <aidin_hand2/types/error.hpp>
#include <aidin_hand2/types/error_codes.hpp>

namespace aidin_hand2
{

// Failure result returned by internal calls
struct Status {
  // A number from error_codes.hpp, 0 on success
  std::uint16_t code{0};
  std::string message{};

  [[nodiscard]] bool ok() const noexcept
  {
    return code == 0;
  }

  [[nodiscard]] ErrorKind kind() const noexcept
  {
    return kind_of(code);
  }
};

// The caller reports a thrown failure, so nothing here logs it
// A failure about one hand reads "[left] [E<code>] <message>"
[[noreturn]] inline void throw_error(HandSide side, std::uint16_t code, const std::string& message)
{
  throw Exception(side, code, message);
}

// For a failure that is about no single hand, as in destroy_all()
[[noreturn]] inline void throw_error(std::uint16_t code, const std::string& message)
{
  throw Exception(code, message);
}

inline void throw_if_error(HandSide side, const Status& status)
{
  if (!status.ok()) {
    throw_error(side, status.code, status.message);
  }
}

// Builds "cannot <action>: unexpected failure (<detail>)"
inline std::string unexpected_failure(const char* action, const char* detail)
{
  return std::string("cannot ") + action + ": unexpected failure (" + detail + ")";
}

// Rethrows an Exception as is and wraps anything else as E901
// action is the "<action>" of a "cannot <action>: " message, as in "connect hand"
template <class Callable>
auto guard_boundary(HandSide side, const char* action, Callable&& callable) -> decltype(callable())
{
  try {
    return callable();
  } catch (const Exception&) {
    throw;
  } catch (const std::exception& unexpected) {
    throw_error(side, error_code::kUnexpectedException, unexpected_failure(action, unexpected.what()));
  } catch (...) {
    throw_error(side, error_code::kUnexpectedException, unexpected_failure(action, "unknown exception"));
  }
}

// The same for a call that spans both hands, as destroy_all() does
template <class Callable>
auto guard_boundary(const char* action, Callable&& callable) -> decltype(callable())
{
  try {
    return callable();
  } catch (const Exception&) {
    throw;
  } catch (const std::exception& unexpected) {
    throw_error(error_code::kUnexpectedException, unexpected_failure(action, unexpected.what()));
  } catch (...) {
    throw_error(error_code::kUnexpectedException, unexpected_failure(action, "unknown exception"));
  }
}

}  // namespace aidin_hand2
