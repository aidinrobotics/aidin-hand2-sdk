// Exception carries a kind and a code, what() leads with the code, and the hundreds digit is the kind
#include <cstdio>
#include <string>

#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/error.hpp>
#include <aidin_hand2/types/error_codes.hpp>

using namespace aidin_hand2;

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

static_assert(kind_of(0) == ErrorKind::None);
static_assert(kind_of(101) == ErrorKind::InvalidArgument);
static_assert(kind_of(207) == ErrorKind::WrongCallOrder);
static_assert(kind_of(301) == ErrorKind::InterfaceUnavailable);
static_assert(kind_of(404) == ErrorKind::CommunicationLost);
static_assert(kind_of(503) == ErrorKind::HardwareFault);
static_assert(kind_of(601) == ErrorKind::HandFaulted);
static_assert(kind_of(702) == ErrorKind::ControlLoopFault);
static_assert(kind_of(901) == ErrorKind::UnexpectedError);

// Numbered without gaps, the way the error message table lists them
static_assert(error_code::kMaxEffortInvalid == 104);
static_assert(error_code::kImpedanceCutoffInvalid == 107);
static_assert(error_code::kHandFaulted == 601);
static_assert(error_code::kControlLoopTerminated == 702);
static_assert(error_code::kEventLoggingTerminated == 703);

void test_sdk_exception()
{
  const Exception error(401, "cannot create hands: no data received from can0 within 300 ms");
  check(error.code() == 401, "code() is the number");
  check(error.kind() == ErrorKind::CommunicationLost, "kind() follows the hundreds digit");
  check(std::string(error.what()) == "[E401] cannot create hands: no data received from can0 within 300 ms",
        "what() leads with the code");
}

// A failure about one hand names it first, the way its log lines do
void test_sdk_exception_of_a_hand()
{
  const Exception error(HandSide::Left, 401, "cannot connect hand: no data received from can0 within 300 ms");
  check(error.code() == 401, "code() is the number");
  check(error.kind() == ErrorKind::CommunicationLost, "kind() follows the hundreds digit");
  check(std::string(error.what()) == "[left] [E401] cannot connect hand: no data received from can0 within 300 ms",
        "what() leads with the side, then the code");
}

// An application's own exception has no SDK code, so its message is left as it is
void test_application_exception()
{
  const Exception error(ErrorKind::InvalidArgument, "unknown command: jump");
  check(error.code() == 0, "application exception has code 0");
  check(error.kind() == ErrorKind::InvalidArgument, "application exception keeps its kind");
  check(std::string(error.what()) == "unknown command: jump", "application message has no prefix");
}

// HandConfig throws from a public header, so its numbers are written there and must match the table
void test_hand_config_codes()
{
  try {
    HandConfig config{"", HandSide::Left, HandModel::Gen2};
    check(false, "empty interface_name throws");
  } catch (const Exception& error) {
    check(error.code() == error_code::kInterfaceNameEmpty, "empty interface_name code");
    check(std::string(error.what()) == "[E101] cannot create HandConfig: interface_name must not be empty",
          "a config error has no side, as no hand exists yet");
  }
  try {
    HandConfig config{"can0", static_cast<HandSide>(9), HandModel::Gen2};
    check(false, "invalid hand_side throws");
  } catch (const Exception& error) {
    check(error.code() == error_code::kHandSideInvalid, "invalid hand_side code");
  }
  try {
    HandConfig config{"can0", HandSide::Left, static_cast<HandModel>(9)};
    check(false, "invalid hand_model throws");
  } catch (const Exception& error) {
    check(error.code() == error_code::kHandModelInvalid, "invalid hand_model code");
  }
}

}  // namespace

int main()
{
  test_sdk_exception();
  test_sdk_exception_of_a_hand();
  test_application_exception();
  test_hand_config_codes();

  if (g_failures == 0) std::printf("error: all checks passed\n");
  return g_failures == 0 ? 0 : 1;
}
