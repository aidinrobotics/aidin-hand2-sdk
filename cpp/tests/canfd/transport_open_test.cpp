// Transport::open() refuses a missing or down interface and leaves its configuration alone
#include <sched.h>

#include <cstdio>
#include <string>

#include "hand_core/comms/canfd/transport.hpp"

using namespace aidin_hand2;

namespace
{

int g_failures = 0;

// ctest reads this exit code as skipped
constexpr int kSkipped = 77;

void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

void test_missing_interface()
{
  canfd::Transport transport;
  const Status status = transport.open("aidin_no_if");
  check(status.code == 301, "missing interface is E301");
  check(status.message == "CAN interface 'aidin_no_if' not found",
        "missing interface message");
  check(!transport.is_open(), "missing interface leaves the transport closed");
}

// A fresh network namespace holds only lo, and it starts down
void test_down_interface()
{
  canfd::Transport transport;
  const Status status = transport.open("lo");
  std::printf("down interface: %s\n", status.message.c_str());
  check(status.code == 305, "down interface is E305");
  check(status.message == "CAN interface 'lo' is down",
        "down interface message");
  check(!transport.is_open(), "down interface leaves the transport closed");
}

}  // namespace

int main()
{
  test_missing_interface();

  // Needs unprivileged user namespaces, which some hosts restrict
  if (unshare(CLONE_NEWUSER | CLONE_NEWNET) != 0) {
    std::printf("transport_open: down case skipped (no user namespace)\n");
    return g_failures == 0 ? kSkipped : 1;
  }
  test_down_interface();

  if (g_failures == 0) std::printf("transport_open: all checks passed\n");
  return g_failures == 0 ? 0 : 1;
}
