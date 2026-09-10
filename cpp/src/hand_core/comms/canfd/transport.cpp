#include "hand_core/comms/canfd/transport.hpp"

#include "hand_core/comms/canfd/interface_setup.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <system_error>
#include <utility>

namespace aidin_hand2::canfd
{

// ---------------------------------- Socket ----------------------------------

Transport::~Transport()
{
  close();
}

Status Transport::open(const std::string& interface)
{
  // 1) Leave an existing socket alone, since reopening goes through close()
  if (fd_ >= 0) {
    return {ErrorCode::WrongCallOrder, "CAN transport already open: " + interface + " — close it before reopening"};
  }

  // 2) Bring the interface up with the protocol constants, doing nothing if it is already up
  Status interface_ready = ensure_interface_up(interface);
  if (!interface_ready.ok()) {
    return interface_ready;
  }

  // 3) Create the CAN raw socket
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return open_failure(interface, errno);
  }

  // 4) Switch the socket to the CAN-FD frame format
  const int enable_canfd = 1;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &enable_canfd, sizeof(enable_canfd)) < 0) {
    const int saved_errno = errno;
    ::close(fd);
    return open_failure(interface, saved_errno);
  }

  // 5) Resolve the interface name to an index
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    const int saved_errno = errno;
    ::close(fd);
    return open_failure(interface, saved_errno);
  }

  // 6) Bind, closing only the local fd on failure so this object stays closed
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const int saved_errno = errno;
    ::close(fd);
    return open_failure(interface, saved_errno);
  }

  fd_ = fd;
  bound_interface_index_ = ifr.ifr_ifindex;
  return {};
}

void Transport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ---------------------------------- Frames ----------------------------------

bool Transport::recv(canfd_frame& frame)
{
  if (fd_ < 0) {
    return false;
  }
  // The socket is in blocking mode, so MSG_DONTWAIT is what makes this non-blocking
  const ssize_t received_byte_count = ::recv(fd_, &frame, sizeof(canfd_frame), MSG_DONTWAIT);

  // CANFD_MTU is a CAN-FD frame and CAN_MTU is a classic or error frame, anything else is nothing
  return received_byte_count == static_cast<ssize_t>(CANFD_MTU) ||
         received_byte_count == static_cast<ssize_t>(CAN_MTU);
}

bool Transport::send(const canfd_frame& frame)
{
  if (fd_ < 0) {
    return false;
  }
  // A full TX queue returns immediately instead of blocking the RT thread
  const ssize_t n = ::send(fd_, &frame, sizeof(canfd_frame), MSG_DONTWAIT);
  if (n == static_cast<ssize_t>(sizeof(canfd_frame))) {
    return true;
  }

  // A short write leaves errno unset, so record 0 rather than a stale value
  last_send_errno_ = (n < 0) ? errno : 0;
  return false;
}

// --------------------------------- Filters ----------------------------------

bool Transport::set_id_filters(const std::vector<can_filter>& filters)
{
  if (fd_ < 0) {
    return false;
  }
  // The kernel reads a null pointer as removing every filter
  const void* data = filters.empty() ? nullptr : filters.data();
  const socklen_t len = static_cast<socklen_t>(filters.size() * sizeof(can_filter));
  return ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, data, len) == 0;
}

bool Transport::set_error_mask(can_err_mask_t mask)
{
  if (fd_ < 0) {
    return false;
  }
  return ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask, sizeof(mask)) == 0;
}

// --------------------------------- Helpers ----------------------------------

// Every case is InterfaceUnavailable, only the message differs
Status Transport::open_failure(const std::string& interface, int err)
{
  std::string message;
  switch (err) {
    case ENODEV:
      message = "CAN interface '" + interface +
                "' not found (check the name with `ip link`, and that it exists)";
      break;
    // ENETDOWN cannot reach here, because ensure_interface_up() ran first
    // A link that drops later shows up through send() and last_send_errno()
    case EPERM:
    case EACCES:
      message = "permission denied opening CAN socket on '" + interface +
                "' (need root or CAP_NET_RAW)";
      break;
    case EAFNOSUPPORT:
    case EPROTONOSUPPORT:
      message = "CAN not supported by the kernel (try `modprobe can can_raw`)";
      break;
    case ENOPROTOOPT:
      // setsockopt(CAN_RAW_FD_FRAMES) refused the option itself
      message = "kernel or interface '" + interface +
                "' is not configured for CAN-FD (CAN_RAW_FD_FRAMES);"
                " check with `ip -details link show " + interface + "`";
      break;
    default:
      // Falls back to the standard description, which is thread safe
      message = "failed to open CAN interface '" + interface +
                "': " + std::generic_category().message(err);
      break;
  }
  return {ErrorCode::InterfaceUnavailable, std::move(message)};
}

}  // namespace aidin_hand2::canfd
