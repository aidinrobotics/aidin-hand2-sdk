#include "hand_core/comms/canfd/transport.hpp"

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

namespace aidin_hand2::canfd
{

// ---------------------------------- Socket ----------------------------------

Transport::~Transport()
{
  close();
}

Status Transport::open(const std::string& interface)
{
  // 1) Leave an existing socket alone, since every reopen in the SDK goes through close() first
  if (fd_ >= 0) {
    return {error_code::kInternalState, "CAN transport already open on '" + interface + "'"};
  }

  // 2) Check the name before anything CAN specific, so a wrong name is reported as such
  if (if_nametoindex(interface.c_str()) == 0) {
    return open_failure(interface, errno);
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

  // 5) Resolve the interface name to an index, kept apart because ifr_flags shares its storage
  // An interface removed since step 2 fails here with ENODEV, reported as not found
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    const int saved_errno = errno;
    ::close(fd);
    return open_failure(interface, saved_errno);
  }
  const int interface_index = ifr.ifr_ifindex;

  // 6) A down interface binds without error and then stays silent, so it is refused here
  if (::ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) {
    const int saved_errno = errno;
    ::close(fd);
    return open_failure(interface, saved_errno);
  }
  if ((ifr.ifr_flags & IFF_UP) == 0) {
    ::close(fd);
    return {error_code::kInterfaceDown, "CAN interface '" + interface + "' is down"};
  }

  // 7) Bind, closing only the local fd on failure so this object stays closed
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = interface_index;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const int saved_errno = errno;
    ::close(fd);
    return open_failure(interface, saved_errno);
  }

  fd_ = fd;
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

// --------------------------------- Helpers ----------------------------------

// Every case is an E3xx, InterfaceUnavailable
Status Transport::open_failure(const std::string& interface, int err)
{
  switch (err) {
    case ENODEV:
      return {error_code::kInterfaceNotFound, "CAN interface '" + interface + "' not found"};
    // ENETDOWN cannot reach here, because open() refuses a down interface before binding
    // A link that drops later shows up through send() and last_send_errno()
    case EPERM:
    case EACCES:
      return {error_code::kSocketPermissionDenied,
              "permission denied opening CAN socket on '" + interface + "'"};
    case EAFNOSUPPORT:
    case EPROTONOSUPPORT:
      return {error_code::kCanNotSupported, "CAN not supported by the kernel"};
    case ENOPROTOOPT:
      // setsockopt(CAN_RAW_FD_FRAMES) refused the option itself
      return {error_code::kCanFdNotConfigured,
              "kernel or interface '" + interface + "' is not configured for CAN-FD (CAN_RAW_FD_FRAMES)"};
    default:
      // Falls back to the standard description, which is thread safe
      return {error_code::kInterfaceOpenFailed,
              "CAN interface '" + interface + "' could not be opened (" + std::generic_category().message(err) + ")"};
  }
}

}  // namespace aidin_hand2::canfd
