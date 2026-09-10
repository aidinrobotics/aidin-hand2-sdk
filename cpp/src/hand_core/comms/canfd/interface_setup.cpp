#include "hand_core/comms/canfd/interface_setup.hpp"

#include "logging/log.hpp"

#include <linux/can.h>
#include <linux/can/netlink.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>

// Does over rtnetlink what `ip link set can0 type can ... && ip link set can0 up` does
// Message layout, following the kernel UAPI in linux/can/netlink.h
//   RTM_NEWLINK + ifinfomsg + IFLA_TXQLEN
//     + IFLA_LINKINFO { IFLA_INFO_KIND="can",
//                       IFLA_INFO_DATA { IFLA_CAN_BITTIMING, IFLA_CAN_DATA_BITTIMING,
//                                        IFLA_CAN_CTRLMODE(FD), IFLA_CAN_RESTART_MS } }
// Configuring and bringing up are separate messages, because the kernel only configures a down link

namespace aidin_hand2::canfd
{

namespace
{

// This hand's CAN protocol constants
// Sample points are per mille, as the kernel expects: 875 is 87.5%

// Arbitration phase
constexpr std::uint32_t kNominalBitrate             = 1000000;
constexpr std::uint32_t kNominalSamplePointPerMille = 875;
constexpr std::uint32_t kNominalSyncJumpWidth       = 10;

// FD data phase
constexpr std::uint32_t kDataBitrate                = 5000000;
constexpr std::uint32_t kDataSamplePointPerMille    = 875;
constexpr std::uint32_t kDataSyncJumpWidth          = 2;

// Restarts the controller automatically after bus-off
constexpr std::uint32_t kBusOffRestartMilliseconds  = 100;

constexpr std::uint32_t kTransmitQueueLength        = 1000;

struct NetlinkRequest {
  nlmsghdr header;
  ifinfomsg interface_info;
  char attribute_buffer[512];
};

// Appends one rtattr to the request
void add_attribute(NetlinkRequest& request, unsigned short type, const void* data, std::size_t length);

rtattr* begin_nested_attribute(NetlinkRequest& request, unsigned short type);
void end_nested_attribute(NetlinkRequest& request, rtattr* nested_attribute);

// Sends the request and waits for the kernel ACK, returning its errno
int send_and_acknowledge(int netlink_socket, NetlinkRequest& request);

NetlinkRequest make_newlink_request(unsigned int interface_index);

// Builds a Status whose message is the manual ip link commands
Status manual_bringup_guidance(const std::string& interface_name);

bool is_interface_up(const std::string& interface_name);

Status configure_protocol_constants(int netlink_socket, unsigned int interface_index,
                                    const std::string& interface_name);
Status bring_link_up(int netlink_socket, unsigned int interface_index,
                     const std::string& interface_name);

}  // namespace

Status ensure_interface_up(const std::string& interface_name)
{
  const unsigned int interface_index = if_nametoindex(interface_name.c_str());
  if (interface_index == 0) {
    return {ErrorCode::InterfaceUnavailable,
            "CAN interface '" + interface_name + "' not found (check the name with `ip link`)"};
  }

  // Debug rather than info, because auto-reconnect retries every few hundred ms and the
  // interface stays up regardless of hand power, so this line would flood the log
  if (is_interface_up(interface_name)) {
    log_debug("CAN interface '" + interface_name + "' already up — leaving its configuration untouched");
    return {};
  }

  const int netlink_socket = ::socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
  if (netlink_socket < 0) {
    return manual_bringup_guidance(interface_name);
  }

  Status status = configure_protocol_constants(netlink_socket, interface_index, interface_name);
  if (status.ok()) {
    status = bring_link_up(netlink_socket, interface_index, interface_name);
  }
  ::close(netlink_socket);

  if (status.ok()) {
    // Records what was applied, for diagnosing a hand in the field
    log_info("CAN interface '" + interface_name + "' was down; configured and brought up"
             " (bitrate " + std::to_string(kNominalBitrate) +
             ", dbitrate " + std::to_string(kDataBitrate) +
             ", fd on, restart-ms " + std::to_string(kBusOffRestartMilliseconds) + ")");
  }
  return status;
}

// --------------------------------- Helpers ----------------------------------

namespace
{

// Appends one rtattr to the request
void add_attribute(NetlinkRequest& request, unsigned short type, const void* data, std::size_t length)
{
  rtattr* attribute = reinterpret_cast<rtattr*>(
      reinterpret_cast<char*>(&request) + NLMSG_ALIGN(request.header.nlmsg_len));
  attribute->rta_type = type;
  attribute->rta_len  = static_cast<unsigned short>(RTA_LENGTH(length));
  if (length > 0) {
    std::memcpy(RTA_DATA(attribute), data, length);
  }
  request.header.nlmsg_len = NLMSG_ALIGN(request.header.nlmsg_len) + RTA_ALIGN(attribute->rta_len);
}

// The returned pointer is what end_nested_attribute closes the length with
rtattr* begin_nested_attribute(NetlinkRequest& request, unsigned short type)
{
  rtattr* nested_attribute = reinterpret_cast<rtattr*>(
      reinterpret_cast<char*>(&request) + NLMSG_ALIGN(request.header.nlmsg_len));
  add_attribute(request, type, nullptr, 0);
  return nested_attribute;
}

void end_nested_attribute(NetlinkRequest& request, rtattr* nested_attribute)
{
  nested_attribute->rta_len = static_cast<unsigned short>(
      reinterpret_cast<char*>(&request) + NLMSG_ALIGN(request.header.nlmsg_len) -
      reinterpret_cast<char*>(nested_attribute));
}

// Returns 0 on the kernel ACK, or -errno
int send_and_acknowledge(int netlink_socket, NetlinkRequest& request)
{
  if (::send(netlink_socket, &request, request.header.nlmsg_len, 0) < 0) {
    return -errno;
  }
  char reply_buffer[4096];
  const ssize_t received = ::recv(netlink_socket, reply_buffer, sizeof(reply_buffer), 0);
  if (received < 0) {
    return -errno;
  }
  const nlmsghdr* reply = reinterpret_cast<const nlmsghdr*>(reply_buffer);
  // A reply shorter than an nlmsghdr would mean reading uninitialised buffer
  if (!NLMSG_OK(reply, static_cast<unsigned int>(received))) {
    return -EBADMSG;
  }
  if (reply->nlmsg_type == NLMSG_ERROR) {
    // 0 is the ACK
    return reinterpret_cast<const nlmsgerr*>(NLMSG_DATA(reply))->error;
  }
  // NLM_F_ACK always answers with NLMSG_ERROR, so anything else is a failure
  return -EBADMSG;
}

// Fills the RTM_NEWLINK header both messages share
NetlinkRequest make_newlink_request(unsigned int interface_index)
{
  NetlinkRequest request{};
  request.header.nlmsg_len   = NLMSG_LENGTH(sizeof(ifinfomsg));
  request.header.nlmsg_type  = RTM_NEWLINK;
  request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
  request.interface_info.ifi_family = AF_UNSPEC;
  request.interface_info.ifi_index  = static_cast<int>(interface_index);
  return request;
}

// Used when the process lacks CAP_NET_ADMIN
Status manual_bringup_guidance(const std::string& interface_name)
{
  return {ErrorCode::InterfaceUnavailable,
          "CAN interface '" + interface_name + "' is down and cannot be configured " +
          "(needs CAP_NET_ADMIN). Bring it up manually:\n" +
          "  sudo ip link set " + interface_name + " type can bitrate 1000000 sample-point 0.875 sjw 10 " +
          "dbitrate 5000000 dsample-point 0.875 dsjw 2 fd on restart-ms 100\n" +
          "  sudo ip link set " + interface_name + " up"};
}

// A failed query counts as down, so the caller goes on to configure
bool is_interface_up(const std::string& interface_name)
{
  const int probe_socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (probe_socket < 0) {
    return false;
  }
  ifreq interface_request{};
  std::strncpy(interface_request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  const bool got_flags = ::ioctl(probe_socket, SIOCGIFFLAGS, &interface_request) == 0;
  ::close(probe_socket);
  return got_flags && (interface_request.ifr_flags & IFF_UP);
}

// The kernel only accepts these while the link is down
Status configure_protocol_constants(int netlink_socket, unsigned int interface_index,
                                    const std::string& interface_name)
{
  NetlinkRequest request = make_newlink_request(interface_index);
  add_attribute(request, IFLA_TXQLEN, &kTransmitQueueLength, sizeof(kTransmitQueueLength));

  rtattr* link_info = begin_nested_attribute(request, IFLA_LINKINFO);
  add_attribute(request, IFLA_INFO_KIND, "can", 3);
  rtattr* info_data = begin_nested_attribute(request, IFLA_INFO_DATA);

  can_bittiming nominal_timing{};
  nominal_timing.bitrate      = kNominalBitrate;
  nominal_timing.sample_point = kNominalSamplePointPerMille;
  nominal_timing.sjw          = kNominalSyncJumpWidth;
  add_attribute(request, IFLA_CAN_BITTIMING, &nominal_timing, sizeof(nominal_timing));

  can_bittiming data_timing{};
  data_timing.bitrate      = kDataBitrate;
  data_timing.sample_point = kDataSamplePointPerMille;
  data_timing.sjw          = kDataSyncJumpWidth;
  add_attribute(request, IFLA_CAN_DATA_BITTIMING, &data_timing, sizeof(data_timing));

  can_ctrlmode control_mode{};
  control_mode.mask  = CAN_CTRLMODE_FD;
  control_mode.flags = CAN_CTRLMODE_FD;
  add_attribute(request, IFLA_CAN_CTRLMODE, &control_mode, sizeof(control_mode));

  add_attribute(request, IFLA_CAN_RESTART_MS, &kBusOffRestartMilliseconds, sizeof(kBusOffRestartMilliseconds));

  end_nested_attribute(request, info_data);
  end_nested_attribute(request, link_info);

  const int result = send_and_acknowledge(netlink_socket, request);
  if (result < 0) {
    return (result == -EPERM || result == -EACCES)
               ? manual_bringup_guidance(interface_name)
               : Status{ErrorCode::InterfaceUnavailable,
                        "failed to configure CAN interface '" + interface_name +
                            "': " + std::strerror(-result)};
  }
  return {};
}

// The same as `ip link set <name> up`
Status bring_link_up(int netlink_socket, unsigned int interface_index,
                     const std::string& interface_name)
{
  NetlinkRequest request = make_newlink_request(interface_index);
  request.interface_info.ifi_flags  = IFF_UP;
  request.interface_info.ifi_change = IFF_UP;

  const int result = send_and_acknowledge(netlink_socket, request);
  if (result < 0) {
    return (result == -EPERM || result == -EACCES)
               ? manual_bringup_guidance(interface_name)
               : Status{ErrorCode::InterfaceUnavailable,
                        "failed to bring up CAN interface '" + interface_name +
                            "': " + std::strerror(-result)};
  }
  return {};
}

}  // namespace

}  // namespace aidin_hand2::canfd
