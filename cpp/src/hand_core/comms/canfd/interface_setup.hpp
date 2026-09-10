#pragma once

#include <string>

#include "types/status.hpp"

// Brings a CAN interface up through the kernel rtnetlink, without an external library
// The bitrates are this hand's protocol constants, not a user choice, so the SDK sets them

namespace aidin_hand2::canfd
{

// Configures and brings up an interface that is down
//   bitrate 1M (sp 0.875, sjw 10), dbitrate 5M (dsp 0.875, dsjw 2), fd on
//   restart-ms 100, txqueuelen 1000
// An interface that is already up is left alone, since the kernel refuses bittiming changes there
// Without CAP_NET_ADMIN it fails with a Status whose message is the manual commands
[[nodiscard]] Status ensure_interface_up(const std::string& interface_name);

}  // namespace aidin_hand2::canfd
