# CAN-FD setup

The SDK talks to the AIDIN Hand Gen2 over a USB CAN-FD adapter on Linux SocketCAN (1 Mbit/s
nominal, 5 Mbit/s data phase). This document covers two adapters, the **PEAK PCAN-USB FD** and the
**CANable 2.0 Pro** (candleLight firmware). Find the adapter you use, bring it up as CAN-FD, and
configure the SDK to use that interface.

## Contents

&nbsp;&nbsp;[**1. Bring up the interface**](#1-bring-up-the-interface)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Check the driver](#11-check-the-driver)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Find the interface](#12-find-the-interface)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Bring up that interface](#13-bring-up-that-interface)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.4 Verify the link](#14-verify-the-link)<br>
&nbsp;&nbsp;[**2. Use the interface in the SDK**](#2-use-the-interface-in-the-sdk)

## 1. Bring up the interface

Check the adapter's kernel driver, find the interface the AIDIN Hand Gen2 is connected to, bring
it up as CAN-FD, and tell which side each interface is from the frames coming in.

### 1.1 Check the driver

Without the driver module, connecting the adapter registers no CAN network device.

```bash
modinfo peak_usb | head -2       # PEAK PCAN-USB FD
modinfo gs_usb   | head -2       # CANable 2.0 Pro (candleLight)
```

A `filename` line for the driver you use means it is present. The Ubuntu 22.04 and 24.04 kernels
include both drivers as modules and load them on connection, so no separate installation is
required. `Module ... not found` means that kernel lacks the driver: on a stock Ubuntu
kernel run `sudo apt install linux-modules-extra-$(uname -r)`, and on a kernel you built yourself
enable `CONFIG_CAN_PEAK_USB` and `CONFIG_CAN_GS_USB` per [1.5 Configure the
kernel](04_real_time_kernel_setup.md#15-configure-the-kernel). A vendor kernel needs the same two
symbols, and you rebuild it by whatever procedure that vendor documents.

### 1.2 Find the interface

Check the CAN interfaces and the driver behind each.

```bash
ip -brief link show type can
for i in $(ip -o link show type can | awk -F': ' '{print $2}'); do
  echo "=== $i ==="; ethtool -i "$i" | grep -E '^(driver|bus-info):'
done
```

A `driver` of `peak_usb` is the PCAN-USB FD; `gs_usb` is the CANable 2.0 Pro. If the AIDIN Hand
Gen2 comes as a left/right pair, there are two such entries. Note each interface name (for example
`can0`, `can1`).

### 1.3 Bring up that interface

Bring the interfaces found in [1.2](#12-find-the-interface) up as CAN-FD at the AIDIN Hand Gen2's
bitrate. The side is not needed at this step. Bring up every interface you found, then determine
the side in [1.4](#14-verify-the-link). The example below has two, `can0` and `can1`.

`can0`:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can \
  bitrate 1000000 sample-point 0.875 sjw 10 \
  dbitrate 5000000 dsample-point 0.875 dsjw 2 \
  fd on restart-ms 100
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000
```

`can1`:

```bash
sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can1 type can \
  bitrate 1000000 sample-point 0.875 sjw 10 \
  dbitrate 5000000 dsample-point 0.875 dsjw 2 \
  fd on restart-ms 100
sudo ip link set can1 up
sudo ip link set can1 txqueuelen 1000
```

> [!NOTE]
> `restart-ms` is the delay (ms) before automatic recovery after a bus-off. If 0, auto-recovery is
> off, so a momentary power drop leaves the controller stuck in bus-off and the SDK's auto-reconnect
> cannot recover from it either.

### 1.4 Verify the link

Check one interface at a time. Set the interface to check (start with `can0`, then `can1`).

```bash
CAN=can0                       # interface to check
```

Check the link state.

```bash
ip -details -statistics link show "$CAN"
```

Confirm the lines and values below in the output.

```text
# <FD> = CAN-FD, ERROR-ACTIVE = normal bus, berr-counter 0 = error not climbing
can <FD> state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100
# nominal: bitrate 1 Mbit/s, sample-point 0.875
  bitrate 1000000 sample-point 0.875
# data phase: bitrate 5 Mbit/s, sample-point 0.875
  dbitrate 5000000 dsample-point 0.875
```

If `state` is `BUS-OFF`/`ERROR-PASSIVE`, or `berr-counter` keeps climbing, check the wiring,
termination, and bitrate.

When the AIDIN Hand Gen2 is connected and powered on, state frames come in periodically. Watch the
actual frames with `candump`.

```bash
candump "$CAN"
```

The first digit of the incoming IDs tells the side — **left starts with `0x2xx`** (`221` etc.),
**right with `0x1xx`** (`121` etc.).

If the AIDIN Hand Gen2 is not connected or is powered off, there are no frames and the RX packet
count stays 0 — the link configuration itself is fine. Connect it, power on, and check again.

Each state frame comes in at 500 Hz. To watch the per-ID rate in real time, run the following
(Ctrl-C to quit).

```bash
candump -t a "$CAN" | gawk '
BEGIN { win = 1.0; refresh = 1/60 }
{
  ts = $1; gsub(/[()]/, "", ts)
  id = $3
  q[id, ++tail[id]] = ts
  if (ts - drawn < refresh) next
  printf "\033[H\033[J"
  m = asorti(tail, ids)
  for (i = 1; i <= m; i++) {
    k = ids[i]
    while (head[k] < tail[k] && ts - q[k, head[k]+1] > win) delete q[k, ++head[k]]
    printf "0x%s: %d Hz\n", k, tail[k] - head[k]
  }
  drawn = ts
}'
```

It is normal when the four state lines (left `0x221`–`0x224`, right `0x121`–`0x124`) each read
around 500 Hz.

> [!NOTE]
> The boot order and hotplug decide names like `can0`/`can1`, so reconnecting an adapter or
> rebooting can change which side maps to which name. Once the mapping changes, check it again
> with `candump`.

## 2. Use the interface in the SDK

Put the interface name you confirmed in section 1 and the side into
[`HandConfig`](08_cpp_api_reference/types_config.md#handconfig).

```cpp
ah2::HandConfig config{"can0", ah2::HandSide::Left};    // interface name, side
```

Next, build the SDK in [SDK build & install](06_sdk_build_and_install.md).
