# SDK build & install

Build, test, and install the SDK, then confirm it works with the example. Complete
[Real-time kernel setup](04_real_time_kernel_setup.md) and
[CAN-FD setup](05_can_fd_setup.md) first.

## Contents

&nbsp;&nbsp;[**1. Install dependencies**](#1-install-dependencies)<br>
&nbsp;&nbsp;[**2. Build**](#2-build)<br>
&nbsp;&nbsp;[**3. Install and link**](#3-install-and-link)<br>
&nbsp;&nbsp;[**4. Test program**](#4-test-program)

## 1. Install dependencies

The SDK supports Ubuntu 22.04 / 24.04. The apt packages of both distributions meet the required
versions, so you do not need to pin versions.

```bash
sudo apt update
sudo apt install -y build-essential cmake libspdlog-dev can-utils
```

The minimum requirements are spdlog 1.9 and a C++17 compiler. Only the kinematics implementation
uses Eigen, and that part ships prebuilt, so you do not need to install it.

## 2. Build

Run all following commands from the SDK repo root.

```bash
cd aidin-hand2-sdk
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build cpp/build -j"$(nproc)"
```

Optionally, run the code-logic tests.

```bash
ctest --test-dir cpp/build --output-on-failure
```

If they fail, don't proceed to install. Paste the output log and the environment info below into
an [issue](https://github.com/aidinrobotics/aidin-hand2-sdk/issues) as-is — no extra explanation
needed.

```bash
lsb_release -d; uname -srm; gcc --version | head -1; cmake --version | head -1
```

## 3. Install and link

Install the public headers, two shared libraries, and the CMake package config into `/usr/local`.
The loader has to find the shared libraries, so refresh its cache with `ldconfig` afterwards.

```bash
sudo cmake --install cpp/build
sudo ldconfig
```

To avoid touching system paths, set a prefix: `cmake --install cpp/build --prefix <prefix>`.
`ldconfig` is not needed then, because CMake puts a RUNPATH into your application.

Add the following to your application's CMakeLists.txt to link the SDK. Always request the
version: a different minor is not compatible, and CMake skips the version check when none is
requested.

```cmake
list(APPEND CMAKE_PREFIX_PATH "<prefix>")   # only if you installed outside /usr/local
find_package(aidin_hand2 0.5 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

If you install your application itself, CMake strips that RUNPATH at install time. Set
`INSTALL_RPATH` on your application when you used a prefix other than `/usr/local`.

## 4. Test program

Building in section 2 produces the example executable `cpp/build/basic_control`. It confirms
operation in this order: connect → homing → move each joint one by one from index 0 to 50° and
back (abduction excluded) → stop.

First, use `candump` to check which hand is connected to which interface. With the hand powered
on, state frames come up, and **the left hand uses `0x2xx` IDs (for example `0x221`), the right hand
uses `0x1xx` (for example `0x121`)**.

```bash
candump -n 20 can0        # the interface to check. the first digit of the incoming IDs tells the hand
#   0x2xx observed → left,  0x1xx observed → right
```

Pass the hand and interface you confirmed as arguments.

```bash
./cpp/build/basic_control left can0      # [right|left] [interface] — the values confirmed above
```

A normal run produces the following output.

```text
<!-- TODO: replace with the actual normal log from running basic_control on real hardware -->
```
