# SDK build & install

Build, test, and install the SDK, then confirm it works with the example. Complete
[Real-time kernel setup](04_real_time_kernel_setup.md) and
[CAN-FD setup](05_can_fd_setup.md) first.

## Contents

&nbsp;&nbsp;[**1. Install dependencies**](#1-install-dependencies)<br>
&nbsp;&nbsp;[**2. Build**](#2-build)<br>
&nbsp;&nbsp;[**3. Install and link**](#3-install-and-link)<br>
&nbsp;&nbsp;[**4. Test program**](#4-test-program)<br>
&nbsp;&nbsp;[**5. Uninstall**](#5-uninstall)

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
```

> [!IMPORTANT]
> **Two of the thumb actuators have a ball screw whose lead is either 1 mm or 2 mm, depending on
> the hardware generation.** The lead decides how many encoder counts one millimetre of actuator
> travel takes, so building for the wrong one makes those two actuators move **twice or half as
> far as asked.** Identify your hardware before you build, and if you cannot, ask on the
> [issue tracker](https://github.com/aidinrobotics/aidin-hand2-sdk/issues).

Run only the one below that matches your hand.

**a) 1 mm lead**

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   -- aidin_hand2: thumb lead = 1mm
#   -- aidin_hand2: kinematics = .../libaidin_hand2_kinematics.so.0.5.2
```

**b) 2 mm lead**

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAIDIN_HAND2_THUMB_LEAD=2mm
#   -- aidin_hand2: thumb lead = 2mm
#   -- aidin_hand2: kinematics = .../libaidin_hand2_kinematics.so.0.5.2-2mm
```

Confirm the selection from those two lines, then build.

```bash
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

What gets installed is the public headers, two shared libraries, and the CMake package config.

Follow **one** of the two below. `/usr/local` is the default; go to b) only when you cannot use
`sudo`.

**a) Install into `/usr/local` (recommended)**

`/usr/local` is both a default CMake search path and a path the loader keeps in its cache, so
your application never has to name a location.

```bash
sudo cmake --install cpp/build
sudo ldconfig
```

`ldconfig` registers the shared libraries in the loader cache. Skip it and the libraries are not
found at run time.

Your application's CMakeLists.txt is two lines. Always request the version: a different minor is
not compatible, and CMake skips the version check when none is requested.

```cmake
find_package(aidin_hand2 0.5 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

**b) Install into a user prefix**

Use this only when `sudo` is unavailable. `ldconfig` is not needed, but your application has to
name two paths instead.

```bash
cmake --install cpp/build --prefix "$HOME/.local"
```

Tell your application's CMakeLists.txt where the prefix is.

```cmake
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
find_package(aidin_hand2 0.5 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

If you install your application itself, CMake strips the RUNPATH at install time, so set
`INSTALL_RPATH` as well. Without it the installed application does not find the SDK.

```cmake
set_target_properties(your_app PROPERTIES INSTALL_RPATH "$ENV{HOME}/.local/lib")
```

> [!WARNING]
> Do **not** use a) and b) together. If you later move to `/usr/local`, remove the SDK from the
> earlier prefix first. With the SDK in two prefixes, which one `find_package` picks is not
> determined, and libraries from two different releases can end up in one process.
>
> ```bash
> rm -rf "$HOME/.local/lib/libaidin_hand2"* "$HOME/.local/lib/cmake/aidin_hand2" \
>        "$HOME/.local/include/aidin_hand2"
> ```
>
> Check what a prefix holds with the following.
>
> ```bash
> grep -m1 'set(PACKAGE_VERSION "' <prefix>/lib/cmake/aidin_hand2/aidin_hand2ConfigVersion.cmake
> ```

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

## 5. Uninstall

The build tree's `install_manifest.txt` lists what was installed. When the build tree is still
around, removing what that file names is the most accurate way.

```bash
xargs -a cpp/build/install_manifest.txt sudo rm -f
sudo rm -rf /usr/local/include/aidin_hand2 /usr/local/lib/cmake/aidin_hand2
sudo ldconfig
```

The manifest lists files only and leaves the directories behind, so those two go separately.
`ldconfig` then updates the loader cache.

Without the build tree, remove the paths directly. `<prefix>` is the prefix you installed into,
or `/usr/local` if you named none.

```bash
sudo rm -rf <prefix>/include/aidin_hand2 \
            <prefix>/lib/cmake/aidin_hand2 \
            <prefix>/lib/libaidin_hand2.so* \
            <prefix>/lib/libaidin_hand2_kinematics.so*
sudo ldconfig
```

Either way, confirm that nothing is left.

```bash
ldconfig -p | grep aidin_hand2        # should print nothing
ls <prefix>/lib/cmake/aidin_hand2     # should say No such file or directory
```

Repeat this for every prefix you have installed into. Clearing one and leaving another keeps the
remaining prefix on the search path, so libraries from two different releases can still end up in
one process.
