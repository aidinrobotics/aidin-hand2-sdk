# Third-Party Notices

The AIDIN Hand Gen2 SDK is licensed under the Apache License 2.0; see [LICENSE](LICENSE)
and [NOTICE](NOTICE). It also includes and links third-party software. This file lists each
component, the license it is provided under, and where its source can be obtained. Full
license texts are in [licenses/](licenses).

Components reach a user of this SDK in two different ways, and the difference decides what
this repository has to carry.

- **Compiled into a binary distributed here.** Eigen is a header-only library. The kinematics
  library shipped as `cpp/prebuilt/<arch>/libaidin_hand2_kinematics.so.<version>` is built from a
  source file that includes Eigen headers, so Eigen-derived code is part of that binary.
- **Installed by the user and linked at build time.** spdlog and fmt are taken from the
  distribution's packages. This repository does not redistribute either one.

## Components

| Component | Version | License | How it is used |
|---|---|---|---|
| [Eigen](https://eigen.tuxfamily.org) | 3.4.0 | MPL-2.0, with BSD-3-Clause parts | Compiled into `libaidin_hand2_kinematics.so.<version>` |
| [spdlog](https://github.com/gabime/spdlog) | ≥ 1.9 | MIT | Linked into `libaidin_hand2.so` |
| [fmt](https://github.com/fmtlib/fmt) | as packaged with spdlog | MIT with an optional exception | Linked transitively through spdlog |
| Linux userspace API headers | kernel headers of the build host | BSD-3-Clause, or GPL-2.0 WITH Linux-syscall-note | Included when the SDK is compiled |

## Eigen

Eigen is licensed under the Mozilla Public License 2.0, whose text is in
[licenses/MPL-2.0.txt](licenses/MPL-2.0.txt). Eigen is not modified in any way; the headers
are used as the distribution installs them.

`libaidin_hand2_kinematics.so.<version>` is distributed in this repository as an Executable Form of
software covered by the MPL. Section 3.2 of the MPL requires that recipients be informed of
this and be able to obtain the Source Code Form of the covered files. The covered files are
the unmodified Eigen headers, and the source is available from the Eigen project:

- Release archives: https://eigen.tuxfamily.org
- Repository: https://gitlab.com/libeigen/eigen

The MPL applies to the Eigen files themselves and does not extend to the AIDIN ROBOTICS
source in this repository, which stays under the Apache License 2.0. Section 3.3 of the MPL
permits this combination as long as the covered files remain available under the MPL, which
the links above satisfy.

Only the `Eigen/Dense` and `Eigen/Geometry` modules are used. The Eigen files that carry
LGPL terms — `Eigen/src/SparseCholesky/SimplicialCholesky.h`,
`Eigen/src/OrderingMethods/Amd.h`, `unsupported/Eigen/src/IterativeSolvers/*` — belong to the
sparse solver modules and are not reachable from those two headers, so no LGPL obligation
arises from this SDK.

Three parts of the Eigen headers that the SDK does compile carry additional or different
terms. Their notices are reproduced in
[licenses/BSD-3-Clause-Eigen.txt](licenses/BSD-3-Clause-Eigen.txt):

| File | Terms | Copyright |
|---|---|---|
| `Eigen/src/Core/util/MKL_support.h` | BSD-3-Clause | 2011 Intel Corporation |
| `Eigen/src/Geometry/AlignedBox.h`, `transform()` only | BSD-3-Clause | 2011-2014 Willow Garage, Inc.; 2014-2015 Open Source Robotics Foundation |
| `Eigen/src/LU/arch/InverseSize4.h` | MPL-2.0, retaining an Intel attribution notice | 2001 Intel Corporation |

## spdlog

spdlog provides the logging backend used by `src/logging/logging.cpp`. It is linked
privately, so it does not appear in the SDK's public headers and is not propagated to
consumers. It is licensed under the MIT license, whose text is in
[licenses/MIT-spdlog.txt](licenses/MIT-spdlog.txt).

The SDK requires spdlog 1.9 or newer and expects it to be installed from the distribution's
packages, as [README.md](README.md) describes. Source is available from
https://github.com/gabime/spdlog.

## fmt

spdlog uses the fmt library for formatting. Whether fmt is a separate shared library or is
bundled inside spdlog depends on how the distribution built spdlog: Ubuntu builds it with
`SPDLOG_FMT_EXTERNAL`, so `libspdlog.so` links `libfmt.so` and fmt reaches the SDK as a
transitive dependency.

fmt is licensed under the MIT license with an optional exception that waives the notice
requirement for portions embedded in object form. The full text is in
[licenses/MIT-fmt.txt](licenses/MIT-fmt.txt), and source is available from
https://github.com/fmtlib/fmt.

## Linux userspace API headers

The CAN-FD transport is built on SocketCAN and includes the kernel's userspace API headers.
These headers declare the socket, frame and netlink structures the kernel exposes; the SDK
does not link any kernel code.

| Header | SPDX identifier |
|---|---|
| `linux/can.h`, `linux/can/error.h`, `linux/can/raw.h` | `(GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause` |
| `linux/can/netlink.h` | `GPL-2.0-only WITH Linux-syscall-note` |
| `linux/netlink.h`, `linux/rtnetlink.h` | `GPL-2.0 WITH Linux-syscall-note` |

For the three dual-licensed CAN headers the SDK takes the BSD-3-Clause option, so no GPL
term applies to them. The notice is reproduced in
[licenses/BSD-3-Clause-SocketCAN.txt](licenses/BSD-3-Clause-SocketCAN.txt).

The remaining three headers are available only under GPL-2.0 with the `Linux-syscall-note`
exception. The exception states that the kernel's copyright does not cover user programs
that use kernel services through normal system calls, and that such use is not a derived
work. The SDK is such a user program, so the GPL does not extend to it. The exception text
is in the kernel tree at `LICENSES/exceptions/Linux-syscall-note`, and the kernel source is
available from https://www.kernel.org.

## can-utils

[README.md](README.md) recommends installing can-utils for bringing up and inspecting a CAN
interface. The SDK never invokes it: interface configuration is done through netlink from
`src/hand_core/comms/canfd/interface_setup.cpp`, and no code path starts an external
process. can-utils is therefore a separate program a user may choose to run, not a
dependency of this software, and its license imposes no obligation on this repository.
