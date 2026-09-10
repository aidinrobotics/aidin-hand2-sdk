# Real-time kernel setup

Scheduling the SDK's 500 Hz control and communication loop in real time needs a `PREEMPT_RT`
kernel, plus permission for a non-root user to run at real-time priority (`SCHED_FIFO`) and lock
memory. This document builds that kernel from source on Ubuntu (x86_64 / arm64) and grants the
permission.

## Contents

&nbsp;&nbsp;[**1. Build the RT kernel**](#1-build-the-rt-kernel)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Install build dependencies](#11-install-build-dependencies)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Get the kernel source](#12-get-the-kernel-source)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Verify signatures](#13-verify-signatures)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.4 Apply the RT patch](#14-apply-the-rt-patch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.5 Configure the kernel](#15-configure-the-kernel)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.6 Build & install](#16-build--install)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.7 Boot into the kernel](#17-boot-into-the-kernel)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.8 Verify the kernel](#18-verify-the-kernel)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.9 Remove the kernel](#19-remove-the-kernel)<br>
&nbsp;&nbsp;[**2. Real-time settings**](#2-real-time-settings)

## 1. Build the RT kernel

### 1.1 Install build dependencies

```bash
sudo apt update
sudo apt install -y build-essential bc bison flex gawk \
  libssl-dev libelf-dev libncurses-dev dwarves \
  fakeroot rsync cpio kmod zstd gnupg dirmngr
```

> [!NOTE]
> This list targets kernel 6.12. Build dependencies vary with the kernel version and `.config`,
> so for the version you install, check the required packages in the source tree's
> `Documentation/process/changes.rst`.


### 1.2 Get the kernel source

The example uses the LTS kernel 6.12 (6.12.100 / rt20). Check which kernels keep an RT patch at
[PREEMPT_RT versions](https://wiki.linuxfoundation.org/realtime/preempt_rt_versions), confirm that
[kernel source](https://cdn.kernel.org/pub/linux/kernel/v6.x/) has `linux-6.12.100.tar.xz` and
[RT patch](https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.12/) has
`patch-6.12.100-rt20.patch.xz`, then set the variables below.

```bash
ver=6.12.100                   # ← set this: kernel version
rt=rt20                        # ← set this: RT revision for that ver (rtN)
major="${ver%%.*}"             # 6    (source directory v6.x)
base="${ver%.*}"               # 6.12 (RT patch directory major.minor)
kroot=https://cdn.kernel.org/pub/linux/kernel
```

Download the source and patch with their signatures.

```bash
curl -LO "${kroot}/v${major}.x/linux-${ver}.tar.xz"
curl -LO "${kroot}/v${major}.x/linux-${ver}.tar.sign"
curl -LO "${kroot}/projects/rt/${base}/patch-${ver}-${rt}.patch.xz"
curl -LO "${kroot}/projects/rt/${base}/patch-${ver}-${rt}.patch.sign"
```

### 1.3 Verify signatures

Run the check first without the key, and read the `RSA key <FINGERPRINT>` from the output. Since
you have not imported the key yet, `No public key` is normal here.

```bash
xz -cd "linux-${ver}.tar.xz" | gpg --verify "linux-${ver}.tar.sign" -
```

Match that `<FINGERPRINT>` against
[kernel.org signatures](https://www.kernel.org/signature.html) (stable = Greg KH or Sasha Levin,
mainline = Linus). If it matches, import that signer's key.

```bash
gpg --locate-keys gregkh@kernel.org       # stable — Greg KH
# gpg --locate-keys sashal@kernel.org     # stable — Sasha Levin
# gpg --locate-keys torvalds@kernel.org   # mainline — Linus
```

With the key imported, re-verify the source and patch signatures. It passes when `Good signature`
appears and the fingerprint matches the one above (the trailing `WARNING: not certified` is fine —
you compared the fingerprint directly).

```bash
xz -cd "linux-${ver}.tar.xz"         | gpg --verify "linux-${ver}.tar.sign" -
xz -cd "patch-${ver}-${rt}.patch.xz" | gpg --verify "patch-${ver}-${rt}.patch.sign" -
```

### 1.4 Apply the RT patch

Once the signatures verify, extract the source tarball and apply the RT patch.

```bash
tar xf "linux-${ver}.tar.xz"
cd "linux-${ver}"
xzcat "../patch-${ver}-${rt}.patch.xz" | patch -p1
```

### 1.5 Configure the kernel

Copy the config of the running kernel (`/boot/config-$(uname -r)`) as the baseline — the current
hardware setup, including the CAN driver, carries over. Then enable real-time preemption, clear
the Canonical signing-certificate paths that don't exist locally (leaving them set makes the build
fail on a missing file), and fill in new options with `olddefconfig`.

> [!NOTE]
> `cp` copies the config of the kernel you are **currently booted into**. If you booted a kernel
> you built yourself, you inherit its config instead of Ubuntu's default — make sure you start
> from the config you intend.

```bash
cp /boot/config-"$(uname -r)" .config
scripts/config --enable  CONFIG_PREEMPT_RT
scripts/config --set-str CONFIG_SYSTEM_TRUSTED_KEYS ""      # clear signing certificate paths
scripts/config --set-str CONFIG_SYSTEM_REVOCATION_KEYS ""
make olddefconfig                                          # apply the changes, default the new options
```

Confirm the CAN driver survived in the config.

```bash
grep -E 'CONFIG_CAN=|CONFIG_CAN_DEV|CONFIG_CAN_RAW|CONFIG_CAN_PEAK_USB|CONFIG_CAN_GS_USB' .config
```

If missing, enable the SocketCAN core and the adapter drivers, then re-run `olddefconfig`. We use
the PEAK PCAN-USB FD (driver `peak_usb`) and the candleLight-based CANable Pro 2.0 (driver
`gs_usb`), so enable both drivers.

```bash
scripts/config --module CONFIG_CAN
scripts/config --module CONFIG_CAN_DEV
scripts/config --module CONFIG_CAN_RAW
scripts/config --module CONFIG_CAN_PEAK_USB   # ← PCAN-USB FD
scripts/config --module CONFIG_CAN_GS_USB     # ← CANable Pro 2.0 (candleLight)
make olddefconfig
```

For a different USB-CAN adapter, add its driver line as well — `CONFIG_CAN_KVASER_USB` (Kvaser),
`CONFIG_CAN_8DEV_USB` (8devices), `CONFIG_CAN_ESD_USB` (esd), and so on; the full list is in the
kernel source's `drivers/net/can/usb/Kconfig`. Built as modules (`--module`), unused drivers aren't
loaded, so if unsure you can enable several at once.

### 1.6 Build & install

```bash
make -j$(nproc) bindeb-pkg
sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg -i ../linux-image-*.deb ../linux-headers-*.deb
```

`IGNORE_PREEMPT_RT_PRESENCE=1` bypasses the check where a DKMS module (for example the NVIDIA
driver) refuses to build once it detects an RT kernel. It's harmless if you have no DKMS modules.

### 1.7 Boot into the kernel

The RT kernel you just installed may not be the default boot kernel. After rebooting, pick the
version you just built under GRUB's **Advanced options for Ubuntu** — rebooting straight through
lands on the old kernel, which won't pass the next section's check.

```bash
sudo reboot
```

Once verified and you plan to keep this kernel, pin it as the default boot kernel so you don't
have to pick it in GRUB every time (set `GRUB_DEFAULT` in `/etc/default/grub`, then
`sudo update-grub`).

> [!WARNING]
> With Secure Boot enabled, the system refuses an unsigned self-built kernel at boot ("Invalid
> Signature"). Disable Secure Boot in the UEFI/BIOS menu and boot again.

### 1.8 Verify the kernel

After reboot, confirm the RT kernel is running:

```bash
uname -a                   # the version you built, marked PREEMPT_RT
cat /sys/kernel/realtime   # 1 on an RT kernel (absent if not RT)
```

### 1.9 Remove the kernel

To roll back, remove the installed kernel packages and reboot (find the version with `uname -r`
or `dpkg -l 'linux-image-*'`).

```bash
sudo dpkg -r linux-image-"${ver}"* linux-headers-"${ver}"*
sudo reboot
```

## 2. Real-time settings

A non-root user needs permission to use `SCHED_FIFO` priority and `mlockall`. Create a `realtime`
group, add the user, and grant it the limits.

```bash
sudo groupadd -r realtime
sudo usermod -aG realtime "$USER"
sudo tee /etc/security/limits.d/99-realtime.conf >/dev/null <<'EOF'
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
EOF
```

The SDK raises its control loop to priority 90, so `rtprio` must be at least that; `mlockall`
also locks future allocations, so `memlock` must be `unlimited`.

Log out and back in, then verify in the new session.

```bash
id -nG         # current user's groups → realtime present
ulimit -r      # rtprio  → 99
ulimit -l      # memlock → unlimited
```

---

Next: bring up the interface in [CAN-FD setup](05_can_fd_setup.md), then build the SDK in
[SDK build & install](06_sdk_build_and_install.md).
