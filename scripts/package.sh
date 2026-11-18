#!/usr/bin/env bash
# Builds the prebuilt package of this checkout for the host architecture:
#   scripts/package.sh [output dir, build-package by default]
# Writes aidin_hand2-<version>-<arch>.tar.gz and its .debug file, then builds and runs
# cpp/tests/package against the package alone.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
out=$(realpath -m "${1:-$root/build-package}")
arch=$(uname -m)
version=$(sed -n 's/^project(aidin_hand2 VERSION \([0-9.]*\).*/\1/p' "$root/cpp/CMakeLists.txt")
name=aidin_hand2-$version-$arch
build=$out/build
stage=$out/$name

rm -rf "$build" "$stage" "$out/consumer"
cmake -S "$root/cpp" -B "$build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DAIDIN_HAND2_BUILD_TESTS=OFF -DAIDIN_HAND2_BUILD_EXAMPLES=OFF
cmake --build "$build" -j"$(nproc)"
cmake --install "$build" --prefix "$stage"

# Debug info goes to its own file, which the stripped library names
lib=$stage/lib/libaidin_hand2.so.$version
objcopy --only-keep-debug "$lib" "$out/$name.debug"
strip --strip-unneeded "$lib"
objcopy --add-gnu-debuglink="$out/$name.debug" "$lib"

dirty=no
git -C "$root" diff --quiet HEAD || dirty=yes
cat > "$stage/share/doc/aidin_hand2/BUILD-INFO" <<EOF
sdk_version:   $version
commit:        $(git -C "$root" rev-parse HEAD)
source_dirty:  $dirty
built:         $(date -u +%Y-%m-%dT%H:%M:%SZ)
arch:          $arch
compiler:      $(c++ --version | head -1)
glibc:         $(ldd --version | head -1 | awk '{print $NF}')
EOF

tar -C "$out" -czf "$out/$name.tar.gz" "$name"

cmake -S "$root/cpp/tests/package" -B "$out/consumer" -DCMAKE_PREFIX_PATH="$stage"
cmake --build "$out/consumer"
"$out/consumer/package_consumer"

echo "$out/$name.tar.gz"
