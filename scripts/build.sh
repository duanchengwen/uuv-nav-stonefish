#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SF_SRC="${SF_SRC:-$(cd "$ROOT/.." && pwd)/stonefish}"
SF_BUILD="${SF_BUILD:-$(cd "$ROOT/.." && pwd)/build_stonefish}"
SF_INSTALL="${SF_INSTALL:-$(cd "$ROOT/.." && pwd)/install_stonefish}"

if [[ ! -f "$SF_SRC/CMakeLists.txt" ]]; then
  echo "Stonefish source not found at $SF_SRC"
  echo "Clone it first: git clone --branch v1.5 https://github.com/patrykcieslak/stonefish.git $SF_SRC"
  exit 1
fi

cmake -S "$SF_SRC" -B "$SF_BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$SF_INSTALL" \
  -DBUILD_TESTS=OFF
cmake --build "$SF_BUILD" -j"$(nproc)"
cmake --install "$SF_BUILD"

cmake -S "$ROOT" -B "$ROOT/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$SF_INSTALL"
cmake --build "$ROOT/build" -j"$(nproc)"

echo
echo "Built: $ROOT/build/uuv_nav"
echo "Run with:"
echo "  export LD_LIBRARY_PATH=$SF_INSTALL/lib:\$LD_LIBRARY_PATH"
echo "  ./build/uuv_nav --scene <episode>/scene.scn --episode-dir <episode>"
