#!/usr/bin/env bash
# Build locate-anything.cpp (CLI + shared library) at a pinned commit.
#
#   scripts/build_locate_anything.sh           # auto: CUDA if nvcc is found (Jetson), else CPU
#   LA_CUDA=0 scripts/build_locate_anything.sh # force CPU
#
# Output: third_party/locate-anything.cpp/build/bin/locate-anything-cli
#         third_party/locate-anything.cpp/build/**/liblocate_anything.so  (C API: include/la_capi.h)
# On the Jetson, make sure CUDA is on PATH first:  export PATH=/usr/local/cuda/bin:$PATH
set -euo pipefail

URL="https://github.com/mudler/locate-anything.cpp"
COMMIT="77376ab332de918220f7a7e391542eefb5407c9f"   # pinned engine version (matches the GGUF format)

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/third_party/locate-anything.cpp"

if [[ ! -d "$SRC/.git" ]]; then
  git clone --filter=blob:none "$URL" "$SRC"
fi
git -C "$SRC" fetch --quiet origin "$COMMIT" || true
git -C "$SRC" checkout --quiet "$COMMIT"
git -C "$SRC" submodule update --init --recursive --quiet

cuda="${LA_CUDA:-auto}"
if [[ "$cuda" == auto ]]; then
  if command -v nvcc >/dev/null 2>&1; then cuda=1; else cuda=0; fi
fi
flags=(-DCMAKE_BUILD_TYPE=Release -DLA_BUILD_CLI=ON -DLA_SHARED=ON)
if [[ "$cuda" == 1 ]]; then
  flags+=(-DLA_GGML_CUDA=ON)
  # Jetson Orin = compute capability 8.7; building only for it is much faster.
  if [[ -f /etc/nv_tegra_release ]]; then flags+=(-DCMAKE_CUDA_ARCHITECTURES=87); fi
  echo "Building with CUDA."
else
  echo "Building for CPU only."
fi

jobs="${JOBS:-$(nproc)}"
# The Orin Nano has 8 GB shared RAM; cap parallel compile jobs so the build does not swap.
if [[ -f /etc/nv_tegra_release && -z "${JOBS:-}" ]]; then jobs=4; fi

cmake -S "$SRC" -B "$SRC/build" "${flags[@]}"
cmake --build "$SRC/build" -j "$jobs"

cli=$(find "$SRC/build" -name locate-anything-cli -type f -perm -u+x | head -1)
lib=$(find "$SRC/build" -name 'liblocate_anything.so*' -type f | head -1)
echo "CLI: ${cli:-NOT FOUND}"
echo "Library: ${lib:-NOT FOUND}"
[[ -n "$cli" && -n "$lib" ]]
