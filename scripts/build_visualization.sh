#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build-filament"
jobs=8
while (($#)); do
    case "$1" in
        --jobs) jobs="${2:?--jobs requires a count}"; shift 2 ;;
        --help) echo 'usage: build_visualization.sh [--jobs N]'; exit 0 ;;
        *) echo "unknown build option: $1" >&2; exit 1 ;;
    esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'invalid build job count' >&2; exit 1; }
for tool in cmake ninja git pkg-config; do
    command -v "$tool" >/dev/null || { echo "required tool missing: $tool" >&2; exit 1; }
done
if [[ ! -x "$build_dir/toolchain/bin/clang++-18" ]]; then
    "$project_dir/scripts/setup_visualization_toolchain.sh"
fi
cache="$build_dir/native/CMakeCache.txt"
if [[ -f "$cache" ]]; then
    cached_compiler="$(sed -n 's/^CMAKE_CXX_COMPILER:STRING=//p; s/^CMAKE_CXX_COMPILER:FILEPATH=//p' "$cache")"
    if [[ "$cached_compiler" != "$build_dir/toolchain/bin/clang++-18" ]]; then
        echo '[build] Compiler changed; explicitly clearing the visualization CMake cache before configuring'
        rm -- "$cache"
    fi
fi
pkg-config --exists glfw3 vulkan || { echo 'GLFW3 and Vulkan development packages are required' >&2; exit 1; }
revision=a892750516336d8358949f4518fcc4fbbd85beae
source_dir="$build_dir/mujoco-src"
if [[ ! -d "$source_dir" ]]; then
    mkdir -p "$build_dir"
    git clone --depth 1 --branch 3.16.0 https://github.com/google-deepmind/mujoco.git "$source_dir"
fi
actual_revision="$(git -C "$source_dir" rev-parse HEAD)"
[[ "$actual_revision" == "$revision" ]] || {
    echo "MuJoCo source revision mismatch: expected $revision; found $actual_revision" >&2; exit 1;
}
git -C "$source_dir" diff --exit-code HEAD -- >/dev/null || {
    echo 'pinned MuJoCo source has local modifications; refusing to build' >&2; exit 1;
}
echo "[build] MuJoCo 3.16.0 ($revision); upstream-pinned Filament rc/1.77.3; Vulkan + Classic mjr"
cmake -S "$source_dir" -B "$build_dir/native" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER="$build_dir/toolchain/bin/clang-18" \
    -DCMAKE_CXX_COMPILER="$build_dir/toolchain/bin/clang++-18" \
    -DCMAKE_CXX_FLAGS=-Wno-error=deprecated-declarations \
    -DUSE_STATIC_LIBCXX=OFF -DMUJOCO_ENABLE_LTO=OFF -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
    -DMUJOCO_USE_FILAMENT=ON -DMUJOCO_USE_FILAMENT_MJR_COMPAT=OFF \
    -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_TEST_PYTHON_UTIL=OFF -DMUJOCO_BUILD_EXAMPLES=OFF \
    -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_STUDIO=OFF \
    -DQHRR05_SOURCE_DIR="$project_dir" \
    -DCMAKE_PROJECT_mujoco_INCLUDE="$project_dir/cmake/filament_viewer.cmake"
cmake --build "$build_dir/native" --target qhrr_visualizer visualization_snapshot_test -j "$jobs"
echo "[build] ready: $build_dir/native/bin/qhrr_visualizer"
