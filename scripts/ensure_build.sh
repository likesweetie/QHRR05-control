#!/usr/bin/env bash
# Shared by the daemon/visualization launchers; the web launcher is independent.

qhrr_native_ready() {
    local build_dir="$1" target binary
    shift
    for target in "$@"; do
        binary="$target"
        [[ "$target" != qhrr_logctl ]] || binary=qhrr-logctl
        [[ -x "$build_dir/$binary" ]] || return 1
    done
}

qhrr_visualization_ready() {
    local directory="$1/build-filament/native/bin" asset
    [[ -x "$directory/qhrr_visualizer" ]] || return 1
    [[ -s "$directory/assets/.qhrr-manifest" ]] || return 1
    [[ -s "$directory/assets/qhrr_pbr.filamat" ]] || return 1
    while IFS= read -r asset; do
        [[ "$asset" =~ ^[a-zA-Z0-9_.-]+$ ]] || return 1
        [[ -s "$directory/assets/$asset" ]] || return 1
    done < "$directory/assets/.qhrr-manifest"
}

qhrr_install_visualization_assets() {
    local source="$1/build-filament/native/src/render/filament/assets"
    local destination="$1/build-filament/native/bin/assets" file
    [[ -s "$source/ibl.ktx" ]] || {
        echo '[build] Compiled visualization assets are missing' >&2
        return 1
    }
    cmake -E copy_directory "$source" "$destination"
    # Copy on every recovery build, even when the executable did not need relinking.
    # Atomic manifest replacement prevents an interrupted copy from appearing ready.
    printf 'ibl.ktx\n' > "$destination/.qhrr-manifest.tmp"
    for file in "$source"/*.filamat; do
        [[ -s "$file" ]] || {
            echo '[build] Compiled shader is missing' >&2
            return 1
        }
        printf '%s\n' "${file##*/}" >> "$destination/.qhrr-manifest.tmp"
    done
    mv -- "$destination/.qhrr-manifest.tmp" "$destination/.qhrr-manifest"
}

qhrr_build_tools() {
    local tool
    for tool in cmake flock; do
        command -v "$tool" >/dev/null || {
            echo "[build] required tool is missing: $tool" >&2
            return 1
        }
    done
    [[ "${QHRR_BUILD_JOBS:-4}" =~ ^[1-9][0-9]*$ ]] || {
        echo '[build] QHRR_BUILD_JOBS must be a positive integer' >&2
        return 1
    }
}

qhrr_ensure_native_build() (
    set -euo pipefail
    local project_dir="$1"
    shift
    qhrr_native_ready "$project_dir/build" "$@" && exit 0
    qhrr_build_tools
    mkdir -p "$project_dir/.cache/build-locks"
    exec 9>"$project_dir/.cache/build-locks/native.lock"
    flock 9
    # Another launcher may have completed the build while we waited.
    qhrr_native_ready "$project_dir/build" "$@" && exit 0
    echo "[build] Native executable missing; automatically building: $*"
    if [[ -f "$project_dir/build/CMakeCache.txt" ]]; then
        cmake -S "$project_dir" -B "$project_dir/build"
    else
        cmake -S "$project_dir" -B "$project_dir/build" -DCMAKE_BUILD_TYPE=Debug
    fi
    cmake --build "$project_dir/build" --parallel "${QHRR_BUILD_JOBS:-4}" --target "$@"
    qhrr_native_ready "$project_dir/build" "$@" || {
        echo '[build] Native build finished without the required executables; not launching' >&2
        exit 1
    }
)

qhrr_ensure_visualization_build() (
    set -euo pipefail
    local project_dir="$1" force="${2:-0}"
    if [[ "$force" == 0 ]] && qhrr_visualization_ready "$project_dir"; then
        exit 0
    fi
    qhrr_build_tools
    mkdir -p "$project_dir/.cache/build-locks"
    exec 9>"$project_dir/.cache/build-locks/visualization.lock"
    flock 9
    if [[ "$force" == 0 ]] && qhrr_visualization_ready "$project_dir"; then
        exit 0
    fi
    echo '[build] Preparing the pinned visualizer and runtime assets before launch'
    echo '[build] A clean build may download the pinned sources/toolchain and take several minutes'
    "$project_dir/scripts/build_visualization.sh" --jobs "${QHRR_BUILD_JOBS:-8}"
    qhrr_install_visualization_assets "$project_dir"
    qhrr_visualization_ready "$project_dir" || {
        echo '[build] Visualization build finished without the executable/assets; not launching' >&2
        exit 1
    }
)
