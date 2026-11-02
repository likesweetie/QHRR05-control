#!/usr/bin/env bash
# First-time dependency installation and builds for Ubuntu/Pop!_OS x86_64.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
jobs="${QHRR_BUILD_JOBS:-4}"
skip_system=0
visualization=0
dry_run=0
temporary_dir=

usage() {
    cat <<'HELP'
usage: scripts/setup.sh [--skip-system] [--with-visualization] [--jobs N] [--dry-run]

Install native/CAN dependencies, prepare a Python analysis venv and Node.js,
then build the native programs and web GUI. Run as your normal login user;
only apt-get uses sudo when needed. Ubuntu/Pop!_OS, Linux x86_64.

  --skip-system         Skip apt-get when system packages are already installed.
  --with-visualization  Also build the pinned Filament viewer/toolchain.
  --jobs N              Native build parallelism (default: QHRR_BUILD_JOBS or 4).
  --dry-run             Print the plan without installing or changing files.

The daemon, CAN interface, and motors are not started by setup.
HELP
}

while (($#)); do
    case "$1" in
        --skip-system) skip_system=1; shift ;;
        --with-visualization) visualization=1; shift ;;
        --jobs) jobs="${2:?--jobs requires a positive integer}"; shift 2 ;;
        --dry-run) dry_run=1; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "[setup] Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo '[setup] --jobs must be a positive integer' >&2; exit 1; }
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || {
    echo '[setup] This repository bundles a Linux x86_64 MuJoCo runtime.' >&2
    exit 1
}
for file in third_party/mujoco/include/mujoco/mujoco.h third_party/mujoco/lib/libmujoco.so \
    third_party/data_tamer/data_tamer_cpp/CMakeLists.txt third_party/mcap/cpp/mcap/include/mcap/reader.hpp; do
    [[ -s "$project_dir/$file" ]] || { echo "[setup] Missing bundled dependency: $file" >&2; exit 1; }
done

run() {
    printf '[setup]'
    printf ' %q' "$@"
    printf '\n'
    if ((!dry_run)); then "$@"; fi
}

packages=(
    build-essential cmake ninja-build pkg-config git curl ca-certificates xz-utils
    python3 python3-venv python3-pip python3-yaml
    libyaml-cpp-dev libssl-dev libzstd-dev liblz4-dev
    libglfw3-dev libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libvulkan-dev
    iproute2 can-utils util-linux gnome-terminal dbus-x11
)
if ((visualization)); then
    packages+=(libedit2 libxml2 libz3-4 libtinfo6 zlib1g-dev)
fi
if ((!skip_system)); then
    command -v apt-get >/dev/null || { echo '[setup] apt-get is required; install system dependencies manually and use --skip-system.' >&2; exit 1; }
    apt_command=(apt-get)
    if ((EUID != 0)); then
        command -v sudo >/dev/null || { echo '[setup] sudo is required to install system packages.' >&2; exit 1; }
        apt_command=(sudo -- apt-get)
    fi
    run "${apt_command[@]}" update
    run "${apt_command[@]}" install -y --no-install-recommends "${packages[@]}"
fi

source "$project_dir/scripts/ensure_build.sh"
qhrr_use_local_tools "$project_dir"
node_ready() {
    command -v node >/dev/null && command -v npm >/dev/null && node -e '
        const [major, minor] = process.versions.node.split(".").map(Number);
        process.exit(major > 22 || (major === 22 && minor >= 12) ? 0 : 1);
    ' >/dev/null 2>&1
}

if ! node_ready; then
    echo '[setup] Preparing official Node.js 22 LTS in .cache/setup (SHA256 verified).'
    echo '[setup] Download source: https://nodejs.org/dist/latest-v22.x/SHASUMS256.txt'
    if ((!dry_run)); then
        for tool in curl tar sha256sum; do
            command -v "$tool" >/dev/null || { echo "[setup] Missing executable: $tool" >&2; exit 1; }
        done
        cache_dir="$project_dir/.cache/setup"
        mkdir -p "$cache_dir"
        temporary_dir=$(mktemp -d "$cache_dir/node-download.XXXXXX")
        trap '[[ -z "$temporary_dir" ]] || rm -rf -- "$temporary_dir"' EXIT
        base_url=https://nodejs.org/dist/latest-v22.x
        curl_options=(--fail --location --proto '=https' --tlsv1.2 --retry 3 --connect-timeout 15 --max-time 300)
        curl "${curl_options[@]}" "$base_url/SHASUMS256.txt" -o "$temporary_dir/SHASUMS256.txt"
        archive=$(awk '$2 ~ /^node-v22\.[0-9]+\.[0-9]+-linux-x64\.tar\.xz$/ {print $2}' "$temporary_dir/SHASUMS256.txt")
        [[ "$archive" =~ ^node-v22\.[0-9]+\.[0-9]+-linux-x64\.tar\.xz$ ]] || {
            echo '[setup] Invalid or ambiguous Node.js archive in official checksum list.' >&2; exit 1;
        }
        release="${archive#node-}"
        release="${release%-linux-x64.tar.xz}"
        curl "${curl_options[@]}" "https://nodejs.org/dist/$release/$archive" -o "$temporary_dir/$archive"
        (cd "$temporary_dir"; awk -v name="$archive" '$2 == name {print}' SHASUMS256.txt | sha256sum --check --strict -)
        mkdir "$temporary_dir/unpacked"
        tar -xJf "$temporary_dir/$archive" --strip-components=1 -C "$temporary_dir/unpacked"
        [[ -x "$temporary_dir/unpacked/bin/node" && -x "$temporary_dir/unpacked/bin/npm" ]] || {
            echo '[setup] Node.js archive is missing node/npm.' >&2; exit 1;
        }
        version_dir="$cache_dir/${archive%.tar.xz}"
        if [[ ! -e "$version_dir" ]]; then
            mv -- "$temporary_dir/unpacked" "$version_dir"
        fi
        [[ -x "$version_dir/bin/node" && -x "$version_dir/bin/npm" ]] || {
            echo "[setup] Incomplete cached Node.js installation: $version_dir" >&2; exit 1;
        }
        ln -s -- "${archive%.tar.xz}" "$temporary_dir/node"
        mv -Tf -- "$temporary_dir/node" "$cache_dir/node"
        qhrr_use_local_tools "$project_dir"
        node_ready || { echo '[setup] Installed Node.js does not meet the web GUI requirement.' >&2; exit 1; }
    fi
elif [[ ! -x "$project_dir/.cache/setup/node/bin/node" || ! -x "$project_dir/.cache/setup/node/bin/npm" ]]; then
    # Remember an existing nvm/system installation without editing shell profiles.
    # The launcher can then find both tools even in a fresh non-login shell.
    node_path="$(type -P node)"
    npm_path="$(type -P npm)"
    cache_dir="$project_dir/.cache/setup"
    run mkdir -p "$cache_dir/existing-node/bin"
    run ln -sfnT -- "$node_path" "$cache_dir/existing-node/bin/node"
    run ln -sfnT -- "$npm_path" "$cache_dir/existing-node/bin/npm"
    run ln -sfnT -- existing-node "$cache_dir/node"
    qhrr_use_local_tools "$project_dir"
fi

venv_dir="$project_dir/.venv"
run python3 -c 'import yaml'
run python3 -m venv "$venv_dir"
run "$venv_dir/bin/python" -m pip install -r "$project_dir/tools/analysis/requirements.txt"
run "$venv_dir/bin/python" -c 'import mcap, numpy, pandas, matplotlib, yaml'
run pkg-config --exists yaml-cpp glfw3 openssl libzstd liblz4 x11 xrandr
run cmake -S "$project_dir" -B "$project_dir/build" -DCMAKE_BUILD_TYPE=Debug \
    "-DQHRR_ANALYSIS_PYTHON=$venv_dir/bin/python"
run cmake --build "$project_dir/build" --parallel "$jobs"
run npm --prefix "$project_dir/apps/interfaces/web_gui/frontend" ci --no-audit --no-fund
run npm --prefix "$project_dir/apps/interfaces/web_gui/frontend" ls --depth=0
run npm --prefix "$project_dir/apps/interfaces/web_gui/frontend" run build
if ((visualization)); then
    run "$project_dir/scripts/build_visualization.sh" --jobs "$jobs"
fi

if ((dry_run)); then
    echo '[setup] Dry run complete; no changes made.'
else
    echo "[setup] Ready. Node.js $(node --version); Python analysis: $venv_dir/bin/python"
    echo '[setup] Start a hardware session with: ./scripts/run_daemon.sh'
fi
