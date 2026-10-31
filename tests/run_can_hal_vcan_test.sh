#!/usr/bin/env bash
set -euo pipefail
# Private vcan and user/network namespaces; no host interface changes or sudo.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec unshare --user --map-root-user --net -- bash -ec '
    project_dir=$1
    shift
    ip link set dev lo up
    ip link add dev vcan0 type vcan
    ip link set dev vcan0 up
    runtime_dir=$(mktemp -d /tmp/qhrr-vcan-test.XXXXXX)
    trap '\''rm -rf -- "$runtime_dir"'\'' EXIT
    python3 "$project_dir/scripts/simulation_config.py" "$project_dir" "$runtime_dir/can.yaml" --hal-only
    args=()
    for arg in "$@"; do
        if [[ "$arg" == "$project_dir/config/can_hal/can_hal.yaml" ]]; then
            args+=("$runtime_dir/can.yaml")
        else
            args+=("$arg")
        fi
    done
    "${args[@]}"
' bash "$project_dir" "$@"
