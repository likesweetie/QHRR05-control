#!/usr/bin/env bash
# Default physical-CAN entry point. The manifest explicitly chooses the device.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
daemon_config="$project_dir/config/daemon/daemon.yaml"
daemon_args=()
while (($#)); do
    case "$1" in
        --config) daemon_config="$(realpath -- "${2:?--config requires a path}")"; shift 2 ;;
        --no-terminals|--terminals) daemon_args+=("$1"); shift ;;
        --help)
            echo 'usage: run_daemon.sh [--config YAML] [--no-terminals | --terminals]'
            echo 'Build changed native sources, bring up the configured CAN interface, then start the hardware session.'
            echo 'Before launch, stop the previous project session and remove its stale SHM/sockets.'
            echo 'Only interface setup may use sudo. For MuJoCo use run_simulation.sh.'
            echo 'Each process opens its own tiled terminal, using XWayland on Wayland; --no-terminals uses inline logs.'
            exit 0 ;;
        *) echo "Unknown hardware option: $1. For simulation use scripts/run_simulation.sh." >&2; exit 1 ;;
    esac
done
source "$project_dir/scripts/ensure_build.sh"
qhrr_use_local_tools "$project_dir"
qhrr_ensure_native_build "$project_dir" daemon control can_hal input shm_logger qhrr_logctl web_gui_bridge
qhrr_cleanup_session "$project_dir" "$daemon_config"
# Prepare only the explicitly configured, existing CAN device after a successful build.
python3 "$project_dir/scripts/prepare_can.py" "$daemon_config"
exec "$project_dir/build/daemon" --config "$daemon_config" "${daemon_args[@]}"
