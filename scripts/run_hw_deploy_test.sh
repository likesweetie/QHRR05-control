#!/usr/bin/env bash
# Hardware application and settings, with vcan0 as the only device substitution.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
verify=0
selected_mode=
requested_session=
private=0
inside=0
while (($#)); do
    case "$1" in
        --interactive|--verify)
            if [[ -n "$selected_mode" && "$selected_mode" != "$1" ]]; then
                echo '--interactive and --verify cannot be combined' >&2; exit 1
            fi
            selected_mode=$1
            if [[ "$1" == --verify ]]; then verify=1; else verify=0; fi
            shift ;;
        --isolated-vcan) private=1; shift ;;
        --shm-name) requested_session="${2:?--shm-name requires a value}"; shift 2 ;;
        --in-namespace) inside=1; shift ;;
        --help)
            echo 'usage: run_hw_deploy_test.sh [--interactive | --verify [--isolated-vcan]]'
            echo 'Default: interactive MuJoCo OpenGL window on host vcan0.'
            echo '--verify: run the automatic hardware-pipeline checks without a window.'
            echo '--shm-name /NAME: choose a separate session (GUI default: /qhrr05).'
            exit 0 ;;
        *) echo "unknown hardware test option: $1" >&2; exit 1 ;;
    esac
done
if ((private && !verify)); then
    echo 'Interactive OpenGL testing uses host vcan0; use --verify --isolated-vcan for automatic verification.' >&2; exit 1
fi
if ((private && !inside)); then
    extra=()
    if [[ -n "$requested_session" ]]; then extra+=(--shm-name "$requested_session"); fi
    exec unshare --user --map-root-user --net -- "$0" --verify --in-namespace "${extra[@]}"
fi
if ((inside)); then
    ip link set dev lo up
    ip link add dev vcan0 type vcan
    ip link set dev vcan0 up
fi
python3 - <<'PY'
import json, subprocess, sys
result = subprocess.run(['ip', '-json', '-details', 'link', 'show', 'dev', 'vcan0'],
                        capture_output=True, text=True)
if result.returncode:
    sys.exit('vcan0 is required; use --verify --isolated-vcan for an isolated automatic test.')
link, = json.loads(result.stdout)
if link.get('linkinfo', {}).get('info_kind') != 'vcan' or 'UP' not in link.get('flags', []):
    sys.exit('Hardware testing requires an UP virtual CAN interface named vcan0; no device is substituted.')
PY
if ((!verify)) && [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo 'Interactive OpenGL testing requires a display; use --verify for a windowless test.' >&2; exit 1
fi
source "$project_dir/scripts/ensure_build.sh"
qhrr_ensure_native_build "$project_dir" daemon control can_hal input shm_logger web_gui_bridge simulation
work_dir=$(mktemp -d /tmp/qhrr-hw-deploy.XXXXXX)
trap 'rm -rf -- "$work_dir"' EXIT
session="/qhrr_hw_test_$$"
if ((!verify)); then session=/qhrr05; fi
if [[ -n "$requested_session" ]]; then session=$requested_session; fi
if [[ ! "$session" =~ ^/[a-zA-Z0-9_.-]+$ ]]; then
    echo 'SHM session must be / followed by a single safe name.' >&2; exit 1
fi
simulation_options=(--hardware --shm-name "$session")
if ((verify)); then simulation_options+=(--headless); fi
python3 "$project_dir/scripts/simulation_config.py" "$project_dir" "$work_dir" "${simulation_options[@]}"
echo "[hardware-test] Hardware policy, physical CAN profile, vcan0 device, session=$session"
if ((verify)); then
    python3 "$project_dir/tests/hardware_deploy_test.py" "$work_dir/daemon.yaml" "$session"
else
    echo '[hardware-test] MuJoCo OpenGL | Z: zero set, A: Arm, X: Release, H: homing'
    "$project_dir/scripts/run_daemon.sh" --config "$work_dir/daemon.yaml" --no-terminals
fi
