#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
daemon_config=
mode=host
daemon_options=()

while (($#)); do
    case "$1" in
        --help|-h)
            echo 'usage: run_simulation.sh [--existing-can | --isolated-vcan | --without-can] [--config YAML] [--no-terminals]'
            echo 'Default: vcan0 + existing MuJoCo OpenGL GUI + control processes + web GUI.'
            echo 'Physical hardware uses run_daemon.sh; HW-policy vcan tests use run_hw_deploy_test.sh.'
            exit 0 ;;
        --existing-can|--isolated-vcan|--without-can)
            if [[ "$mode" != host ]]; then
                echo '--existing-can, --isolated-vcan and --without-can cannot be combined' >&2
                exit 1
            fi

            mode="${1#--}"
            shift
            ;;
        --no-terminals) daemon_options+=(--no-terminals); shift ;;
        --config)
            if (($# < 2)); then
                echo '--config requires a YAML path' >&2
                exit 1
            fi

            daemon_config="$(realpath -- "$2")"
            shift 2
            ;;
        *) echo 'usage: run_simulation.sh [--existing-can | --isolated-vcan | --without-can] [--config YAML] [--no-terminals]' >&2; exit 1 ;;
    esac
done



# Build before any CAN setup or process launch, including missing child binaries.
source "$project_dir/scripts/ensure_build.sh"
qhrr_ensure_native_build "$project_dir" daemon simulation control can_hal input shm_logger qhrr_logctl web_gui_bridge
work_dir=$(mktemp -d /tmp/qhrr-simulation.XXXXXX)
session_pid=
cleanup() {
    if [[ -n "$session_pid" ]]; then
        kill -TERM -- "-$session_pid" 2>/dev/null || true
        wait "$session_pid" 2>/dev/null || true
    fi
    rm -rf -- "$work_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
run_session() {
    setsid -- "$@" &
    session_pid=$!
    local status=0
    wait "$session_pid" || status=$?
    session_pid=
    return "$status"
}
config_options=()
if [[ -n "$daemon_config" ]]; then config_options+=(--template "$daemon_config"); fi
if [[ "$mode" == without-can ]]; then config_options+=(--without-can); fi
python3 "$project_dir/scripts/simulation_config.py" "$project_dir" "$work_dir" "${config_options[@]}"
daemon_config="$work_dir/daemon.yaml"


if [[ "$mode" == without-can ]]; then
    run_session "$project_dir/build/daemon" --config "$daemon_config" --without-can "${daemon_options[@]}"
    exit $?
fi

if [[ "$mode" == isolated-vcan ]]; then
    echo '[launcher] DEBUG: creating vcan0 in an isolated network namespace'
    run_session unshare --user --map-root-user --net -- sh -ec '
        ip link set dev lo up
        ip link add dev vcan0 type vcan
        ip link set dev vcan0 up
        echo "[launcher] DEBUG network namespace: $(readlink /proc/self/ns/net)"
        ip -details link show dev vcan0
        runtime_dir=$(mktemp -d /tmp/qhrr05-can-session.XXXXXX)
        export XDG_RUNTIME_DIR="$runtime_dir"
        unset GNOME_TERMINAL_SERVICE GNOME_TERMINAL_SCREEN AT_SPI_BUS_ADDRESS
        export NO_AT_BRIDGE=1 GIO_USE_VFS=local GTK_USE_PORTAL=0
        trap "rm -rf -- \"$runtime_dir\"" EXIT
        dbus-run-session -- "$@"
    ' sh "$project_dir/build/daemon" --config "$daemon_config" "${daemon_options[@]}"
    exit $?
fi

if [[ "$mode" == host ]]; then
    # Query errors abort. An existing device must really be a vcan device.
    link_state=$(ip -json -details link show | python3 -c '
import json, sys
link = next((item for item in json.load(sys.stdin) if item["ifname"] == "vcan0"), None)
if link is None:
    print("missing")
elif link.get("linkinfo", {}).get("info_kind") != "vcan":
    sys.exit("[launcher] vcan0 exists but is not a vcan device; refusing to change it")
else:
    print("up" if "UP" in link["flags"] else "down")
')

    if [[ "$link_state" != up ]]; then
        net_admin=0

        while read -r key value _; do
            if [[ "$key" == CapEff: ]]; then
                net_admin=$((16#$value & 0x1000))
                break
            fi
        done < /proc/self/status

        network_ip=(ip)

        if ((net_admin == 0)); then
            echo '[launcher] Host vcan0 setup requires sudo; only ip link commands are elevated.' >&2
            network_ip=(sudo -- ip)
        fi

        if [[ "$link_state" == missing ]]; then
            "${network_ip[@]}" link add dev vcan0 type vcan
        fi

        "${network_ip[@]}" link set dev vcan0 up
    fi

    echo "[launcher] Host vcan0: $(readlink /proc/self/ns/net) (kept after daemon exit)"
    ip -details link show dev vcan0
else
    echo "[launcher] Existing CAN: $(readlink /proc/self/ns/net); no interface setup"
fi

python3 - "$daemon_config" <<'PYCODE'
import json, pathlib, subprocess, sys, yaml
path = pathlib.Path(sys.argv[1])
manifest = yaml.safe_load(path.read_text())
process = next((p for p in manifest['processes'] if p['name'] == 'can_hal'), None)
if process:
    work = (path.parent / process.get('working_directory', '.')).resolve()
    interface = yaml.safe_load((work / process['args'][0]).read_text())['interface']
    link = json.loads(subprocess.check_output(['ip', '-json', '-details', 'link', 'show', 'dev', interface]))[0]
    if link.get('linkinfo', {}).get('info_kind') != 'vcan':
        sys.exit('Simulation may use only an explicit vcan interface, never a physical CAN device')
PYCODE
run_session "$project_dir/build/daemon" --config "$daemon_config" "${daemon_options[@]}"
