#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
daemon_config="$project_dir/config/daemon/daemon.yaml"
mode=host
daemon_options=()

while (($#)); do
    case "$1" in
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
        *) echo 'usage: run_daemon.sh [--existing-can | --isolated-vcan | --without-can] [--config YAML] [--no-terminals]' >&2; exit 1 ;;
    esac
done

if [[ "$mode" == without-can ]]; then
    exec "$project_dir/build/daemon" --config "$daemon_config" --without-can "${daemon_options[@]}"
fi

if [[ "$mode" == isolated-vcan ]]; then
    echo '[launcher] DEBUG: creating vcan0 in an isolated network namespace'
    exec unshare --user --map-root-user --net -- sh -ec '
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

exec "$project_dir/build/daemon" --config "$daemon_config" "${daemon_options[@]}"
