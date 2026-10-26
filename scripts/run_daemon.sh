#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
process_config="$project_dir/config/app_config/daemon_config.yaml"
existing_can=false
without_can=false

while (($#)); do
    case "$1" in
        --existing-can) existing_can=true; shift ;;
        --without-can) without_can=true; shift ;;
        --process-config)
            if (($# < 2)); then
                echo '--process-config requires a YAML path' >&2
                exit 1
            fi

            process_config="$(realpath -- "$2")"
            shift 2
            ;;
        *) echo 'usage: run_daemon.sh [--existing-can | --without-can] [--process-config YAML]' >&2; exit 1 ;;
    esac
done

if "$existing_can" && "$without_can"; then
    echo '--existing-can and --without-can cannot be combined' >&2
    exit 1
fi

if "$without_can"; then
    exec "$project_dir/build/daemon" --config "$process_config" --without-can
fi

if ! "$existing_can"; then
    exec unshare --user --map-root-user --net -- sh -ec '
        ip link add dev vcan0 type vcan
        ip link set dev vcan0 up
        runtime_dir=$(mktemp -d /tmp/qhrr05-can-session.XXXXXX)
        export XDG_RUNTIME_DIR="$runtime_dir"
        unset GNOME_TERMINAL_SERVICE GNOME_TERMINAL_SCREEN AT_SPI_BUS_ADDRESS
        export NO_AT_BRIDGE=1 GIO_USE_VFS=local GTK_USE_PORTAL=0
        trap "rm -rf -- \"$runtime_dir\"" EXIT
        dbus-run-session -- "$@"
    ' sh "$project_dir/scripts/run_daemon.sh" --existing-can --process-config "$process_config"
fi

exec "$project_dir/build/daemon" --config "$process_config"
