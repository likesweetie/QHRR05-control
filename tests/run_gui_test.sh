#!/usr/bin/env bash
set -euo pipefail
gui_runtime_dir=$(mktemp -d /tmp/qhrr05-gui-session.XXXXXX)
trap 'rm -rf -- "$gui_runtime_dir"' EXIT
export XDG_RUNTIME_DIR="$gui_runtime_dir"
export GDK_BACKEND=x11 NO_AT_BRIDGE=1 GIO_USE_VFS=local GTK_USE_PORTAL=0
unset GNOME_TERMINAL_SERVICE GNOME_TERMINAL_SCREEN AT_SPI_BUS_ADDRESS WAYLAND_DISPLAY
xvfb-run -a -s '-screen 0 2560x1440x24' dbus-run-session -- \
    python3 "$@"
