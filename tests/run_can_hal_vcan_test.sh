#!/usr/bin/env bash
set -euo pipefail
# Private vcan and user/network namespaces; no host interface changes or sudo.
exec unshare --user --map-root-user --net -- sh -ec '
    ip link add dev vcan0 type vcan
    ip link set dev vcan0 up
    exec "$@"
' sh "$@"
