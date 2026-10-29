#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode=session
setup=0
daemon_config="$project_dir/config/daemon/visualization.yaml"
custom_config=0
viewer_args=()
daemon_args=()
export QHRR_VISUALIZATION_BACKEND=filament QHRR_VISUALIZATION_FPS=60
while (($#)); do
    case "$1" in
        --setup) setup=1; shift ;;
        --config) daemon_config="$(realpath -- "${2:?--config requires a path}")"; custom_config=1; shift 2 ;;
        --attach|--preview)
            [[ "$mode" == session ]] || { echo '--attach and --preview cannot be combined' >&2; exit 1; }
            mode="${1#--}"; shift ;;
        --backend) export QHRR_VISUALIZATION_BACKEND="${2:?--backend requires a value}"; shift 2 ;;
        --msaa) export QHRR_VISUALIZATION_MSAA="${2:?--msaa requires a value}"; shift 2 ;;
        --render-scale) export QHRR_VISUALIZATION_RENDER_SCALE="${2:?--render-scale requires a value}"; shift 2 ;;
        --fps) export QHRR_VISUALIZATION_FPS="${2:?--fps requires a value}"; shift 2 ;;
        --shm-name|--frames|--width|--height|--slow-render-ms|--screenshot|--assets|--ibl|--control-config)
            viewer_args+=("$1" "${2:?option requires a value}"); shift 2 ;;
        --hidden|--no-ui) viewer_args+=("$1"); shift ;;
        --existing-can|--isolated-vcan|--without-can|--no-terminals)
            daemon_args+=("$1"); shift ;;
        --help)
            cat <<'USAGE'
usage: run_visualization.sh [--setup] [--backend filament|classic] [--fps 60]
  Default: a NEW daemon session using headless physics and the separate viewer.
  --isolated-vcan | --existing-can | --without-can; --no-terminals
  --config FILE.yaml: select an explicit daemon manifest for a new session
  --attach: add an operator/graph window to a RUNNING session (--shm-name /qhrr05)
  --preview: static model only, no daemon/SHM/physics
  Attach/preview: --frames N --screenshot FILE.ppm --width N --height N --hidden
  --msaa 1|2|4|8 --render-scale 1..2: fixed Filament quality, also for new sessions
  --no-ui: hide operator graphs initially; Tab toggles them
  --control-config FILE: match the running controller's plot/workspace configuration
  Missing executable/runtime assets are built automatically before launch.
  --setup: force preparation of the pinned visualization dependencies
Original development launcher: scripts/run_daemon.sh
USAGE
            exit 0 ;;
        *) echo "unknown visualization launcher option: $1" >&2; exit 1 ;;
    esac
done
case "$QHRR_VISUALIZATION_BACKEND" in filament|classic) ;; *) echo 'backend must be filament or classic' >&2; exit 1 ;; esac
if [[ -n "${QHRR_VISUALIZATION_MSAA:-}" ]]; then
    case "$QHRR_VISUALIZATION_MSAA" in 1|2|4|8) ;; *) echo 'MSAA must be 1, 2, 4 or 8' >&2; exit 1 ;; esac
fi
if [[ -n "${QHRR_VISUALIZATION_RENDER_SCALE:-}" ]]; then
    [[ "$QHRR_VISUALIZATION_RENDER_SCALE" =~ ^(1(\.[0-9]+)?|2(\.0+)?)$ ]] || {
        echo 'render scale must be a finite number from 1 to 2' >&2; exit 1;
    }
    if [[ "$QHRR_VISUALIZATION_BACKEND" == classic && "$QHRR_VISUALIZATION_RENDER_SCALE" != 1 && "$QHRR_VISUALIZATION_RENDER_SCALE" != 1.0 ]]; then
        echo 'render scale is supported only by Filament' >&2; exit 1
    fi
fi
if [[ "$mode" == session ]]; then
    ((${#viewer_args[@]} == 0)) || { echo 'viewer-specific options require --attach or --preview' >&2; exit 1; }
else
    ((${custom_config} == 0)) || { echo '--config requires a new session' >&2; exit 1; }
    ((${#daemon_args[@]} == 0)) || { echo 'daemon options cannot be combined with --attach/--preview' >&2; exit 1; }
fi
source "$project_dir/scripts/ensure_build.sh"
qhrr_ensure_visualization_build "$project_dir" "$setup"
if [[ "$mode" == session ]]; then
    cd "$project_dir"
    exec "$project_dir/scripts/run_daemon.sh" --config "$daemon_config" "${daemon_args[@]}"
fi
if [[ "$mode" == preview ]]; then viewer_args+=(--preview); fi
exec "$project_dir/scripts/visualization_process.sh" "${viewer_args[@]}"
