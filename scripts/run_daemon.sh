#!/usr/bin/env bash
# Default physical-CAN entry point. The manifest explicitly chooses the device.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
daemon_config="$project_dir/config/daemon/daemon.yaml"
daemon_args=()
while (($#)); do
    case "$1" in
        --config) daemon_config="$(realpath -- "${2:?--config requires a path}")"; shift 2 ;;
        --no-terminals) daemon_args+=(--no-terminals); shift ;;
        --help)
            echo 'usage: run_daemon.sh [--config YAML] [--no-terminals]'
            echo 'Physical CAN, control processes and web GUI. For MuJoCo use run_simulation.sh.'
            exit 0 ;;
        *) echo "Unknown hardware option: $1. For simulation use scripts/run_simulation.sh." >&2; exit 1 ;;
    esac
done
# This path never creates or changes network interfaces, and never launches a simulator.
python3 - "$daemon_config" <<'PYCODE'
import json, pathlib, subprocess, sys, yaml
manifest_path = pathlib.Path(sys.argv[1])
manifest = yaml.safe_load(manifest_path.read_text())
if manifest.get('runtime_mode') != 'hardware':
    sys.exit('run_daemon.sh requires runtime_mode: hardware; use run_simulation.sh for simulation')
hal = next((p for p in manifest['processes'] if p['name'] == 'can_hal'), None)
if not hal:
    sys.exit('hardware manifest requires can_hal')
workdir = (manifest_path.parent / hal.get('working_directory', '.')).resolve()
hal_path = (workdir / hal['args'][0]).resolve()
config = yaml.safe_load(hal_path.read_text())
name = config['interface']
links = json.loads(subprocess.check_output(['ip', '-json', '-details', 'link', 'show', 'dev', name]))
link = links[0]
kind = link.get('linkinfo', {}).get('info_kind')
if kind not in ('can', 'vcan') or 'UP' not in link.get('flags', []):
    sys.exit(f'CAN interface {name} must be CAN/vcan and UP; no automatic replacement is performed')
if kind == 'can':
    actual = link['linkinfo'].get('info_data', {}).get('bittiming', {}).get('bitrate')
    if actual != config['bitrate']:
        sys.exit(f'CAN bitrate mismatch on {name}: expected {config["bitrate"]}, got {actual}')
print(f'[launcher] Hardware pipeline | interface={name} kind={kind}; existing interface unchanged', flush=True)
PYCODE
source "$project_dir/scripts/ensure_build.sh"
qhrr_ensure_native_build "$project_dir" daemon control can_hal input shm_logger qhrr_logctl web_gui_bridge
exec "$project_dir/build/daemon" --config "$daemon_config" "${daemon_args[@]}"
