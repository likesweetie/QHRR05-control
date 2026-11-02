"""Prepare the existing CAN interface selected by the hardware daemon manifest."""

import json
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


def run(command):
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode:
        reason = result.stderr.strip() or result.stdout.strip() or f'exit {result.returncode}'
        raise RuntimeError(f'{" ".join(command)}: {reason}')
    return result.stdout


def read_link(name):
    links = json.loads(run(['ip', '-json', '-details', 'link', 'show', 'dev', name]))
    if len(links) != 1 or links[0].get('ifname') != name:
        raise RuntimeError(f'CAN interface not found: {name}')
    return links[0]


def admin_ip():
    for line in Path('/proc/self/status').read_text().splitlines():
        if line.startswith('CapEff:') and int(line.split()[1], 16) & (1 << 12):
            return ['ip']
    if not shutil.which('sudo'):
        raise RuntimeError('CAN setup requires CAP_NET_ADMIN or sudo, but sudo is unavailable')
    print('[launcher] CAN setup requires sudo; only ip link commands are elevated.', flush=True)
    return ['sudo', '--', 'ip']


def bitrate(link):
    return link.get('linkinfo', {}).get('info_data', {}).get('bittiming', {}).get('bitrate')


def prepare(manifest_path):
    manifest_path = Path(manifest_path).resolve()
    manifest = yaml.safe_load(manifest_path.read_text())
    if manifest.get('runtime_mode') != 'hardware':
        raise ValueError('run_daemon.sh requires runtime_mode: hardware; use run_simulation.sh for simulation')
    hal = next((p for p in manifest['processes'] if p['name'] == 'can_hal'), None)
    if not hal:
        raise ValueError('hardware manifest requires can_hal')
    workdir = (manifest_path.parent / hal.get('working_directory', '.')).resolve()
    hal_path = (workdir / hal['args'][0]).resolve()
    config = yaml.safe_load(hal_path.read_text())
    name = config['interface']
    if not isinstance(name, str) or not name or len(name.encode()) >= 16 or name.startswith('-'):
        raise ValueError('CAN interface must be a nonempty Linux interface name shorter than 16 bytes')
    expected = config.get('bitrate')
    link = read_link(name)
    kind = link.get('linkinfo', {}).get('info_kind')
    if kind not in ('can', 'vcan'):
        raise ValueError(f'Configured interface {name} is not CAN/vcan; refusing to change it')
    if kind == 'can' and (type(expected) is not int or not 1 <= expected <= 1000000):
        raise ValueError('CAN bitrate must be an integer in [1, 1000000] bit/s')
    is_up = 'UP' in link.get('flags', [])
    change_bitrate = kind == 'can' and bitrate(link) != expected
    if not is_up or change_bitrate:
        command = admin_ip() + ['link', 'set', 'dev', name]
        if change_bitrate:
            print(f'[launcher] Preparing {name}: bitrate {bitrate(link)} -> {expected} bit/s', flush=True)
            if is_up:
                run(command + ['down'])
            run(command + ['type', 'can', 'bitrate', str(expected)])
        print(f'[launcher] Bringing {name} UP', flush=True)
        run(command + ['up'])
        link = read_link(name)
        if link.get('linkinfo', {}).get('info_kind') != kind or 'UP' not in link.get('flags', []):
            raise RuntimeError(f'CAN interface {name} did not become UP')
        if kind == 'can' and bitrate(link) != expected:
            raise RuntimeError(f'CAN bitrate mismatch on {name}: expected {expected}, got {bitrate(link)}')
    speed = f' bitrate={expected}' if kind == 'can' else ''
    print(f'[launcher] Hardware pipeline | interface={name} kind={kind} state=UP{speed}', flush=True)


if __name__ == '__main__':
    try:
        if len(sys.argv) != 2:
            raise ValueError('usage: prepare_can.py DAEMON_CONFIG.yaml')
        prepare(sys.argv[1])
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as error:
        sys.exit(f'[launcher] CAN preparation failed: {error}')
