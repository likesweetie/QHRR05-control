"""Build explicit process manifests; never edit the user's controller tuning."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def simulation_command(directory, shm_name, csv_path=None, duration=None, control=None):
    control = Path(control) if control else ROOT / 'tests/fixtures/control.yaml'
    args = [
        '--without-can',
        '--model',
        str(ROOT / 'models/qhrr-05/scene.xml'),
        '--control-config',
        str(control),
        '--shm-name',
        '{shm_name}',
    ]

    if duration is not None:
        args += ['--headless', '--duration', str(duration)]

    def process(name, arguments, ready=False):
        return {
            'name': name,
            'exec_path': str(ROOT / 'build' / name),
            'args': arguments,
            'wait_ready': ready,
        }

    processes = [
        process('simulation', args, True),
        process('control', [str(control), '{shm_name}', '--without-can']),
    ]

    if csv_path:
        processes.append(process('shm_logger', ['{shm_name}', str(csv_path)]))

    path = Path(directory) / 'processes.json'
    path.write_text(
        json.dumps(
            {
                'shm_name': shm_name,
                'monitoring': {'display_interval_sec': 2.0, 'window_sec': 10.0, 'sample_hz': 100.0},
                'debug_processes': processes,
            }
        )
    )
    return [str(ROOT / 'build/daemon'), '--config', str(path), '--without-can', '--no-terminals']
