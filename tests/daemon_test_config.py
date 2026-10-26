"""Build explicit process manifests; never edit the user's controller tuning."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def simulation_command(directory, shm_name, csv_path=None, duration=None, control=None):
    control = Path(control) if control else ROOT / 'tests/fixtures/control_config.yaml'
    args = [
        '--create-shm',
        '--without-can',
        '--model',
        str(ROOT / 'models/qhrr-05/scene.xml'),
        '--control-config',
        str(control),
        '--shm-name',
        shm_name,
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
        process('mujoco_simulate', args, True),
        process('task_controller', [str(control), shm_name, '--without-can']),
    ]

    if csv_path:
        processes.append(process('shm_logger', [shm_name, str(csv_path)]))

    path = Path(directory) / 'processes.json'
    path.write_text(json.dumps({'debug_processes': processes}))
    return [str(ROOT / 'build/daemon'), '--config', str(path), '--without-can']
