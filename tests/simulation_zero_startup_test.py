"""Run the real daemon with delayed enable and check CAN/model coordinates."""

import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile

import yaml
from daemon_test_config import default_simulation_manifest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from log_data import snapshots


with tempfile.TemporaryDirectory(prefix='qhrr05-zero-startup-') as directory:
    directory = Path(directory)
    name = f'/qhrr05-zero-startup-{os.getpid()}'
    control = yaml.safe_load((ROOT / 'config/control/control.yaml').read_text())
    control['model_path'] = str(ROOT / 'models/qhrr-05/qhrr-05.xml')
    control_path = directory / 'control.yaml'
    control_path.write_text(yaml.safe_dump(control))
    settings = yaml.safe_load((ROOT / 'config/simulation/simulation.yaml').read_text())
    settings['motor']['require_zero_position_setup'] = False
    simulation_path = directory / 'simulation.yaml'
    simulation_path.write_text(yaml.safe_dump(settings))
    logger = yaml.safe_load((ROOT / 'config/shm_logger/shm_logger.yaml').read_text())
    logger['logging']['output_dir'] = str(directory / 'recordings')
    logger['logging']['storage']['minimum_free_gib'] = 0
    logger['logging']['flight_recorder']['enabled'] = False
    logger_path = directory / 'logger.yaml'
    logger_path.write_text(yaml.safe_dump(logger))
    delayed = directory / 'delayed-control.sh'
    delayed.write_text('#!/bin/sh\nsleep 1\nexec "$@"\n')
    delayed.chmod(0o700)
    manifest = default_simulation_manifest(directory / 'default-session')
    hal_path = directory / 'default-session/can.yaml'
    manifest['shm_name'] = name
    manifest['log_directory'] = str(directory / 'logs')
    manifest['processes'] = [p for p in manifest['processes'] if p['name'] != 'input']

    for spec in manifest['processes']:
        spec['exec_path'] = str(ROOT / 'build' / spec['exec_path'])
        spec['working_directory'] = str(ROOT)
        if spec['name'] == 'simulation':
            spec['args'] = [
                '--model',
                str(ROOT / 'models/qhrr-05/scene.xml'),
                '--control-config',
                str(control_path),
                '--shm-name',
                '{shm_name}',
                '--hal-config',
                str(hal_path),
                '--simulation-config',
                str(simulation_path),
                '--headless',
                '--duration',
                '4',
            ]
        elif spec['name'] == 'can_hal':
            spec['args'] = [str(hal_path), '{shm_name}']
        elif spec['name'] == 'control':
            spec['exec_path'] = str(delayed)
            spec['args'] = [str(ROOT / 'build/control'), str(control_path), '{shm_name}']
        elif spec['name'] == 'shm_logger':
            spec['args'] = [
                '--config',
                str(logger_path),
                '--shm-name',
                '{shm_name}',
                '--daemon-config',
                '{daemon_config}',
            ]

    manifest_path = directory / 'daemon.yaml'
    manifest_path.write_text(yaml.safe_dump(manifest))
    result = subprocess.run(
        [str(ROOT / 'build/daemon'), '--config', str(manifest_path), '--no-terminals'],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=18,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    sessions = list((directory / 'recordings').glob('*'))
    assert len(sessions) == 1, sessions
    errors = [[], [], []]
    first_pose = None
    forces = set()
    count = 0

    for stamp, channel, values in snapshots(sessions[0]):
        if channel != 'qhrr/state':
            continue
        forces.add(values['control/target_force'])
        if not all(values[f'motor/received_sequence[{i}]'] > 10 for i in range(3)):
            continue
        count += 1
        if first_pose is None:
            first_pose = [values[f'simulation/ground_truth/position_gt[{i}]'] for i in range(3)]
        for i in range(3):
            if abs(values[f'motor/feedback[{i}]/velocity']) < 0.2:
                measured = (
                    values[f'motor/position_offset[{i}]'] + values[f'motor/feedback[{i}]/position']
                )
                errors[i].append(measured - values[f'simulation/ground_truth/position_gt[{i}]'])

    assert count > 100, count
    initial_pose = control['homing']['initial_position_rad']
    displacement = [position - initial for position, initial in zip(first_pose, initial_pose)]
    assert max(abs(value) for value in displacement) > 0.1, (
        f'Test did not exercise motion before enable: {displacement}'
    )
    assert all(len(axis) > 10 for axis in errors), [len(axis) for axis in errors]
    medians = [statistics.median(axis) for axis in errors]
    assert all(abs(error) < 0.002 for error in medians), f'Persistent encoder bias: {medians}'
    assert forces == {control['magnetic_touch']['target_force_n']}, forces
    assert not Path('/dev/shm', name[1:]).exists(), 'Daemon left test SHM behind'
    print(
        'PASS: delayed-enable model zero matches physics; median error rad=',
        medians,
        'pre-enable joint displacement rad=',
        displacement,
        'configured force=',
        forces,
    )
