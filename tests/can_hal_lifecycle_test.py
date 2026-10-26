"""Run inside a private vcan namespace. HAL owns SHM; daemon knows only children."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

hal, daemon = map(Path, sys.argv[1:])

with tempfile.TemporaryDirectory(prefix='qhrr05-can-life-') as directory:
    directory = Path(directory)
    name = f'/qhrr05-can-life-{os.getpid()}'
    shm = Path('/dev/shm') / name[1:]
    config = directory / 'can.json'
    manifest = directory / 'processes.json'
    settings = {
        'shm_name': name,
        'interface': 'vcan0',
        'frequency_hz': 1000,
        'statistics_hz': 10,
        'motor_ids': [1, 2, 3],
        'mit': {
            'velocity_max_rad_s': 45,
            'torque_max_nm': 53,
            'kt_output_nm_a': 1.44,
            'gear_ratio': 9,
            'kd_scale': 1,
        },
    }
    config.write_text(json.dumps(settings))
    manifest.write_text(
        json.dumps(
            {
                'processes': [
                    {
                        'name': 'transport-service',
                        'exec_path': str(hal),
                        'args': [str(config)],
                        'wait_ready': True,
                    }
                ]
            }
        )
    )

    for action in ['terminate', 'interrupt', 'hangup', 'collision', 'kill']:
        log_path = directory / f'{action}.log'
        with log_path.open('w') as log:
            process = subprocess.Popen(
                [str(daemon), '--config', str(manifest)], stdout=log, stderr=subprocess.STDOUT
            )
        child = None
        try:
            deadline = time.monotonic() + 5
            while child is None:
                contents = log_path.read_text()
                for line in contents.splitlines():
                    if 'transport-service started pid=' in line:
                        child = int(line.rsplit('=', 1)[1])
                assert process.poll() is None, contents
                assert time.monotonic() < deadline, contents
                time.sleep(0.01)

            assert shm.exists()
            time.sleep(0.15)
            assert 'MIT position / tau_ff=0' in log_path.read_text()

            if action == 'collision':
                original = shm.stat().st_ino
                duplicate = subprocess.run([str(hal), str(config)], capture_output=True, text=True)
                assert duplicate.returncode == 1 and 'File exists' in duplicate.stderr
                assert shm.stat().st_ino == original
                os.kill(process.pid, signal.SIGTERM)
            elif action == 'kill':
                os.kill(child, signal.SIGKILL)
            elif action == 'hangup':
                os.kill(process.pid, signal.SIGHUP)
            elif action == 'interrupt':
                os.kill(process.pid, signal.SIGINT)
            else:
                os.kill(process.pid, signal.SIGTERM)

            code = process.wait(timeout=5)
            assert (code != 0) == (action == 'kill'), log_path.read_text()
            if action == 'kill':
                # A killed owner cannot run its destructor. Never silently reuse it.
                assert shm.exists()
                duplicate = subprocess.run([str(hal), str(config)], capture_output=True, text=True)
                assert duplicate.returncode == 1 and 'File exists' in duplicate.stderr
                shm.unlink()  # Explicit cleanup of this test's unique object only.
            else:
                assert not shm.exists(), log_path.read_text()
            print('PASS:', action, 'process supervision, exclusive ownership and cleanup')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)

    for key, bad in [('interface', 'no_such_can'), ('frequency_hz', 0), ('motor_ids', [1, 1, 3])]:
        invalid = dict(settings)
        invalid[key] = bad
        config.write_text(json.dumps(invalid))
        result = subprocess.run(
            [str(daemon), '--config', str(manifest)], capture_output=True, text=True, timeout=6
        )
        assert result.returncode == 1, result.stdout + result.stderr
        assert not shm.exists()
        print('PASS: explicit startup failure for', key)

    # A peer failure during shutdown must not be masked by an earlier clean exit.
    manifest.write_text(
        json.dumps(
            {
                'processes': [
                    {
                        'name': 'peer',
                        'exec_path': '/bin/sh',
                        'wait_ready': True,
                        'args': [
                            '-c',
                            'trap "exit 7" TERM; printf R >&"$QHRR_PROCESS_READY_FD"; while :; do sleep 0.1; done',
                        ],
                    },
                    {'name': 'completed', 'exec_path': '/bin/true', 'args': []},
                ]
            }
        )
    )
    result = subprocess.run(
        [str(daemon), '--config', str(manifest)], capture_output=True, text=True, timeout=5
    )
    assert result.returncode == 7, result.stdout + result.stderr
    print('PASS: generic supervisor propagates peer failure during shutdown')
