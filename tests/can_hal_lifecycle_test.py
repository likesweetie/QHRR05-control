"""Run inside a private vcan namespace. Daemon owns SHM and child lifetimes."""

import json
import os
from pathlib import Path
import signal
import socket
import struct
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
        'interface': 'vcan0',
        'frequency_hz': 1000,
        'statistics_hz': 10,
        'bus_load_bitrate': 1000000,
        'motor_ids': [1, 2, 3],
        'mit': {
            'velocity_max_rad_s': 45,
            'torque_max_nm': 53,
            'kt_output_nm_a': 1.44,
            'gear_ratio': 9,
            'kd_scale': 1,
        },
    }
    profile = directory / 'motor profile.json'
    profile.write_text(json.dumps({'mit': settings.pop('mit')}))
    settings['motor_config'] = profile.name
    config.write_text(json.dumps(settings))
    manifest.write_text(
        json.dumps(
            {
                'shm_name': name,
                'monitoring': {'display_interval_sec': 2.0, 'window_sec': 10.0, 'sample_hz': 100.0},
                'processes': [
                    {
                        'name': 'transport-service',
                        'exec_path': str(hal),
                        'args': [str(config), '{shm_name}'],
                        'wait_ready': True,
                    }
                ],
            }
        )
    )

    missing = subprocess.run([str(hal), str(config), name], capture_output=True, text=True)
    assert missing.returncode == 1 and 'shm_open' in missing.stderr, missing.stderr
    assert not shm.exists(), 'HAL must not create a missing shared-memory region'
    print('PASS: standalone HAL refuses a missing daemon-owned SHM')

    for action in ['terminate', 'interrupt', 'hangup', 'collision', 'kill']:
        log_path = directory / f'{action}.log'
        with log_path.open('w') as log:
            process = subprocess.Popen(
                [str(daemon), '--config', str(manifest), '--no-terminals'],
                stdout=log,
                stderr=subprocess.STDOUT,
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
                duplicate = subprocess.run(
                    [str(daemon), '--config', str(manifest), '--no-terminals'],
                    capture_output=True,
                    text=True,
                )
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
            assert not shm.exists(), log_path.read_text()
            print('PASS:', action, 'process supervision, exclusive ownership and cleanup')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)

    for key, bad in [
        ('shm_name', '/obsolete-hal-setting'),
        ('interface', 'no_such_can'),
        ('frequency_hz', 0),
        ('bus_load_bitrate', 0),
        ('bus_load_bitrate', 1000001),
        ('bus_load_bitrate', None),
        ('motor_ids', [1, 1, 3]),
        ('motor_config', 'missing-profile.yaml'),
    ]:
        invalid = dict(settings)
        invalid[key] = bad

        if key == 'motor_config':
            invalid['mit'] = json.loads(profile.read_text())['mit']  # No inline-profile fallback.

        config.write_text(json.dumps(invalid))
        result = subprocess.run(
            [str(daemon), '--config', str(manifest), '--no-terminals'],
            capture_output=True,
            text=True,
            timeout=6,
        )
        assert result.returncode == 1, result.stdout + result.stderr
        assert not shm.exists()
        print('PASS: explicit startup failure for', key)

    # A real C4 mismatch must stop startup instead of accepting a stale/default profile.
    config.write_text(json.dumps(settings))
    log_path = directory / 'rom-mismatch.log'

    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as peer:
        peer.bind(('vcan0',))
        peer.settimeout(0.2)

        with log_path.open('w') as log:
            process = subprocess.Popen(
                [str(daemon), '--config', str(manifest), '--no-terminals'],
                stdout=log,
                stderr=subprocess.STDOUT,
            )

        try:
            deadline = time.monotonic() + 5

            while True:
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()

                try:
                    frame = peer.recv(16)
                except TimeoutError:
                    continue

                identifier, length, payload = struct.unpack('=IB3x8s', frame)
                assert payload[0] not in (0xC0, 0xC1), 'Enabled before ROM verification'

                if payload[0] == 0xC4:
                    # Config expects V_MAX=45; this motor reports 46.
                    reply = bytes([0xC4, 46, 53, 0xA0, 5, 0x84, 3, 1])
                    peer.send(struct.pack('=IB3x8s', identifier, 8, reply))
                    break

            assert process.wait(timeout=5) == 1, log_path.read_text()
            assert 'MIT C4 profile mismatch' in log_path.read_text()
            assert not shm.exists(), log_path.read_text()
            print('PASS: ROM mismatch fails explicitly without a SHM/default profile')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)

    # A peer failure during shutdown must not be masked by an earlier clean exit.
    manifest.write_text(
        json.dumps(
            {
                'shm_name': name,
                'monitoring': {'display_interval_sec': 2.0, 'window_sec': 10.0, 'sample_hz': 100.0},
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
                ],
            }
        )
    )
    result = subprocess.run(
        [str(daemon), '--config', str(manifest), '--no-terminals'],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 7, result.stdout + result.stderr
    print('PASS: generic supervisor propagates peer failure during shutdown')
