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
        'command_timeout_sec': 0.10,
        'feedback_timeout_sec': 0.25,
        'enable_timeout_sec': 1.0,
        'poll_released': False,
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
                'runtime_mode': 'simulation', 'shm_name': name,
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
            assert 'CAN HAL | vcan0 | MIT C0 / RMD A1' in log_path.read_text()

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

    # Reproduce the physical 1.298 vs C5-quantized 1.296 case. Both exact
    # representations are valid before writing; neither needs another ROM write.
    config.write_text(json.dumps(settings))
    original_profile = profile.read_text()
    requested = json.loads(original_profile)
    requested['mit']['torque_max_nm'] = 33
    requested['mit']['kt_output_nm_a'] = 1.298
    profile.write_text(json.dumps(requested))
    for label, kt_raw in [('configured', 1298), ('previously_written', 1296)]:
        log_path = directory / f'existing-{label}.log'
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as peer:
            peer.bind(('vcan0',))
            peer.settimeout(0.1)
            with log_path.open('w') as log:
                process = subprocess.Popen(
                    [str(daemon), '--config', str(manifest), '--no-terminals'],
                    stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 5
            try:
                while log_path.read_text().count('C5 skipped motor_id=') < 3:
                    assert process.poll() is None, log_path.read_text()
                    assert time.monotonic() < deadline, log_path.read_text()
                    try:
                        identifier, length, payload = struct.unpack('=IB3x8s', peer.recv(16))
                    except TimeoutError:
                        continue
                    assert payload[0] not in (0xC0, 0xC1, 0xC5), payload.hex()
                    if payload[0] == 0xC2:
                        reply = bytes([0xC2]) + bytes(7)
                    elif payload[0] == 0xC4:
                        reply = bytes([0xC4, 45, 33, kt_raw & 255, kt_raw >> 8, 0x84, 3, 1])
                    else:
                        raise AssertionError(payload.hex())
                    peer.send(struct.pack('=IB3x8s', identifier, 8, reply))
                process.terminate()
                assert process.wait(timeout=5) == 0, log_path.read_text()
                assert not shm.exists()
                print('PASS: exact', label, 'C4 starts without C5 or drive activation')
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
    profile.write_text(original_profile)

    # A real C4 mismatch must be written and then verified, never accepted.
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

                if payload[0] == 0xC2:
                    peer.send(struct.pack('=IB3x8s', identifier, 8, bytes([0xC2]) + bytes(7)))
                elif payload[0] == 0xC4:
                    reply = bytes([0xC4, 46, 54, 0xDC, 5, 0xE8, 3, 2])
                    peer.send(struct.pack('=IB3x8s', identifier, 8, reply))
                elif payload[0] == 0xC5:
                    assert payload == bytes([0xC5, 45, 53, 160, 0, 0x84, 3, 1]), payload
                    # Report different values for every field, including LE/scaled ones.
                    reply = bytes([0xC4, 46, 54, 0xDC, 5, 0xE8, 3, 2])
                    peer.send(struct.pack('=IB3x8s', identifier, 8, reply))
                    break

            assert process.wait(timeout=5) == 1, log_path.read_text()
            contents = log_path.read_text()
            assert 'MIT C4 profile mismatch; refusing enable' in contents, contents
            assert 'C5 verification failed' in contents and 'FOC OFF' in contents, contents
            axis = ['HR', 'HP', 'KP'][identifier - 0x141]
            assert f'interface=vcan0 axis={axis} motor_id={identifier - 0x140} SID=0x{identifier:X}' in contents, contents
            assert str(profile) in contents, contents
            for detail in [
                'velocity_max_rad_s (V_MAX, rad/s): actual=46 expected=45 [MISMATCH]',
                'torque_max_nm (T_MAX, Nm): actual=54 expected=53 [MISMATCH]',
                'kt_output_nm_a (Kt_out, Nm/A): actual=1.500 expected=1.440 [MISMATCH]',
                'gear_ratio: actual=10.00 expected=9.00 [MISMATCH]',
                'kd_scale: actual=2 expected=1 [MISMATCH]',
                'actual bytes: C4 2E 36 DC 05 E8 03 02',
                'expected bytes: C4 2D 35 A0 05 84 03 01',
            ]:
                assert detail in contents, contents
            assert not shm.exists(), log_path.read_text()
            print('PASS: rejected C5 write reports motor, config, decoded differences and raw bytes')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)

    # A lost C5 reply may trigger read retries, never repeated EEPROM writes.
    log_path = directory / 'c5-timeout.log'
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as peer:
        peer.bind(('vcan0',))
        peer.settimeout(0.1)
        with log_path.open('w') as log:
            process = subprocess.Popen(
                [str(daemon), '--config', str(manifest), '--no-terminals'],
                stdout=log, stderr=subprocess.STDOUT)
        writes = [0, 0, 0]
        reads = 0
        deadline = time.monotonic() + 5
        try:
            while process.poll() is None:
                assert time.monotonic() < deadline, log_path.read_text()
                try:
                    identifier, length, payload = struct.unpack('=IB3x8s', peer.recv(16))
                except TimeoutError:
                    continue
                assert payload[0] not in (0xC0, 0xC1), 'Enabled before C5 verification'
                if payload[0] == 0xC2:
                    peer.send(struct.pack('=IB3x8s', identifier, 8, bytes([0xC2]) + bytes(7)))
                elif payload[0] == 0xC5:
                    writes[identifier - 0x141] += 1
                elif payload[0] == 0xC4:
                    reads += 1
                    if not writes[identifier - 0x141]:
                        reply = bytes([0xC4, 46, 54, 0xDC, 5, 0xE8, 3, 2])
                        peer.send(struct.pack('=IB3x8s', identifier, 8, reply))
            assert process.returncode == 1, log_path.read_text()
            assert writes == [1, 1, 1] and reads > 0, (writes, reads)
            assert 'C5/C4 verification timeout' in log_path.read_text()
            assert not shm.exists()
            print('PASS: C5 timeout retries only reads, blocks enable and cleans up')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)

    # A peer failure during shutdown must not be masked by an earlier clean exit.
    manifest.write_text(
        json.dumps(
            {
                'runtime_mode': 'simulation', 'shm_name': name,
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
