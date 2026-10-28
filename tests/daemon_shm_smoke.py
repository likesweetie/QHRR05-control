"""Integration checks for SHM ownership, CSV consumer, child failure, and shutdown."""

import csv
import ctypes
import errno
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import yaml

from daemon_test_config import simulation_command

root = Path(__file__).resolve().parents[1]
libc = ctypes.CDLL(None, use_errno=True)


def assert_unlinked(name):
    fd = libc.shm_open(name.encode(), os.O_RDWR, 0o600)

    if fd >= 0:
        os.close(fd)

    assert fd < 0 and ctypes.get_errno() == errno.ENOENT, f'owned SHM remains: {name}'


with tempfile.TemporaryDirectory(prefix='qhrr05-daemon-shm-') as tmp:
    name = f'/qhrr05-daemon-test-{os.getpid()}'
    # This smoke uses no Tracking motion, but all modes need a model for path validation.
    # The copied YAML lives in a different directory, so resolve its model path explicitly.
    control = yaml.safe_load((root / 'tests/fixtures/control.yaml').read_text())
    control['model_path'] = str(root / 'models/qhrr-05/qhrr-05.xml')
    control['tracking']['mode'] = 'impedance'
    control['ball_following']['enabled'] = False
    control_path = Path(tmp) / 'control.yaml'
    control_path.write_text(yaml.safe_dump(control))
    command = simulation_command(
        tmp, name, Path(tmp) / 'telemetry.csv', duration=100000, control=control_path
    )
    csv_path = Path(tmp) / 'telemetry.csv'
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        cwd=tmp,
    )

    # Accelerated physics can finish before the controller loads its model.
    # Wait for recorded publications rather than assuming a fixed startup speed.
    ready = False
    try:
        deadline = time.monotonic() + 10
        while process.poll() is None and time.monotonic() < deadline:
            if csv_path.exists():
                text = csv_path.read_text()
                complete = text[:text.rfind('\n') + 1]
                observed = list(csv.DictReader(complete.splitlines()))
                ready = len(observed) > 1 and any(int(row['command_seq']) > 0 for row in observed)
                if ready:
                    break
            time.sleep(.01)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
        try:
            output, _ = process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
            raise AssertionError(output)

    result = subprocess.CompletedProcess(command, process.returncode, output, '')
    assert ready and result.returncode == 0, result.stdout

    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) > 1, result.stdout
    sequences = [int(row['sample_seq']) for row in rows]

    assert all(a < b for a, b in zip(sequences, sequences[1:]))
    assert sum(int(row['skipped_samples']) for row in rows) == sequences[-1] - len(rows)
    assert any(row['state_seq'] != row['command_seq'] for row in rows)
    assert all(
        (not row['control_latency_us'] or float(row['control_latency_us']) >= 0)
        and float(row['sample_age_us']) >= 0
        for row in rows
    )
    assert 'shm_logger exited code=0' in result.stdout, result.stdout
    assert_unlinked(name)
    print(
        f'PASS: headless SHM CSV, {len(rows)} rows, {sequences[-1]-len(rows)} skipped samples explicitly reported'
    )

    original = csv_path.read_bytes()
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=15,
    )

    assert result.returncode != 0, result.stdout + result.stderr
    assert csv_path.read_bytes() == original
    assert_unlinked(name)
    print('PASS: CSV collision fails explicitly and owned SHM is cleaned up')

    fd = libc.shm_open(name.encode(), os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o600)

    assert fd >= 0

    try:
        os.write(fd, b'pre-existing-shm')
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)

        assert result.returncode != 0
        assert os.fstat(fd).st_size == len(b'pre-existing-shm')
        os.lseek(fd, 0, os.SEEK_SET)
        assert os.read(fd, 100) == b'pre-existing-shm'
        check = libc.shm_open(name.encode(), os.O_RDWR, 0o600)

        assert check >= 0, 'daemon unlinked a SHM it did not create'
        os.close(check)
    finally:
        os.close(fd)
        libc.shm_unlink(name.encode())

    print('PASS: existing SHM is rejected without reuse, resize, or unlink')

    for action in ['kill_controller', 'stop_daemon']:
        process = subprocess.Popen(
            simulation_command(tmp, name, duration=100000, control=control_path),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        output = []
        controller = None

        try:
            for line in process.stdout:
                output.append(line)
                match = re.search(r'control started pid=(\d+)', line)

                if match:
                    controller = int(match.group(1))

                if controller:
                    break

            assert controller and process.poll() is None, ''.join(output)

            if action == 'kill_controller':
                os.kill(controller, signal.SIGKILL)
            else:
                process.send_signal(signal.SIGTERM)

            tail, _ = process.communicate(timeout=8)

            output.append(tail)
            assert (process.returncode != 0) == (action == 'kill_controller'), ''.join(output)
            assert_unlinked(name)
            print(f'PASS: {action}, peer shutdown and SHM unlink')
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)

                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
