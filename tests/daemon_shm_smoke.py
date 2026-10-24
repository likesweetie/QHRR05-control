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

root = Path(__file__).resolve().parents[1]
libc = ctypes.CDLL(None, use_errno=True)


def assert_unlinked(name):
    fd = libc.shm_open(name.encode(), os.O_RDWR, 0o600)
    if fd >= 0:
        os.close(fd)
    assert fd < 0 and ctypes.get_errno() == errno.ENOENT, f'owned SHM remains: {name}'


with tempfile.TemporaryDirectory(prefix='qhrr05-daemon-shm-') as tmp:
    name = f'/qhrr05-daemon-test-{os.getpid()}'
    command = [str(root/'build/daemon'), '--shm-name', name]
    csv_path = Path(tmp)/'telemetry.csv'
    result = subprocess.run(command+['--headless', '--duration', '4', '--csv', str(csv_path)],
                            capture_output=True, text=True, timeout=15, cwd=tmp)
    assert result.returncode == 0, result.stdout+result.stderr
    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) > 1, result.stdout
    sequences = [int(row['sample_seq']) for row in rows]
    assert all(a < b for a, b in zip(sequences, sequences[1:]))
    assert sum(int(row['skipped_samples']) for row in rows) == sequences[-1]-len(rows)
    assert any(row['state_seq'] != row['command_seq'] for row in rows)
    assert all((not row['control_latency_us'] or float(row['control_latency_us']) >= 0)
               and float(row['sample_age_us']) >= 0 for row in rows)
    assert 'shm_logger exited code=0' in result.stdout, result.stdout
    assert_unlinked(name)
    print(f'PASS: headless SHM CSV, {len(rows)} rows, {sequences[-1]-len(rows)} skipped samples explicitly reported')

    original = csv_path.read_bytes()
    result = subprocess.run(command+['--headless', '--duration', '4', '--csv', str(csv_path)],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode != 0, result.stdout+result.stderr
    assert csv_path.read_bytes() == original
    assert_unlinked(name)
    print('PASS: CSV collision fails explicitly and owned SHM is cleaned up')

    fd = libc.shm_open(name.encode(), os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o600)
    assert fd >= 0
    try:
        os.write(fd, b'pre-existing-shm')
        result = subprocess.run(command+['--headless', '--duration', '.1'],
                                capture_output=True, text=True, timeout=15)
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
        process = subprocess.Popen(command+['--headless', '--duration', '100000'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        output = []
        controller = None
        try:
            for line in process.stdout:
                output.append(line)
                match = re.search(r'task_controller started pid=(\d+)', line)
                if match:
                    controller = int(match.group(1))
                if '[simulate] qhrr-05 viewer' in line:
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
