import csv
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
from PIL import ImageGrab

root = Path(__file__).resolve().parents[1]

with tempfile.TemporaryDirectory(prefix='qhrr05-gui-') as tmp:
    log = Path(tmp) / 'daemon.log'
    csv_path = Path(tmp) / 'telemetry.csv'

    with log.open('w') as stream:
        process = subprocess.Popen(
            [
                str(root / 'build/daemon'),
                '--shm-name',
                f'/qhrr05-gui-{os.getpid()}',
                '--csv',
                str(csv_path),
            ],
            cwd='/tmp',
            stdout=stream,
            stderr=subprocess.STDOUT,
        )

    window = None

    def contents():
        return log.read_text()

    def key(value):
        subprocess.run(
            ['xdotool', 'key', '--window', window, value], check=True, capture_output=True
        )

    def wait_for(predicate, timeout=8):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(contents())

            if predicate():
                return

            time.sleep(0.05)

        raise AssertionError(contents())

    def click(x, y):
        subprocess.run(
            ['xdotool', 'mousemove', '--window', window, str(x), str(y), 'click', '1'],
            check=True,
            capture_output=True,
        )

    try:
        deadline = time.monotonic() + 25

        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(contents())

            found = subprocess.run(
                ['xdotool', 'search', '--name', 'MuJoCo simulate.*qhrr-05'],
                capture_output=True,
                text=True,
            )

            if found.returncode == 0:
                window = found.stdout.splitlines()[-1]

            if window and 'HOLD:' in contents():
                break

            time.sleep(0.1)

        assert window and 'HOLD:' in contents(), contents()
        time.sleep(0.5)
        key('space')
        time.sleep(0.4)
        geometry = subprocess.check_output(
            ['xdotool', 'getwindowgeometry', '--shell', window], text=True
        )
        fields = dict(line.split('=', 1) for line in geometry.splitlines() if '=' in line)
        x, y, w, h = (int(fields[k]) for k in ('X', 'Y', 'WIDTH', 'HEIGHT'))

        ImageGrab.grab().crop((x, y, x + w, y + h)).save(root / 'docs/homing_preview.png')
        # Exercise the actual native Simulate buttons, then keyboard shortcuts.
        click(185, 735)  # Simulation > Controller > Gravity [G], default 100% font.
        wait_for(lambda: '[control] GRAVITY' in contents())
        key('space')
        time.sleep(1)
        key('space')
        time.sleep(0.2)
        ImageGrab.grab().crop((x, y, x + w, y + h)).save(root / 'docs/gravity_preview.png')
        click(65, 735)  # Homing [H].
        wait_for(lambda: '[control] HOMING' in contents())
        key('space')
        wait_for(lambda: contents().count('HOLD:') >= 2)
        key('g')
        wait_for(lambda: contents().count('[control] GRAVITY') >= 2)
        time.sleep(0.3)
        key('h')
        wait_for(lambda: contents().count('[control] HOMING') >= 2)
        time.sleep(0.3)
        key('space')
        key('r')
        time.sleep(0.2)
        key('space')
        time.sleep(0.7)
        key('Escape')
        code = process.wait(timeout=8)

        assert code == 0, contents()
        rows = list(csv.DictReader(csv_path.open()))

        assert len(rows) > 100, len(rows)
        assert {row['run_id'] for row in rows} == {'0', '1'}
        first_run = [row for row in rows if row['run_id'] == '0']

        assert any(
            float(b['control_time']) - float(a['control_time'])
            > float(b['sim_time']) - float(a['sim_time']) + 0.1
            for a, b in zip(first_run, first_run[1:])
        ), 'controller time stopped during GUI pause'
        goal = [0, 0.9, -1.8]
        joints = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint']
        # Last sample of initial homing, before the first gravity transition.
        last = first_run[
            next(i for i, row in enumerate(first_run) if row['controller'] == 'gravity') - 1
        ]
        error = max(abs(float(last['q_' + j]) - g) for j, g in zip(joints, goal))

        assert error < 0.04, error  # Pure PD retains gravity deflection (~0.026 rad).
        gravity_rows = [row for row in first_run if row['controller'] == 'gravity']

        assert len(gravity_rows) > 50
        # Both gravity entries capture the current pose rather than commanding home.
        assert all(
            abs(float(row['q_ref_' + j]) - float(row['q_' + j])) < 0.03
            for row in gravity_rows
            for j in joints
        )
        assert 'task_controller exited code=0' in contents(), contents()
        assert 'shm_logger exited code=0' in contents(), contents()
        assert any(row['state_seq'] != row['command_seq'] for row in rows)
        assert all(
            not row['control_latency_us'] or float(row['control_latency_us']) >= 0 for row in rows
        )
        print(
            'PASS: native homing/gravity buttons, H/G shortcuts, plots, pause/reset/ESC, CSV mode/run IDs'
        )
        print('GUI pure PD home error:', error)
        print(contents())
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)

            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
