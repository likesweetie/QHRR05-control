"""xvfb-run -a -s '-screen 0 1800x1000x24' python3 tests/gui_magnetic_smoke.py"""

import csv
import io
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from PIL import ImageGrab

from daemon_test_config import simulation_command

root = Path(__file__).resolve().parents[1]

with tempfile.TemporaryDirectory(prefix='qhrr05-magnetic-gui-') as tmp:
    log, csv_path = Path(tmp) / 'daemon.log', Path(tmp) / 'telemetry.csv'

    with log.open('w') as stream:
        process = subprocess.Popen(
            simulation_command(tmp, f'/qhrr05-mag-gui-{os.getpid()}', csv_path),
            stdout=stream,
            stderr=subprocess.STDOUT,
        )

    window = None

    def event(*args):
        subprocess.run(['xdotool', *map(str, args)], check=True, capture_output=True)

    def rows():
        if not csv_path.exists():
            return []

        text = csv_path.read_text()

        return list(csv.DictReader(io.StringIO(text[: text.rfind('\n') + 1])))

    def magnetic_rows():
        return [row for row in rows() if row['controller'] == 'magnetic']

    def wait_for(predicate, timeout=12):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            assert process.poll() is None, log.read_text()

            if predicate():
                return

            time.sleep(0.05)

        raise AssertionError(log.read_text())

    def key(value):
        event('key', '--window', window, value)

    def click(x, y):
        event('mousemove', '--window', window, x, y, 'click', 1)

    try:
        deadline = time.monotonic() + 25

        while time.monotonic() < deadline:
            assert process.poll() is None, log.read_text()
            found = subprocess.run(
                ['xdotool', 'search', '--name', 'MuJoCo simulate.*qhrr-05'],
                capture_output=True,
                text=True,
            )

            if found.returncode == 0:
                window = found.stdout.splitlines()[-1]

            if window and 'HOLD:' in log.read_text():
                break

            time.sleep(0.1)

        assert window and 'HOLD:' in log.read_text(), log.read_text()
        time.sleep(0.3)
        click(65, 765)  # Actual native Magnetic button.
        wait_for(lambda: len(magnetic_rows()) > 20)
        before = magnetic_rows()[-1]

        assert before['observer_valid'] == '1', before
        click(225, 795)  # Actual live force slider.
        wait_for(lambda: float(magnetic_rows()[-1]['force_target_n']) > 3.5)
        after = magnetic_rows()[-1]

        assert after['event_seq'] == before['event_seq'], 'force slider restarted the FSM'
        initial_object = [float(after['object_' + axis]) for axis in 'xyz']
        screenshot = ImageGrab.grab()
        pixels = screenshot.load()
        orange = [
            (x, y)
            for y in range(350, 840, 3)
            for x in range(280, 900, 3)
            if pixels[x, y][0] > 120
            and 45 < pixels[x, y][1] < 170
            and pixels[x, y][2] < 90
            and pixels[x, y][0] > 1.4 * pixels[x, y][1]
        ]

        assert len(orange) > 30, 'interactive object is not visible'
        x = (min(p[0] for p in orange) + max(p[0] for p in orange)) // 2
        y = (min(p[1] for p in orange) + max(p[1] for p in orange)) // 2

        event('mousemove', '--window', window, x, y, 'click', '--repeat', 2, '--delay', 120, 1)
        time.sleep(0.3)
        event('windowfocus', window)
        event('keydown', 'Control_L', 'Shift_L')
        time.sleep(0.1)
        event('mousedown', 3)
        preview_saved = False

        for step in range(1, 51):
            event('mousemove', '--window', window, x - 4 * step, y)
            time.sleep(0.04)

            if not preview_saved and magnetic_rows()[-1]['magnetic_phase'] == 'FORCE FOLLOWING':
                geometry = subprocess.check_output(
                    ['xdotool', 'getwindowgeometry', '--shell', window], text=True
                )
                fields = dict(line.split('=', 1) for line in geometry.splitlines() if '=' in line)
                wx, wy, ww, wh = (int(fields[k]) for k in ('X', 'Y', 'WIDTH', 'HEIGHT'))

                ImageGrab.grab().crop((wx, wy, wx + ww, wy + wh)).save(
                    root / 'docs/magnetic_touch_preview.png'
                )
                preview_saved = True

        event('mouseup', 3)
        event('keyup', 'Control_L', 'Shift_L')
        time.sleep(0.5)
        moved = magnetic_rows()[-1]
        displacement = math.sqrt(
            sum(
                (float(moved['object_' + axis]) - initial_object[i]) ** 2
                for i, axis in enumerate('xyz')
            )
        )

        assert displacement > 0.005, ('Ctrl+drag did not move mocap object', displacement)
        assert preview_saved, 'mouse contact did not enter FORCE FOLLOWING'
        key('g')
        wait_for(lambda: '[control] GRAVITY' in log.read_text())
        key('m')
        wait_for(lambda: log.read_text().count('[control] MAGNETIC') >= 2)
        key('r')
        time.sleep(0.3)
        key('Escape')
        assert process.wait(timeout=8) == 0, log.read_text()
        data = magnetic_rows()

        assert any(row['magnetic_phase'] == 'FORCE FOLLOWING' for row in data)
        assert all(0 <= float(row['force_command_n']) <= 5 for row in data)
        assert all(abs(float(row['displacement_m'])) <= 0.015001 for row in data)
        assert 'task_controller exited code=0' in log.read_text(), log.read_text()
        assert 'shm_logger exited code=0' in log.read_text(), log.read_text()
        print(
            'PASS: Magnetic button, live slider without reset, RFOB/GT graphs, object Ctrl+drag, G/M/reset/exit'
        )
        print(
            'object movement [m]:',
            displacement,
            'observed phases:',
            sorted({row['magnetic_phase'] for row in data}),
        )
        print(log.read_text())
    finally:
        event('mouseup', 3)
        event('keyup', 'Control_L', 'Shift_L')

        if process.poll() is None:
            process.send_signal(signal.SIGTERM)

            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
