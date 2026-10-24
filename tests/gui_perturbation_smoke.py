"""Run with xvfb-run -a -s '-screen 0 1800x1000x24' python3 tests/gui_perturbation_smoke.py."""
import csv
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

root = Path(__file__).resolve().parents[1]


def event(*args):
    subprocess.run(['xdotool', *map(str, args)], check=True, capture_output=True)


# Click points on the calf with the native default camera/UI font and graph panel.
# The second case also verifies selection after resizing the window.
for button, width, height, calf_x, calf_y in [(3, 1600, 900, 750, 565),
                                            (1, 1400, 800, 672, 502)]:
    with tempfile.TemporaryDirectory(prefix='qhrr05-drag-') as tmp:
        log = Path(tmp)/'daemon.log'
        csv_path = Path(tmp)/'drag.csv'
        with log.open('w') as stream:
            process = subprocess.Popen([str(root/'build/daemon'), '--shm-name', f'/qhrr05-drag-{os.getpid()}-{button}', '--csv', str(csv_path)],
                                       stdout=stream, stderr=subprocess.STDOUT, cwd='/tmp')
        try:
            window = None
            deadline = time.monotonic()+25
            while time.monotonic() < deadline:
                assert process.poll() is None, log.read_text()
                found = subprocess.run(['xdotool', 'search', '--name', 'MuJoCo simulate.*qhrr-05'],
                                       capture_output=True, text=True)
                if found.returncode == 0:
                    window = found.stdout.splitlines()[-1]
                if window and 'HOLD:' in log.read_text():
                    break
                time.sleep(.1)
            assert window and 'HOLD:' in log.read_text(), log.read_text()
            event('key', '--window', window, 'space')
            event('key', '--window', window, 'g')
            event('windowsize', window, width, height)
            time.sleep(.3)
            event('mousemove', '--window', window, calf_x, calf_y,
                  'click', '--repeat', 2, '--delay', 120, 1)
            time.sleep(.3)
            event('key', '--window', window, 'space')
            time.sleep(.5)
            event('windowfocus', window)
            # XTEST events update GLFW's actual modifier state, unlike synthetic
            # key events sent directly with --window.
            event('keydown', 'Control_L')
            time.sleep(.1)
            event('mousedown', button)
            for step in range(12):
                event('mousemove', '--window', window, calf_x+step*5, calf_y-step*8)
                time.sleep(.05)
            time.sleep(.2)
            event('mouseup', button)
            event('keyup', 'Control_L')
            time.sleep(.5)
            event('key', '--window', window, 'Escape')
            assert process.wait(timeout=8) == 0, log.read_text()
            with csv_path.open() as stream:
                rows = [row for row in csv.DictReader(stream) if row['controller'] == 'gravity']
            assert len(rows) > 50, log.read_text()
            joints = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint']
            before_drag = [row for row in rows if float(row['sim_time']) < float(rows[0]['sim_time'])+.3]
            baseline = max(abs(float(row['q_'+j])-float(rows[0]['q_'+j]))
                           for row in before_drag for j in joints)
            displacement = max(abs(float(row['q_'+j])-float(rows[0]['q_'+j]))
                               for row in rows for j in joints)
            assert baseline < .005, ('leg was moving before drag', baseline)
            assert displacement > .03, ('Ctrl+drag did not perturb the leg', button, displacement)
            assert 'task_controller exited code=0' in log.read_text(), log.read_text()
            print(f'PASS: Ctrl+button {button}, window {width}x{height}, joint displacement={displacement:.4f} rad')
        finally:
            event('mouseup', button)
            event('keyup', 'Control_L')
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
