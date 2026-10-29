"""Native controls and Ctrl-drag, without image matching or tracked screenshot output."""

from contextlib import contextmanager
import csv
import io
from itertools import groupby
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from daemon_test_config import simulation_command
from gui_test_utils import event, find_window, wait_for


@contextmanager
def simulator(directory, name):
    folder = directory / name
    folder.mkdir()
    log, telemetry = folder / 'daemon.log', folder / 'telemetry.csv'
    with log.open('w') as stream:
        process = subprocess.Popen(
            simulation_command(folder, f'/qhrr05-gui-{os.getpid()}-{name}', telemetry),
            stdout=stream,
            stderr=subprocess.STDOUT,
        )
    try:

        def ready():
            assert process.poll() is None, log.read_text()
            return find_window('MuJoCo simulate.*qhrr-05') if '[control] DAMPING target captured:' in log.read_text() else None

        window = wait_for(ready, log.read_text, timeout=25)
        event('windowsize', window, 1600, 900)
        assert 'HOLD:' not in log.read_text(), 'Startup unexpectedly ran homing'
        event('key', '--window', window, 'h')
        wait_for(lambda: 'HOLD:' in log.read_text(), log.read_text)
        yield window, process, log, telemetry
    finally:
        event('mouseup', 1, 'mouseup', 3, 'keyup', 'Control_L', 'Shift_L')
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8)


def rows(path):
    if not path.exists():
        return []
    text = path.read_text()
    return list(csv.DictReader(io.StringIO(text[: text.rfind('\n') + 1])))


with tempfile.TemporaryDirectory(prefix='qhrr05-simulator-gui-') as tmp:
    directory = Path(tmp)
    with simulator(directory, 'controls') as (window, process, log, telemetry):
        key = lambda value: event('key', '--window', window, value)
        click = lambda x, y: event('mousemove', '--window', window, x, y, 'click', 1)
        key('w')
        wait_for(lambda: '[workspace] ready:' in log.read_text(), log.read_text)
        assert '[workspace] ERROR:' not in log.read_text(), log.read_text()
        key('w')
        wait_for(lambda: '[workspace] OFF' in log.read_text(), log.read_text)
        key('w')  # Cached geometry can be shown again without rebuilding it.
        wait_for(lambda: log.read_text().count('[workspace] ON') == 2, log.read_text)
        click(185, 735)  # Native Gravity button.
        wait_for(lambda: '[control] GRAVITY' in log.read_text(), log.read_text)
        key('space')
        time.sleep(0.3)
        key('space')
        click(65, 765)  # Native Magnetic button.
        magnetic = lambda: [r for r in rows(telemetry) if r['controller'] == 'magnetic']
        wait_for(lambda: len(magnetic()) > 20, log.read_text)
        # Hold the physical contact state while testing only the slider. Natural
        # contact/release transitions in a moving model also increment event_seq.
        key('space')
        wait_for(lambda: magnetic()[-1]['sim_paused'] == '1', log.read_text)
        time.sleep(.3)
        before = magnetic()[-1]
        click(225, 795)  # Live force slider must not restart the FSM.
        wait_for(lambda: float(magnetic()[-1]['force_target_n']) > 3.5, log.read_text)
        assert magnetic()[-1]['event_seq'] == before['event_seq']
        key('space')
        holds = log.read_text().count('HOLD:')
        click(65, 735)  # Native Homing button.
        wait_for(lambda: log.read_text().count('HOLD:') > holds, log.read_text)
        key('g')
        wait_for(lambda: log.read_text().count('[control] GRAVITY') >= 2, log.read_text)
        key('m')
        wait_for(lambda: log.read_text().count('[control] MAGNETIC') >= 2, log.read_text)
        holds = log.read_text().count('HOLD:')
        key('h')
        wait_for(lambda: log.read_text().count('HOLD:') > holds, log.read_text)
        click(65, 825)  # Native Tracking button, below the force slider.
        wait_for(lambda: '[control] TRACKING' in log.read_text(), log.read_text)
        wait_for(
            lambda: sum(r['controller'] == 'tracking' for r in rows(telemetry)) > 120,
            log.read_text,
        )
        key('g')
        wait_for(lambda: log.read_text().count('[control] GRAVITY') >= 3, log.read_text)
        key('t')
        wait_for(lambda: log.read_text().count('[control] TRACKING') >= 2, log.read_text)
        for shortcut, plane in [('2', 'XZ'), ('3', 'YZ'), ('1', 'XY')]:
            key(shortcut)
            wait_for(lambda: f'[control] TRACKING {plane}' in log.read_text(), log.read_text)
            time.sleep(0.4)
        holds = log.read_text().count('HOLD:')
        key('h')
        wait_for(lambda: log.read_text().count('HOLD:') > holds, log.read_text)
        click(185, 825)  # Native Ball follow button shares Tracking's row.
        wait_for(lambda: '[control] BALL FOLLOWING' in log.read_text(), log.read_text)
        wait_for(lambda: sum(r['controller'] == 'ball' for r in rows(telemetry)) > 40,
                 log.read_text)
        key('g')
        key('b')
        wait_for(lambda: log.read_text().count('[control] BALL FOLLOWING') >= 2, log.read_text)
        captured = log.read_text().count('[control] DAMPING target captured:')
        key('d')
        wait_for(lambda: log.read_text().count('[control] DAMPING target captured:') > captured,
                 log.read_text)
        wait_for(lambda: rows(telemetry)[-1]['controller'] == 'damping', log.read_text)
        key('r')
        wait_for(lambda: any(r['run_id'] == '1' and r['controller'] == 'damping'
                            for r in rows(telemetry)), log.read_text)
        holds = log.read_text().count('HOLD:')
        time.sleep(.2)
        assert log.read_text().count('HOLD:') == holds, 'Reset automatically ran homing'
        key('Escape')
        assert process.wait(timeout=8) == 0, log.read_text()
        data = rows(telemetry)
        assert {'damping', 'homing', 'gravity', 'magnetic', 'tracking', 'ball'} <= {r['controller'] for r in data}
        first = [r for r in data if r['run_id'] == '0']
        # Paused physics now publishes IPC heartbeats. Accumulate the control-clock
        # span across each stationary simulation-time plateau instead of assuming
        # that a pause produces a gap between two adjacent CSV rows.
        pauses = [list(group) for _, group in groupby(first, lambda row: row['sim_time'])]
        assert any(
            float(group[-1]['control_time']) - float(group[0]['control_time']) > .1
            for group in pauses
        ), 'Controller stopped with paused physics'
        print('PASS: native H/G/M/T controls, slider without restart, pause independence, reset/exit')

    # Fresh sessions keep the default camera deterministic; the second case covers resize.
    for button, width, height, x, y in [(3, 1600, 900, 750, 565), (1, 1400, 800, 672, 502)]:
        with simulator(directory, f'drag-{button}') as (window, process, log, telemetry):
            event('key', '--window', window, 'space', 'g')
            event('windowsize', window, width, height)
            time.sleep(0.2)
            event('mousemove', '--window', window, x, y, 'click', '--repeat', 2, '--delay', 120, 1)
            event('key', '--window', window, 'space')
            wait_for(
                lambda: len([r for r in rows(telemetry)
                             if r['controller'] == 'gravity' and r['sim_paused'] == '0']) > 20,
                log.read_text,
            )
            baseline = rows(telemetry)[-1]
            first_sequence = int(baseline['sample_seq'])
            event('windowfocus', window)
            event('keydown', 'Control_L', 'mousedown', button)
            for step in range(12):
                event('mousemove', '--window', window, x + step * 5, y - step * 8)
                time.sleep(0.05)
            event('mouseup', button, 'keyup', 'Control_L')
            time.sleep(0.2)
            if button == 1:
                event('key', '--window', window, 'w')  # Closing during generation must cancel cleanly.
            event('key', '--window', window, 'Escape')
            assert process.wait(timeout=8) == 0, log.read_text()
            data = [r for r in rows(telemetry) if int(r['sample_seq']) >= first_sequence]
            displacement = max(
                abs(float(r['q_' + joint]) - float(baseline['q_' + joint]))
                for r in data
                for joint in ('FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint')
            )
            assert displacement > 0.03, ('Ctrl-drag did not perturb the leg', button, displacement)
            print(
                'PASS: Ctrl-drag', button, (width, height), 'displacement', round(displacement, 4)
            )
