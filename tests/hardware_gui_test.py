"""Default HW test opens native OpenGL; keyboard commands traverse the hardware Input path."""
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import tempfile
import time

from gui_test_utils import event, find_window

ROOT = Path(__file__).resolve().parents[1]
name = f'/qhrr-hardware-gui-{os.getpid()}'
endpoint = Path(f'/tmp/qhrr05-input-{os.getuid()}/{name[1:]}.sock')
state = {}
bridge = None

with tempfile.TemporaryDirectory(prefix='qhrr-hardware-gui-') as folder:
    log_path = Path(folder) / 'gui.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            [str(ROOT / 'scripts/run_hw_deploy_test.sh'), '--shm-name', name],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_for(predicate, description, timeout=25):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            assert process.poll() is None, log_path.read_text()
            value = predicate()
            if value:
                return value
            time.sleep(.025)
        raise AssertionError(description + '\n' + json.dumps(state) + '\n' + log_path.read_text())

    try:
        wait_for(endpoint.exists, 'Input did not start')
        window = wait_for(lambda: find_window('^MuJoCo simulate : qhrr-05'),
                          'default launcher did not open OpenGL')
        bridge = subprocess.Popen([str(ROOT / 'build/web_gui_bridge'), name,
            str(ROOT / 'config/control/control.yaml')], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, text=True)
        selector = selectors.DefaultSelector()
        selector.register(bridge.stdout, selectors.EVENT_READ)

        def observed(predicate):
            for key, _ in selector.select(.025):
                line = key.fileobj.readline()
                assert line, 'state bridge exited'
                state.update(json.loads(line))
            return predicate(state)

        wait_for(lambda: observed(lambda s: s.get('ready')), 'no control state')
        assert state['drive_state'] == 'DISARMED'
        assert not state['simulation_available'] and state['contact'] is None
        assert not Path(f'/tmp/qhrr05-simulation-{os.getuid()}/{name[1:]}.sock').exists()
        event('windowfocus', '--sync', window)
        event('key', 'z')
        wait_for(lambda: observed(lambda s: s['requested_mode'] == 6), 'Z did not reach Input')
        assert state['drive_state'] == 'DISARMED'
        assert not any(m['enabled'] for m in state['motors'])
        event('key', 'a')
        wait_for(lambda: observed(lambda s: s['drive_state'] == 'ACTIVE' and
                 s['zero_set']['travel_rad'] > .005), 'Arm/zero-set did not move the device')
        last_phase = None

        def calibrated(s):
            global last_phase
            phase = (s['zero_set']['axis'], s['zero_set']['phase'])
            if phase != last_phase:
                print('GUI zero-set:', phase, flush=True)
                last_phase = phase
            assert s['drive_state'] != 'FAULT', json.dumps(s)
            return s['zero_set']['phase'] == 'COMPLETE'

        wait_for(lambda: observed(calibrated), 'GUI zero-set did not complete', timeout=130)
        assert all(m['coordinate_valid'] for m in state['motors'])
        event('key', 'h')
        wait_for(lambda: observed(lambda s: s['mode'] == 0 and s['holding']),
                 'GUI homing did not hold')
        event('key', 'x')
        wait_for(lambda: observed(lambda s: s['drive_state'] == 'DISARMED' and
                 not any(m['enabled'] for m in s['motors'])), 'Release did not reach HAL')
        event('key', 'Escape')
        assert process.wait(timeout=12) == 0, log_path.read_text()
        assert not (Path('/dev/shm') / name[1:]).exists(), 'GUI exit left its hardware session'
        print('PASS: default OpenGL window, complete sequential zero-set, homing, Release through Input, clean GUI exit')
    finally:
        if bridge:
            bridge.terminate()
            bridge.wait(timeout=5)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=12)
