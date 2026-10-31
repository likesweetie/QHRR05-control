"""Black-box hardware pipeline acceptance. Device traffic alone is virtualized."""
import json
import os
from pathlib import Path
import selectors
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
manifest, session = sys.argv[1:3]
endpoint = f'/tmp/qhrr05-input-{os.getuid()}/{session[1:]}.sock'
logs = Path(manifest).parent/'processes.log'
state = {}
bridge = None
controller = None
request_id = 0

def supervise(command, accepted=True):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(30)
        client.connect(f'/tmp/qhrr05-daemon-{os.getuid()}/{session[1:]}.sock')
        client.sendall(f'1 {command}\n'.encode())
        data = b''
        while b'\n' not in data:
            chunk = client.recv(65536)
            assert chunk, 'supervisor closed without a response'
            data += chunk
        reply = json.loads(data)
        assert (reply['type'] == 'ack') == accepted, reply
        return reply['runtime']

def request(command, accepted=True):
    global request_id
    request_id += 1
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(3)
        client.connect(endpoint)
        client.sendall(f'{request_id} {command}\n'.encode())
        data = b''
        while b'\n' not in data:
            chunk = client.recv(4096)
            if not chunk: raise AssertionError('Input closed without a decision')
            data += chunk
        reply = json.loads(data)
    assert (reply['type'] == 'ack') == accepted, (command, reply)
    return reply

def wait_for(predicate, message, timeout=10):
    global state
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if daemon.poll() is not None:
            raise AssertionError(f'daemon exited: {logs.read_text()[-6000:]}')
        ready = selector.select(.2)
        for key, _ in ready:
            line = key.fileobj.readline()
            if not line: raise AssertionError('state bridge exited')
            state = json.loads(line)
        if predicate(state): return state
    raise AssertionError(f'{message}: {json.dumps(state)}\n{logs.read_text()[-6000:]}')

with logs.open('w') as output:
    daemon = subprocess.Popen([str(ROOT/'scripts/run_daemon.sh'), '--config', manifest,
        '--no-terminals'], stdout=output, stderr=subprocess.STDOUT)
    try:
        until = time.monotonic() + 30
        while not Path(endpoint).exists():
            if daemon.poll() is not None or time.monotonic() > until:
                raise AssertionError(logs.read_text())
            time.sleep(.05)
        bridge = subprocess.Popen([str(ROOT/'build/web_gui_bridge'), session,
            str(ROOT/'config/control/control.yaml')], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        selector = selectors.DefaultSelector()
        selector.register(bridge.stdout, selectors.EVENT_READ)
        wait_for(lambda s: s.get('ready'), 'control not ready')
        deadline = time.monotonic() + 15
        while True:
            runtime = supervise('status')
            web = next(p for p in runtime['processes'] if p['name'] == 'web_gui')
            try:
                assert web['running']
                assert urlopen('http://127.0.0.1:8080/processes', timeout=1).status == 200
                break
            except Exception:
                assert time.monotonic() < deadline, 'automatic web GUI did not start'
                time.sleep(.1)
        web_pid = web['pid']
        assert not state['simulation_available'] and not state['simulation_connected']
        assert state['observation_origin'] == 'device'
        assert state['contact'] is None and state['drive_state'] == 'DISARMED'
        assert not Path(f'/tmp/qhrr05-simulation-{os.getuid()}/{session[1:]}.sock').exists()
        request('pause 2', accepted=False)
        request('mode 0 0', accepted=False)  # No assumed joint zero on physical startup.
        request('motor-enable 3', accepted=False)
        request('motor-enable 0')
        wait_for(lambda s: s['drive_state'] == 'ARMED' and s['motors'][0]['enabled'], 'individual motor did not enable')
        assert not any(m['enabled'] for m in state['motors'][1:])
        assert all(m['command']['kp'] == 0 and m['command']['kd'] == 0 for m in state['motors'])
        request('motor-enable 1')
        wait_for(lambda s: s['motors'][1]['enabled'], 'second motor did not enable')
        request('motor-disable 0')
        wait_for(lambda s: not s['motors'][0]['enabled'] and s['motors'][1]['enabled'], 'individual disable affected wrong axes')
        request('motor-enable -1')
        wait_for(lambda s: all(m['enabled'] for m in s['motors']) and s['drive_state'] == 'ARMED', 'neutral enable all failed')
        request('release')
        wait_for(lambda s: not any(m['enabled'] for m in s['motors']), 'release after per-axis enable failed')
        print('[hardware-test] DISARMED startup and unavailable simulation commands verified; starting zero set', flush=True)
        request('mode 6 0')
        request('arm')
        wait_for(lambda s: s['zero_set']['complete'], 'zero set failed', timeout=120)
        print('[hardware-test] all joint coordinates calibrated; checking Homing/Release', flush=True)
        assert all(m['coordinate_valid'] for m in state['motors'])
        assert all(m['current'] is not None for m in state['motors'])
        assert all(m['position'] is not None for m in state['motors'])
        request('release')
        wait_for(lambda s: s['drive_state'] == 'DISARMED' and
                 not any(m['enabled'] for m in s['motors']), 'release failed')
        request('mode 0 0')
        request('arm')
        wait_for(lambda s: s['mode'] == 0 and s['holding'], 'homing did not hold', 15)
        assert all(value is not None for value in state['toe_position'])
        request('release')
        wait_for(lambda s: s['drive_state'] == 'DISARMED', 'second release failed')
        request('mode 5 0')
        request('arm')
        wait_for(lambda s: s['drive_state'] == 'ACTIVE' and
                 all(m['enabled'] for m in s['motors']), 'damping did not arm')
        children = Path(f'/proc/{daemon.pid}/task/{daemon.pid}/children').read_text().split()
        controller = next(int(pid) for pid in children
            if Path(f'/proc/{pid}/comm').read_text().strip() == 'control')
        print('[hardware-test] injecting controller stall; expecting HAL command expiry', flush=True)
        os.kill(controller, signal.SIGSTOP)
        wait_for(lambda s: s['transport_fault'] == 'controller command expired' and
                 not any(m['enabled'] for m in s['motors']), 'HAL did not expire stale command')
        os.kill(controller, signal.SIGCONT)
        controller = None
        wait_for(lambda s: s['drive_state'] == 'FAULT', 'controller did not observe transport fault')
        request('arm', accepted=False)
        request('clear-fault')
        wait_for(lambda s: s['drive_state'] == 'DISARMED' and s['transport_fault'] == 'none',
                 'fault clear failed')
        assert not any(m['enabled'] for m in state['motors'])
        supervise('process control stop', accepted=False)
        runtime = supervise('process shm_logger stop')
        until = time.monotonic() + 5
        while next(p for p in runtime['processes'] if p['name'] == 'shm_logger')['running']:
            assert time.monotonic() < until, runtime
            time.sleep(.05)
            runtime = supervise('status')
        time.sleep(1.2)
        assert not next(p for p in supervise('status')['processes'] if p['name'] == 'shm_logger')['running']
        supervise('process shm_logger start')
        runtime = supervise('stack stop')
        assert runtime['state'] == 'STOPPED' and not any(p['running'] for p in runtime['processes'] if not p['keep_on_stack_stop'])
        assert next(p for p in runtime['processes'] if p['name'] == 'web_gui')['pid'] == web_pid
        assert urlopen('http://127.0.0.1:8080/shm', timeout=1).status == 200
        runtime = supervise('stack start')
        assert runtime['state'] == 'RUNNING'
        assert next(p for p in runtime['processes'] if p['name'] == 'web_gui')['pid'] == web_pid
        wait_for(lambda s: s['ready'] and s['drive_state'] == 'DISARMED' and
                 not any(m['enabled'] for m in s['motors']), 'stack restart did not remain disarmed')
        print('PASS: hardware-only pipeline over vcan0; no simulation IPC/GT; explicit arm, zero set, homing, release, command expiry and fault clear', flush=True)
    except Exception:
        print(f'Hardware test diagnostics: {logs.read_text()[-12000:]}', file=sys.stderr)
        raise
    finally:
        if controller: os.kill(controller, signal.SIGCONT)
        if bridge:
            bridge.terminate()
            bridge.wait(timeout=5)
        daemon.terminate()
        try: daemon.wait(timeout=10)
        except subprocess.TimeoutExpired:
            daemon.kill()
            daemon.wait()
