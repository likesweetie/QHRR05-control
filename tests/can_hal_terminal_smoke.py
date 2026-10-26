"""Verify the default daemon owns HAL, simulator, controller, and statistics window."""

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def wait_for(predicate, message, timeout=8):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        value = predicate()

        if value:
            return value

        time.sleep(0.05)

    raise AssertionError(message())


def find_window(title='QHRR05-CAN-HAL'):
    result = subprocess.run(
        ['xdotool', 'search', '--name', '^' + title + '$'], capture_output=True, text=True
    )
    return result.stdout.splitlines()[-1] if result.returncode == 0 else None


def find_daemon(manifest):
    for candidate in Path('/proc').iterdir():
        if not candidate.name.isdigit():
            continue

        try:
            args = (candidate / 'cmdline').read_bytes().split(b'\0')
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue

        if args and args[0].endswith(b'/daemon') and str(manifest).encode() in args:
            return int(candidate.name)

    return None


with tempfile.TemporaryDirectory(prefix='qhrr05-can-terminal-') as directory:
    directory = Path(directory)

    hal_source = ROOT / 'config/hal_config/can_hal.yaml'
    profile = yaml.safe_load(
        (hal_source.parent / yaml.safe_load(hal_source.read_text())['motor_config']).read_text()
    )
    (directory / 'motor profile.json').write_text(json.dumps(profile))

    for action in [
        'daemon-stop',
        'hal-stop',
        'window-close',
        'controller-window-close',
        'simulator-window-close',
        'terminal-failure',
        'hal-startup-failure',
        'without-can',
        'vcan-startup-failure',
    ]:
        name = f'/qhrr05-can-terminal-{os.getpid()}-{action}'
        sim_name = name
        shm = Path('/dev/shm') / name[1:]
        sim_shm = Path('/dev/shm') / sim_name[1:]
        config = directory / f'{action}-can.json'
        settings = yaml.safe_load((ROOT / 'config/hal_config/can_hal.yaml').read_text())
        settings['motor_config'] = 'motor profile.json'

        if action == 'hal-startup-failure':
            settings['interface'] = 'no_such_can'

        config.write_text(json.dumps(settings))

        simulation_config = directory / f'{action}-simulation.json'
        simulation = yaml.safe_load((ROOT / 'tests/fixtures/simulation_config.yaml').read_text())
        simulation['iq_sensor']['enabled'] = action == 'daemon-stop'
        simulation_config.write_text(json.dumps(simulation))

        manifest = directory / f'{action}-processes.json'
        processes = yaml.safe_load((ROOT / 'config/app_config/daemon_config.yaml').read_text())
        processes['shm_name'] = name
        processes['log_directory'] = str(directory / 'logs')
        control = str(ROOT / 'tests/fixtures/control_config.yaml')

        for spec in processes['processes'] + processes['debug_processes']:
            spec['exec_path'] = str(ROOT / 'build' / spec['exec_path'])
            spec['working_directory'] = str(ROOT)

            if spec['name'] == 'can_hal':
                spec['args'] = [str(config), '{shm_name}']

                if action == 'terminal-failure':
                    spec['terminal_exec'] = '/bin/false'

            elif spec['name'] == 'mujoco_simulate':
                args = spec['args']
                args[args.index('--control-config') + 1] = control

                if '--simulation-config' in args:
                    args[args.index('--simulation-config') + 1] = str(simulation_config)

                if '--hal-config' in args:
                    args[args.index('--hal-config') + 1] = str(config)
            elif spec['name'] == 'task_controller':
                spec['args'] = [control, '{shm_name}'] + (
                    ['--without-can'] if '--without-can' in spec['args'] else []
                )

        manifest.write_text(json.dumps(processes))
        log = directory / f'{action}.log'
        environment = os.environ.copy()
        environment['GDK_BACKEND'] = 'x11'
        environment.pop('WAYLAND_DISPLAY', None)

        if action in ['vcan-startup-failure', 'without-can']:
            fake_bin = directory / f'{action}-bin'
            fake_bin.mkdir()
            fake_ip = fake_bin / 'ip'
            fake_ip.write_text('#!/bin/sh\necho "test: vcan setup failed" >&2\nexit 41\n')
            fake_ip.chmod(0o755)
            environment['PATH'] = str(fake_bin) + os.pathsep + environment['PATH']

        with log.open('w') as stream:
            command = [
                str(ROOT / 'scripts/run_daemon.sh'),
                '--process-config',
                str(manifest),
            ]

            if action == 'without-can':
                command.append('--without-can')
                # Use this test's display; never reuse the host desktop's terminal server.
                command = ['dbus-run-session', '--', *command]
                environment.pop('GNOME_TERMINAL_SERVICE', None)
                environment.pop('GNOME_TERMINAL_SCREEN', None)
                runtime = directory / 'without-can-runtime'
                runtime.mkdir(mode=0o700)
                environment['XDG_RUNTIME_DIR'] = str(runtime)
                environment['NO_AT_BRIDGE'] = '1'
                environment['GIO_USE_VFS'] = 'local'
                environment['GTK_USE_PORTAL'] = '0'
            else:
                command.append('--isolated-vcan')

            process = subprocess.Popen(
                command,
                stdout=stream,
                stderr=subprocess.STDOUT,
                env=environment,
            )

        daemon_pid = None
        children = {}

        try:
            if action == 'vcan-startup-failure':
                assert process.wait(timeout=12) == 41, log.read_text()
                assert 'test: vcan setup failed' in log.read_text(), log.read_text()
                assert '[daemon]' not in log.read_text(), log.read_text()
                assert not shm.exists() and not sim_shm.exists(), log.read_text()
                print('PASS: vcan setup failure aborts without launching daemon or debug mode')
                continue

            if action in ['terminal-failure', 'hal-startup-failure']:
                assert process.wait(timeout=12) != 0, log.read_text()
                assert not shm.exists() and not sim_shm.exists(), log.read_text()

                if action == 'hal-startup-failure':
                    assert (
                        'CAN interface not found: no_such_can' in log.read_text()
                    ), log.read_text()

                print('PASS:', action, 'explicit error and resource cleanup')
                continue

            daemon_pid = wait_for(lambda: find_daemon(manifest), log.read_text, 20)

            def started():
                children.update(
                    (name, int(pid))
                    for name, pid in re.findall(
                        r'\[daemon\] (\S+) started pid=(\d+)', log.read_text()
                    )
                )
                assert process.poll() is None, log.read_text()

                if action == 'without-can':
                    return len(children) == 4 and sim_shm.exists()

                return len(children) == 6 and shm.exists() and sim_shm.exists()

            wait_for(started, log.read_text)
            assert (
                len(list(Path('/dev/shm').glob(name[1:] + '*'))) == 1
            ), 'Multiple SHM objects created'

            for child in children.values():
                status = Path(f'/proc/{child}/status').read_text()
                assert re.search(rf'^PPid:\s+{daemon_pid}$', status, re.MULTILINE), status

            same_network = os.readlink(f'/proc/{daemon_pid}/ns/net') == os.readlink(
                '/proc/self/ns/net'
            )

            for title in ['QHRR05-mujoco_simulate', 'QHRR05-task_controller']:
                wait_for(lambda: find_window(title), log.read_text)

            if action == 'without-can':
                assert same_network, 'Debug execution created an unnecessary CAN namespace'
                assert not find_window(), log.read_text()
                assert 'DEBUG --without-can' in log.read_text(), log.read_text()
                os.kill(daemon_pid, signal.SIGTERM)
                assert process.wait(timeout=8) == 0, log.read_text()
                assert not sim_shm.exists(), log.read_text()
                print(
                    'PASS: explicit --without-can, two application terminals, no HAL/namespace, clean shutdown'
                )
                continue

            if action == 'daemon-stop':
                runs = sorted((directory / 'logs').glob('run-*'))
                sim_log = runs[-1] / 'mujoco_simulate.log'
                wait_for(
                    lambda: '[simulation] Iq sensor ENABLED' in sim_log.read_text(),
                    lambda: log.read_text()
                    + '\nRELAY LOGS:\n'
                    + '\n'.join(
                        str(path) + '\n' + path.read_text() for path in runs[-1].glob('*.log')
                    ),
                )

            assert not same_network, 'Explicit --isolated-vcan did not isolate its vcan bus'
            assert 'vcan0:' in Path(f'/proc/{daemon_pid}/net/dev').read_text()

            window = wait_for(find_window, log.read_text)
            subprocess.run(['xdotool', 'windowsize', window, '1600', '600'], check=True)
            time.sleep(0.5)

            if action == 'daemon-stop':
                from PIL import ImageGrab

                subprocess.run(['xdotool', 'windowraise', window], check=True)
                time.sleep(4.3)
                ImageGrab.grab().save('/tmp/qhrr05-can-terminal.png')
                os.kill(daemon_pid, signal.SIGTERM)
            elif action == 'hal-stop':
                os.kill(children['can_hal'], signal.SIGTERM)
            elif action == 'controller-window-close':
                subprocess.run(
                    ['xdotool', 'windowclose', find_window('QHRR05-task_controller')], check=True
                )
            elif action == 'simulator-window-close':
                subprocess.run(
                    ['xdotool', 'windowclose', find_window('QHRR05-mujoco_simulate')], check=True
                )
            else:
                subprocess.run(['xdotool', 'windowclose', window], check=True)

            code = process.wait(timeout=8)

            if 'window-close' not in action:
                assert code == 0, log.read_text()

            if action == 'daemon-stop':
                assert '[simulate] stopped at t=' in sim_log.read_text(), sim_log.read_text()
                assert 'STATISTICS' not in sim_log.read_text()

            assert not shm.exists() and not sim_shm.exists(), log.read_text()
            assert all(
                not Path(f'/proc/{pid}').exists() for pid in children.values()
            ), log.read_text()
            wait_for(
                lambda: not any(
                    find_window(title)
                    for title in [
                        'QHRR05-CAN-HAL',
                        'QHRR05-mujoco_simulate',
                        'QHRR05-task_controller',
                    ]
                ),
                lambda: 'Application statistics window remains open',
            )
            print('PASS:', action, 'one daemon, six direct children, SHM and window cleanup')
        finally:
            if process.poll() is None:
                if daemon_pid:
                    os.kill(daemon_pid, signal.SIGTERM)

                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
