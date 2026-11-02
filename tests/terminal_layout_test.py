"""Run in an isolated X11 display: tile owned windows, including the actual daemon caller."""

import json
import ctypes
from contextlib import contextmanager
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import tempfile
import time

from gui_test_utils import event, geometry, wait_for, windows as visible_windows

binary = Path(sys.argv[1]).resolve()
terminal = shutil.which('gnome-terminal.real') or shutil.which('gnome-terminal')
assert terminal, 'GNOME Terminal is required for the X11 layout test'


def window_for_role(role):
    for window in visible_windows():
        value = subprocess.run(
            ['xprop', '-id', window, 'WM_WINDOW_ROLE'], capture_output=True, text=True
        ).stdout
        if f'"{role}"' in value:
            return window
    return None


def daemon_pid(manifest):
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            command = (path / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if command and command[0] == str(binary).encode() and str(manifest).encode() in command:
            return int(path.name)
    return None


def overlaps(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return ax < bx + bw and bx < ax + aw and ay < by + bh and by < ay + ah


@contextmanager
def persistent_display():
    # Keep Xvfb from resetting root properties when a short-lived xprop client exits.
    x11 = ctypes.CDLL('libX11.so.6')
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x11.XOpenDisplay(None)
    assert display, 'Test display is unavailable'
    try:
        yield
    finally:
        x11.XCloseDisplay(display)


with persistent_display(), tempfile.TemporaryDirectory(prefix='qhrr05-window-layout-') as directory:
    directory = Path(directory)
    env = os.environ.copy()
    env.pop('GNOME_TERMINAL_SERVICE', None)
    env.pop('GNOME_TERMINAL_SCREEN', None)
    env.pop('WAYLAND_DISPLAY', None)
    env['GDK_BACKEND'] = 'x11'
    # Emulate panels without running a window manager in the Xvfb test display.
    subprocess.run(
        [
            'xprop',
            '-root',
            '-f',
            '_NET_WORKAREA',
            '32c',
            '-set',
            '_NET_WORKAREA',
            '48, 28, 2464, 1380',
        ],
        check=True,
    )
    subprocess.run(
        [
            'xprop',
            '-root',
            '-f',
            '_NET_CURRENT_DESKTOP',
            '32c',
            '-set',
            '_NET_CURRENT_DESKTOP',
            '0',
        ],
        check=True,
    )

    for caller in (False, True):
        name = f'/qhrr05-layout-{os.getpid()}-{caller}'
        manifest = directory / f'{caller}.json'
        settings = {
            'runtime_mode': 'simulation', 'shm_name': name,
            'terminal_exec': 'gnome-terminal',
            'log_directory': str(directory / 'logs'),
            'terminal_layout': {
                'monitor': 'auto',
                'columns': 2,
                'gap_px': 12,
                'margin_px': 12,
                'zoom': 0.85,
                'include_daemon': True,
            },
            'monitoring': {'display_interval_sec': 2, 'window_sec': 10, 'sample_hz': 100},
            'processes': [
                {'name': name, 'exec_path': '/bin/sleep', 'args': ['60']}
                for name in ('alpha', 'beta', 'gamma')
            ],
        }
        manifest.write_text(json.dumps(settings))
        log_path = directory / f'{caller}.log'
        foreign_role = f'unrelated-{os.getpid()}-{caller}'
        with log_path.open('w') as log:
            foreign = subprocess.Popen(
                [
                    terminal,
                    '--wait',
                    '--window',
                    '--role=' + foreign_role,
                    '--title=QHRR05-alpha',
                    '--geometry=80x24',
                    '--',
                    '/bin/sleep',
                    '60',
                ],
                stdout=log,
                stderr=log,
                env=env,
            )
        foreign_window = wait_for(lambda: window_for_role(foreign_role), log_path.read_text)
        foreign_geometry = geometry(foreign_window)
        command = [str(binary), '--config', str(manifest)]
        restored = directory / 'caller-restored'
        if caller:
            wrapper = (
                'import os,subprocess,sys,time; from pathlib import Path; '
                'os.write(1,bytes([27])+b"]0;QHRR05-RESTORE-CHECK"+bytes([7])+b"SHELL BEFORE"+bytes([10])); '
                'p=subprocess.Popen([sys.argv[1],"--config",sys.argv[2]]); '
                'code=p.wait(); os.write(1,b"SHELL AFTER\\n"); '
                'Path(sys.argv[3]).write_text(str(code)); time.sleep(30)'
            )
            command = [
                terminal,
                '--wait',
                '--window',
                '--role=layout-launcher',
                '--geometry=80x24',
                '--',
                sys.executable,
                '-c',
                wrapper,
                str(binary),
                str(manifest),
                str(restored),
            ]
        with log_path.open('a') as log:
            process = subprocess.Popen(command, stdout=log, stderr=log, env=env)
        pid = None
        try:
            pid = wait_for(lambda: daemon_pid(manifest), log_path.read_text)
            roles = [f'qhrr05-{pid}-{name}' for name in ('alpha', 'beta', 'gamma')]
            if caller:
                roles.insert(0, 'layout-launcher')

            def arranged():
                if process.poll() is not None:
                    raise AssertionError(log_path.read_text())
                windows = [window_for_role(role) for role in roles]
                if not all(windows):
                    return None
                rectangles = [geometry(window) for window in windows]
                # Monitor/panel intersection plus configured margins; all tiles must be large.
                for x, y, width, height in rectangles:
                    if (
                        x < 60
                        or y < 40
                        or x + width > 2500
                        or y + height > 1396
                        or width < 600
                        or height < 500
                    ):
                        return None
                if any(
                    overlaps(a, b) for i, a in enumerate(rectangles) for b in rectangles[i + 1 :]
                ):
                    return None
                return windows, rectangles

            windows, rectangles = wait_for(
                arranged, lambda: log_path.read_text() + '\nroles=' + repr(roles)
            )
            assert (
                geometry(foreign_window) == foreign_geometry
            ), 'An unrelated window with the same title was moved'
            if caller:
                title = subprocess.check_output(
                    ['xprop', '-id', windows[0], '_NET_WM_NAME'], text=True
                )
                assert f'QHRR05-DAEMON [{pid}]' in title, title
            time.sleep(0.2)
            os.kill(pid, signal.SIGINT if caller else signal.SIGTERM)
            if caller:
                wait_for(restored.exists, log_path.read_text)
                assert restored.read_text() == '0', restored.read_text()
                # Process exit precedes GNOME consuming the final PTY title sequence.
                def restored_title():
                    return subprocess.check_output(
                        ['xprop', '-id', windows[0], '_NET_WM_NAME'], text=True
                    )

                wait_for(lambda: 'QHRR05-RESTORE-CHECK' in restored_title(), restored_title)
                subprocess.run(['xdotool', 'windowclose', windows[0]], check=True)
                process.wait(timeout=8)
                print(
                    'PASS: Ctrl+C restores actual GNOME title/session restoration; screen control sequence checked by terminal_statistics'
                )
            else:
                assert process.wait(timeout=8) == 0, log_path.read_text()
            assert not (Path('/dev/shm') / name[1:]).exists()
            print(
                'PASS:',
                'four windows including daemon caller' if caller else 'three child terminals',
                rectangles,
            )
        finally:
            if pid and Path(f'/proc/{pid}').exists():
                os.kill(pid, signal.SIGTERM)
            if process.poll() is None:
                launcher = window_for_role('layout-launcher') if caller else None
                if launcher:
                    subprocess.run(['xdotool', 'windowclose', launcher], check=True)
                process.wait(timeout=8)
            subprocess.run(['xdotool', 'windowclose', foreign_window], check=True)
            foreign.wait(timeout=5)

    for key, value, message in [
        ('monitor', 'missing-monitor', 'monitor is unavailable'),
        ('columns', 0, 'invalid terminal_layout'),
    ]:
        settings['terminal_layout'][key] = value
        manifest.write_text(json.dumps(settings))
        result = subprocess.run(
            [str(binary), '--config', str(manifest)],
            capture_output=True,
            text=True,
            env=env,
            timeout=8,
        )
        assert result.returncode == 1 and message in result.stderr, result.stdout + result.stderr
        assert not (Path('/dev/shm') / name[1:]).exists()
        settings['terminal_layout'][key] = 'auto' if key == 'monitor' else 2
        print('PASS: explicit layout error for', key)

    subprocess.run(
        ['xprop', '-root', '-f', '_NET_WORKAREA', '32c', '-set', '_NET_WORKAREA', '0, 0, 800, 600'],
        check=True,
    )
    manifest.write_text(json.dumps(settings))
    result = subprocess.run(
        [str(binary), '--config', str(manifest)], capture_output=True, text=True, env=env, timeout=8
    )
    assert result.returncode == 1 and 'too small for readable tiles' in result.stderr, (
        result.stdout + result.stderr
    )
    print('PASS: undersized work area fails explicitly without overlap or fallback')

    # Exercise the Wayland policy using isolated X11 terminal servers. No compositor
    # or global desktop settings are changed, and the child environment stays native.
    subprocess.run(
        ['xprop', '-root', '-f', '_NET_WORKAREA', '32c', '-set', '_NET_WORKAREA',
         '48, 28, 2464, 1380'], check=True,
    )
    for explicit in (False, True):
        settings['shm_name'] = f'/qhrr05-wayland-windows-{os.getpid()}-{explicit}'
        settings['processes'] = [
            {'name': name, 'exec_path': '/bin/sh',
             'args': ['-c', f'printf "SEPARATE-{name} backend=%s\\n" "$GDK_BACKEND"; exec /bin/sleep 60']}
            for name in ('alpha', 'beta', 'gamma')
        ]
        manifest.write_text(json.dumps(settings))
        log_path = directory / f'wayland-{explicit}.log'
        command = [str(binary), '--config', str(manifest)]
        if explicit:
            command.append('--terminals')
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=log,
                                       env={**env, 'XDG_SESSION_TYPE': 'wayland',
                                            'GDK_BACKEND': 'wayland',
                                            'GNOME_TERMINAL_SERVICE': ':999.12345'})
        try:
            roles = [f'qhrr05-{process.pid}-{name}' for name in ('alpha', 'beta', 'gamma')]

            def tiled_windows():
                assert process.poll() is None, log_path.read_text()
                found = [window_for_role(role) for role in roles]
                if not all(found):
                    return None
                rectangles = [geometry(window) for window in found]
                if any(x < 60 or y < 40 or x + w > 2500 or y + h > 1396
                       or w < 600 or h < 500 for x, y, w, h in rectangles):
                    return None
                if any(overlaps(a, b) for i, a in enumerate(rectangles)
                       for b in rectangles[i + 1:]):
                    return None
                return found

            windows = wait_for(tiled_windows, log_path.read_text)
            assert len(set(windows)) == 3, windows
            output = log_path.read_text()
            assert 'arranging separate process terminals through XWayland' in output, output
            assert '--no-terminals: child output' not in output, output
            server_pids = []
            for window in windows:
                info = subprocess.check_output(['xprop', '-id', window, '_NET_WM_PID'], text=True)
                server_pid = re.search(r'=\s*(\d+)', info).group(1)
                server_pids.append(server_pid)
                server_env = (Path('/proc') / server_pid / 'environ').read_bytes().split(b'\0')
                assert b'GDK_BACKEND=x11' in server_env, server_env
                assert (b'DBUS_SESSION_BUS_ADDRESS=' + env['DBUS_SESSION_BUS_ADDRESS'].encode()) not in server_env
            assert len(set(server_pids)) == 3, 'Terminal servers were not isolated'
            for name in ('alpha', 'beta', 'gamma'):
                marker = f'SEPARATE-{name} backend=wayland'

                def persisted():
                    return any(marker in path.read_text()
                               for path in (directory / 'logs').glob(f'run-*-{process.pid}/{name}.log'))

                wait_for(persisted, log_path.read_text)
                assert marker not in log_path.read_text(), 'Child output was merged into daemon output'
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=8) == 0, log_path.read_text()
            assert not (Path('/dev/shm') / settings['shm_name'][1:]).exists()
            wait_for(lambda: not any(window_for_role(role) for role in roles), log_path.read_text)
            print('PASS: Wayland policy tiles three isolated X11 windows; application environment and logs retained',
                  '(explicit --terminals)' if explicit else '(default)')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=8)
