"""Terminal discovery and session selection, without CAN or a graphical display."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

binary = Path(sys.argv[1]).resolve()

with tempfile.TemporaryDirectory(prefix='qhrr05-terminal-config-') as temporary:
    directory = Path(temporary)
    commands = directory / 'bin'
    commands.mkdir()
    settings = {
        'runtime_mode': 'simulation',
        'shm_name': f'/qhrr05-terminal-config-{os.getpid()}',
        'terminal_exec': 'gnome-terminal',
        'log_directory': str(directory / 'logs'),
        'terminal_layout': {
            'monitor': 'auto', 'columns': 2, 'gap_px': 12, 'margin_px': 12,
            'zoom': 0.85, 'include_daemon': False,
        },
        'monitoring': {'display_interval_sec': 2.0, 'window_sec': 10.0, 'sample_hz': 100.0},
        'processes': [
            {'name': name, 'exec_path': '/bin/sh',
             'args': ['-c', f'printf "{name} started\\n"; /bin/sleep 0.1']}
            for name in ('worker', 'web_gui')
        ],
    }
    manifest = directory / 'session.json'
    environment = {key: value for key, value in os.environ.items()
                   if key not in ('DISPLAY', 'WAYLAND_DISPLAY', 'XDG_SESSION_TYPE')}
    environment.update(PATH=str(commands), DISPLAY='', XDG_SESSION_TYPE='x11')

    def run(*arguments, session='x11', wayland=None):
        manifest.write_text(json.dumps(settings))
        env = environment.copy()
        if session is None:
            env.pop('XDG_SESSION_TYPE')
        else:
            env['XDG_SESSION_TYPE'] = session
        if wayland is not None:
            env['WAYLAND_DISPLAY'] = wayland
        result = subprocess.run([str(binary), '--config', str(manifest), *arguments],
                                env=env, capture_output=True, text=True, timeout=5)
        assert not (Path('/dev/shm') / settings['shm_name'][1:]).exists(), result.stdout
        return result, result.stdout + result.stderr

    def executable(name):
        path = commands / name
        path.write_text('#!/bin/sh\nexit 99\n')
        path.chmod(0o755)
        return path

    # Validation fails before X11 setup; its diagnostic exposes the selected path.
    settings['processes'][0]['terminal_title'] = ''
    terminal = executable('gnome-terminal')
    result, output = run('--terminals')
    assert result.returncode == 1 and f'resolved={terminal};' in output, output
    print('PASS: discover gnome-terminal when no .real executable exists')

    real = executable('gnome-terminal.real')
    result, output = run('--terminals')
    assert result.returncode == 1 and f'resolved={real};' in output, output
    terminal.unlink()
    result, output = run('--terminals')
    assert f'resolved={real};' in output, output
    print('PASS: prefer .real and support installations providing only .real')

    settings['terminal_exec'] = str(real)
    result, output = run('--terminals')
    assert f'resolved={real};' in output, output
    settings['terminal_exec'] = os.path.relpath(real, binary.parent)
    result, output = run('--terminals')
    assert f'resolved={binary.parent / settings["terminal_exec"]};' in output, output
    print('PASS: preserve explicit absolute and binary-relative terminal paths')

    settings['terminal_exec'] = 'gnome-terminal'
    settings['processes'][0].pop('terminal_title')
    result, output = run('--terminals')
    assert result.returncode == 1 and 'accessible X11 display' in output, output
    assert 'invalid terminal configuration' not in output, output
    print('PASS: valid executable proceeds to X11 validation')

    real.chmod(0o644)
    result, output = run('--terminals')
    assert result.returncode == 1 and 'invalid terminal configuration: worker' in output, output
    assert 'install gnome-terminal or use --no-terminals' in output, output
    print('PASS: reject missing/non-executable terminal with actionable diagnostic')

    for session, wayland in [('wayland', None), (None, 'wayland-test')]:
        result, output = run(session=session, wayland=wayland)
        assert result.returncode == 1 and 'invalid terminal configuration: worker' in output, output
        assert 'arranging separate process terminals through XWayland' in output, output
        assert '--no-terminals: child output' not in output, output
        assert not (directory / 'logs').exists(), output
    print('PASS: Wayland requires a terminal and never silently combines child output')

    result, output = run(session='x11', wayland='stale-wayland-display')
    assert result.returncode == 1 and 'invalid terminal configuration' in output, output
    assert 'Wayland session:' not in output, output
    result, output = run('--terminals', session='wayland')
    assert result.returncode == 1 and 'invalid terminal configuration' in output, output
    assert 'arranging separate process terminals through XWayland' in output, output
    print('PASS: respect X11 sessions and explicit --terminals')

    for session in ('x11', 'wayland'):
        result, output = run('--no-terminals', session=session)
        assert result.returncode == 0 and 'worker started' in output and 'web_gui started' in output, output
        assert 'arranging separate process terminals through XWayland' not in output, output
    result, output = run('--terminals', '--no-terminals')
    assert result.returncode == 1 and 'cannot be combined' in output, output
    print('PASS: explicit inline mode and mutually exclusive options')
