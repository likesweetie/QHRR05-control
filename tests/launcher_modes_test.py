"""Run in a test-owned namespace; exercise normal host-mode setup in that namespace."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

launcher = Path(sys.argv[1]).resolve()
namespace = os.readlink('/proc/self/ns/net')


def links():
    return json.loads(subprocess.check_output(['ip', '-json', '-details', 'link', 'show']))


def vcan():
    return next((link for link in links() if link['ifname'] == 'vcan0'), None)


with tempfile.TemporaryDirectory(prefix='qhrr05-launcher-') as directory:
    directory = Path(directory)
    marker = directory / 'child-namespace'
    spec = {
        'name': 'namespace-probe',
        'exec_path': '/bin/sh',
        'args': ['-c', 'readlink /proc/self/ns/net > "$1"', 'probe', str(marker)],
    }
    config = directory / 'processes.json'
    config.write_text(json.dumps({'processes': [spec], 'debug_processes': [spec]}))

    def run(*args, expected=0):
        marker.unlink(missing_ok=True)
        result = subprocess.run(
            [str(launcher), '--process-config', str(config), *args],
            capture_output=True,
            text=True,
            timeout=12,
        )
        assert result.returncode == expected, result.stdout + result.stderr
        return result

    assert vcan() is None, 'Test namespace must start without vcan0'
    run()
    assert marker.read_text().strip() == namespace, 'Default launcher created a hidden namespace'
    link = vcan()
    assert link['linkinfo']['info_kind'] == 'vcan' and 'UP' in link['flags']
    original_index = link['ifindex']
    print('PASS: default creates caller-visible vcan0 UP; interface survives daemon exit')

    run()
    assert vcan()['ifindex'] == original_index, 'Existing vcan0 was replaced'
    subprocess.run(['ip', 'link', 'set', 'dev', 'vcan0', 'down'], check=True)
    run()
    assert vcan()['ifindex'] == original_index and 'UP' in vcan()['flags']
    print('PASS: existing UP reused, DOWN activated without replacing interface')

    subprocess.run(['ip', 'link', 'delete', 'dev', 'vcan0'], check=True)
    subprocess.run(['ip', 'link', 'add', 'dev', 'vcan0', 'type', 'dummy'], check=True)
    result = run(expected=1)
    assert 'not a vcan device' in result.stderr and not marker.exists()
    assert vcan()['linkinfo']['info_kind'] == 'dummy' and 'UP' not in vcan()['flags']
    print('PASS: conflicting device rejected unchanged; no daemon or automatic debug path')

    result = run('--isolated-vcan')
    assert marker.read_text().strip() != namespace
    assert vcan()['linkinfo']['info_kind'] == 'dummy', 'Debug namespace touched caller interface'
    assert 'DEBUG' in result.stdout
    print('PASS: only explicit --isolated-vcan creates a private bus')

    run('--without-can')
    assert marker.read_text().strip() == namespace and vcan()['linkinfo']['info_kind'] == 'dummy'
    run('--existing-can')
    assert marker.read_text().strip() == namespace and vcan()['linkinfo']['info_kind'] == 'dummy'

    for pair in [
        ['--without-can', '--isolated-vcan'],
        ['--without-can', '--existing-can'],
        ['--isolated-vcan', '--existing-can'],
    ]:
        result = run(*pair, expected=1)
        assert 'cannot be combined' in result.stderr and not marker.exists()

    print('PASS: explicit bypass modes preserve interfaces; conflicting flags fail')
