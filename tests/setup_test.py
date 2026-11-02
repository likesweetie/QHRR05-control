"""Exercise initial setup without installing packages, accessing CAN, or networking."""

import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SHIM = '''#!PYTHON
import json, os, shutil, sys
from pathlib import Path
tool, args = Path(sys.argv[0]).name, sys.argv[1:]
with open(os.environ['SETUP_TRACE'], 'a') as stream:
    stream.write(json.dumps([tool, args]) + '\\n')
if os.environ.get('FAIL_TOOL') == tool:
    sys.exit(17)
if tool == 'uname':
    print('Linux' if args == ['-s'] else os.environ.get('SETUP_ARCH', 'x86_64'))
elif tool == 'sudo':
    args = args[1:] if args[0] == '--' else args
    os.execvp(args[0], args)
elif tool == 'node':
    compatible = '.cache/setup/' in sys.argv[0] or os.environ.get('NODE_COMPAT', '1') == '1'
    if args and args[0] == '-e':
        sys.exit(0 if compatible else 1)
    print('v22.99.1' if compatible else 'v18.0.0')
elif tool == 'python3' and args[:2] == ['-m', 'venv']:
    target = Path(args[2]) / 'bin/python'
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(sys.argv[0], target)
elif tool == 'curl':
    output = Path(args[args.index('-o') + 1])
    source = Path(os.environ['NODE_SOURCE'])
    name = 'SHASUMS256.txt' if any(a.endswith('/SHASUMS256.txt') for a in args) else 'node-v22.99.1-linux-x64.tar.xz'
    shutil.copyfile(source / name, output)
elif tool in ('ip', 'modprobe', 'daemon'):
    raise SystemExit('setup must not start a runtime or configure CAN')
'''.replace('PYTHON', sys.executable, 1)


class SetupTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='qhrr-setup-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name) / 'project with spaces'
        self.root.mkdir()
        scripts = self.root / 'scripts'
        scripts.mkdir()
        for name in ('setup.sh', 'ensure_build.sh', 'run_web_gui.sh'):
            shutil.copy2(ROOT / 'scripts' / name, scripts / name)
        (scripts / 'build_visualization.sh').write_text(SHIM)
        (scripts / 'build_visualization.sh').chmod(0o755)
        for file in ('third_party/mujoco/include/mujoco/mujoco.h',
                     'third_party/mujoco/lib/libmujoco.so',
                     'third_party/data_tamer/data_tamer_cpp/CMakeLists.txt',
                     'third_party/mcap/cpp/mcap/include/mcap/reader.hpp',
                     'tools/analysis/requirements.txt'):
            path = self.root / file
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture')
        self.bin = Path(temporary.name) / 'bin'
        self.bin.mkdir()
        for tool in ('uname', 'sudo', 'apt-get', 'python3', 'node', 'npm', 'cmake',
                     'pkg-config', 'curl', 'ip', 'modprobe', 'daemon'):
            path = self.bin / tool
            path.write_text(SHIM)
            path.chmod(0o755)
        self.trace = Path(temporary.name) / 'trace.jsonl'
        self.downloads = Path(temporary.name) / 'downloads'
        self.downloads.mkdir()
        archive = self.downloads / 'node-v22.99.1-linux-x64.tar.xz'
        with tarfile.open(archive, 'w:xz') as tar:
            for name in ('node', 'npm'):
                data = SHIM.encode()
                info = tarfile.TarInfo(f'node-v22.99.1-linux-x64/bin/{name}')
                info.size, info.mode = len(data), 0o755
                tar.addfile(info, io.BytesIO(data))
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        (self.downloads / 'SHASUMS256.txt').write_text(f'{digest}  {archive.name}\n')
        self.env = {**os.environ, 'PATH': str(self.bin) + os.pathsep + os.environ['PATH'],
                    'SETUP_TRACE': str(self.trace), 'NODE_SOURCE': str(self.downloads)}

    def run_setup(self, *args, success=True, **env):
        result = subprocess.run(['bash', str(self.root / 'scripts/setup.sh'), *args],
                                cwd='/tmp', env={**self.env, **env}, capture_output=True,
                                text=True, timeout=15)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        return result.stdout + result.stderr

    def calls(self, tool):
        rows = [json.loads(line) for line in self.trace.read_text().splitlines()]
        return [args for name, args in rows if name == tool]

    def test_dry_run_changes_nothing(self):
        result = self.run_setup('--dry-run', NODE_COMPAT='0')
        for package in ('libzstd-dev', 'liblz4-dev', 'python3-yaml', 'can-utils', 'gnome-terminal'):
            self.assertIn(package, result)
        for tool in ('apt-get', 'curl', 'python3', 'python', 'npm', 'cmake', 'ip', 'daemon'):
            self.assertEqual(self.calls(tool), [], tool)
        self.assertFalse((self.root / '.venv').exists())
        self.assertFalse((self.root / '.cache').exists())

    def test_system_dependencies_venv_and_web_build(self):
        self.run_setup('--jobs', '3', '--with-visualization')
        self.assertEqual(self.calls('apt-get')[0], ['update'])
        self.assertIn('libzstd-dev', self.calls('apt-get')[1])
        self.assertIn('liblz4-dev', self.calls('apt-get')[1])
        self.assertEqual(self.calls('curl'), [])
        self.assertIn(['-m', 'pip', 'install', '-r', str(self.root / 'tools/analysis/requirements.txt')],
                      self.calls('python'))
        self.assertIn(['--build', str(self.root / 'build'), '--parallel', '3'], self.calls('cmake'))
        self.assertTrue(any('-DQHRR_ANALYSIS_PYTHON=' in arg for args in self.calls('cmake') for arg in args))
        self.assertEqual([args[-1] for args in self.calls('npm')], ['--no-fund', '--depth=0', 'build'])
        self.assertEqual(self.calls('build_visualization.sh'), [['--jobs', '3']])
        self.assertEqual(self.calls('ip'), [])
        self.assertEqual(self.calls('daemon'), [])
        found = subprocess.check_output(
            ['bash', '-c', 'source "$1"; qhrr_use_local_tools "$2"; command -v node; node --version',
             'test', str(self.root / 'scripts/ensure_build.sh'), str(self.root)],
            env={**self.env, 'PATH': '/usr/bin:/bin'}, text=True)
        self.assertTrue(found.startswith(str(self.root / '.cache/setup/node/bin/node') + '\n'))
        self.trace.write_text('')
        self.run_setup('--skip-system')
        self.assertEqual(self.calls('apt-get'), [])
        self.assertEqual(self.calls('curl'), [])

    def test_old_node_installs_verified_local_copy(self):
        self.run_setup('--skip-system', NODE_COMPAT='0')
        target = self.root / '.cache/setup/node'
        self.assertTrue(target.is_symlink())
        self.assertTrue((target / 'bin/node').is_file())
        self.assertEqual(len(self.calls('curl')), 2)
        self.assertTrue(any('/v22.99.1/node-' in arg for arg in self.calls('curl')[1]))
        found = subprocess.check_output(
            ['bash', '-c', 'source "$1"; qhrr_use_local_tools "$2"; command -v node',
             'test', str(self.root / 'scripts/ensure_build.sh'), str(self.root)],
            env=self.env, text=True).strip()
        self.assertEqual(found, str(target / 'bin/node'))
        # Even a directly launched web script must find the local Node without
        # inheriting an activated shell, while the fake server opens no sockets.
        vite = self.root / 'apps/interfaces/web_gui/frontend/node_modules/.bin/vite'
        vite.parent.mkdir(parents=True)
        vite.write_text('#!/bin/sh\nexit 0\n')
        vite.chmod(0o755)
        bridge = self.root / 'fake-bridge'
        bridge.write_text('#!/bin/sh\nexit 0\n')
        bridge.chmod(0o755)
        web = subprocess.run(['bash', str(self.root / 'scripts/run_web_gui.sh'), '--bridge', str(bridge)],
                             env={**self.env, 'NODE_COMPAT': '0'}, capture_output=True, text=True)
        self.assertEqual(web.returncode, 0, web.stdout + web.stderr)
        self.trace.write_text('')
        self.run_setup('--skip-system', NODE_COMPAT='0')
        self.assertEqual(self.calls('curl'), [])

    def test_bad_checksum_does_not_install_or_build(self):
        (self.downloads / 'SHASUMS256.txt').write_text('0' * 64 + '  node-v22.99.1-linux-x64.tar.xz\n')
        result = self.run_setup('--skip-system', success=False, NODE_COMPAT='0')
        self.assertIn('FAILED', result)
        self.assertFalse((self.root / '.cache/setup/node').exists())
        self.assertEqual(self.calls('cmake'), [])
        self.assertEqual(self.calls('npm'), [])

    def test_invalid_options_and_install_failure_stop(self):
        self.run_setup('--jobs', '0', success=False)
        self.assertFalse(self.trace.exists())
        self.run_setup('--unknown', success=False)
        self.run_setup(success=False, FAIL_TOOL='apt-get')
        self.assertEqual(self.calls('cmake'), [])
        self.assertEqual(self.calls('npm'), [])


if __name__ == '__main__':
    unittest.main()
