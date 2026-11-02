"""Exercise launcher incremental builds and recovery in a temporary checkout."""

import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def run(root, script, *args, expected=0, env=None):
    result = subprocess.run(
        [str(root / "scripts" / script), *args],
        cwd="/tmp",
        env={**os.environ, "QHRR_BUILD_JOBS": "2", **(env or {})},
        capture_output=True,
        text=True,
        timeout=60,
    )
    if expected is None:
        assert result.returncode != 0, result.stdout + result.stderr
    else:
        assert result.returncode == expected, result.stdout + result.stderr
    return result.stdout + result.stderr


with tempfile.TemporaryDirectory(prefix="qhrr-launcher-build-") as folder:
    root = Path(folder)
    scripts = root / "scripts"
    scripts.mkdir()
    for name in (
        "ensure_build.sh",
        "run_daemon.sh",
        "run_simulation.sh",
        "simulation_config.py",
        "run_visualization.sh",
        "visualization_process.sh",
    ):
        shutil.copy2(ROOT / "scripts" / name, scripts / name)
    # Hardware launcher coverage uses a local CAN-preparation stub, never a device.
    (scripts / "prepare_can.py").write_text('print("CAN_PREPARED", flush=True)\n')
    (root / "config/daemon").mkdir(parents=True)
    (root / "config/daemon/daemon.yaml").write_text(
        'runtime_mode: hardware\nshm_name: /launcher_test\nprocesses:\n'
        '  - name: control\n    exec_path: control\n    working_directory: ../..\n    args: [control.yaml, "{shm_name}"]\n')
    (root / "config/can_hal").mkdir()
    (root / "config/can_hal/can_hal.yaml").write_text('interface: can0\nmotor_config: motor.yaml\n')
    header = root / "message.hpp"
    header.write_text('#define LAUNCH_MESSAGE "NATIVE_LAUNCHED"\n')
    source = root / "main.cpp"
    source.write_text(
        '#include <iostream>\n#include "message.hpp"\n'
        '#ifndef LAUNCH_SUFFIX\n#define LAUNCH_SUFFIX ""\n#endif\n'
        'int main() {std::cout << LAUNCH_MESSAGE << LAUNCH_SUFFIX << "\\n";}\n'
    )
    cmake = root / "CMakeLists.txt"
    cmake.write_text(
        '''cmake_minimum_required(VERSION 3.22)
project(LauncherRecovery CXX)
foreach(name daemon simulation control can_hal input shm_logger qhrr_logctl web_gui_bridge)
  add_executable(${name} main.cpp)
endforeach()
set_target_properties(qhrr_logctl PROPERTIES OUTPUT_NAME qhrr-logctl)
'''
    )
    args = ("--without-can", "--no-terminals")
    first = run(root, "run_simulation.sh", *args)
    assert "automatically building" in first and "NATIVE_LAUNCHED" in first
    for name in (
        "daemon",
        "simulation",
        "control",
        "can_hal",
        "input",
        "shm_logger",
        "qhrr-logctl",
        "web_gui_bridge",
    ):
        assert os.access(root / "build" / name, os.X_OK), name
    daemon = root / "build" / "daemon"
    before = daemon.stat().st_mtime_ns
    ready = run(root, "run_simulation.sh", *args)
    assert "Checking native source changes" in ready and "NATIVE_LAUNCHED" in ready
    assert daemon.stat().st_mtime_ns == before, "unchanged executable was rebuilt"
    (root / "build" / "simulation").unlink()
    assert "automatically building" in run(root, "run_simulation.sh", *args)
    print("PASS: missing binaries recovered; unchanged native targets are not rebuilt")

    source.write_text(source.read_text().replace('<< "\\n";', '<< " SOURCE_UPDATED\\n";'))
    changed = run(root, "run_daemon.sh", "--no-terminals")
    assert "NATIVE_LAUNCHED SOURCE_UPDATED" in changed
    assert "CAN_PREPARED" in changed
    assert changed.index("Checking native source changes") < changed.index("CAN_PREPARED")
    assert changed.index("Previous session cleanup complete") < changed.index("CAN_PREPARED")
    header.write_text('#define LAUNCH_MESSAGE "NATIVE_LAUNCHED_HEADER_UPDATED"\n')
    changed = run(root, "run_daemon.sh", "--no-terminals")
    assert "NATIVE_LAUNCHED_HEADER_UPDATED SOURCE_UPDATED" in changed
    cmake.write_text(cmake.read_text() +
                     '\ntarget_compile_definitions(daemon PRIVATE LAUNCH_SUFFIX=" CMAKE_UPDATED")\n')
    changed = run(root, "run_daemon.sh", "--no-terminals")
    assert "NATIVE_LAUNCHED_HEADER_UPDATED CMAKE_UPDATED SOURCE_UPDATED" in changed
    before = daemon.stat().st_mtime_ns
    run(root, "run_daemon.sh", "--no-terminals")
    assert daemon.stat().st_mtime_ns == before, "unchanged daemon was rebuilt"
    print("PASS: hardware launcher rebuilds source/header/CMake changes before CAN preparation")

    working_source = source.read_text()
    source.write_text("#error intentional_compile_failure\n")
    failed = run(root, "run_daemon.sh", "--no-terminals", expected=None)
    assert "intentional_compile_failure" in failed
    assert "CAN_PREPARED" not in failed and "NATIVE_LAUNCHED" not in failed
    source.write_text(working_source)
    assert "NATIVE_LAUNCHED" in run(root, "run_daemon.sh", "--no-terminals")
    print("PASS: compile failure never launches an existing stale daemon or prepares CAN")

    shutil.rmtree(root / "build")
    children = [
        subprocess.Popen(
            [str(scripts / "run_simulation.sh"), *args],
            cwd="/tmp",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env={**os.environ, "QHRR_BUILD_JOBS": "2"},
        )
        for _ in range(2)
    ]
    outputs = [child.communicate(timeout=60)[0] for child in children]
    assert all(child.returncode == 0 for child in children), outputs
    assert sum(output.count("automatically building") for output in outputs) == 1, outputs
    print("PASS: concurrent launchers serialize build checks and perform one recovery build")

    shutil.rmtree(root / "build")
    cmake.write_text(
        'cmake_minimum_required(VERSION 3.22)\nmessage(FATAL_ERROR "intentional build rejection")\n'
    )
    failed = run(root, "run_simulation.sh", *args, expected=1)
    assert "intentional build rejection" in failed and "NATIVE_LAUNCHED" not in failed
    bad_jobs = run(root, "run_simulation.sh", *args, expected=1, env={"QHRR_BUILD_JOBS": "invalid"})
    assert "positive integer" in bad_jobs and "NATIVE_LAUNCHED" not in bad_jobs
    print("PASS: build errors and invalid jobs fail visibly before any process launch")

    # Only this temporary fixture replaces the expensive GPU build with a test double.
    # The real launcher and its resource checks are unchanged.
    build_script = scripts / "build_visualization.sh"
    build_script.write_text(
        '''#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
echo build >> "$root/visual-builds"
out="$root/build-filament/native/bin"
mkdir -p "$out/assets" "$root/build-filament/native/src/render/filament/assets"
printf '#!/usr/bin/env bash\necho "VISUALIZER_LAUNCHED $*"\n' > "$out/qhrr_visualizer"
chmod +x "$out/qhrr_visualizer"
for name in ibl.ktx pbr.filamat qhrr_pbr.filamat decor.filamat; do echo fixture > "$root/build-filament/native/src/render/filament/assets/$name"; done
'''
    )
    build_script.chmod(0o755)
    environment = root / "models/qhrr-05/visualization/environment"
    environment.mkdir(parents=True)
    (environment / "product_studio_ibl.ktx").write_text("fixture")
    icd = root / "nvidia.json"
    icd.write_text("fixture")
    env = {"QHRR_NVIDIA_ICD": str(icd)}
    first = run(root, "run_visualization.sh", "--preview", "--frames", "2", env=env)
    assert "Preparing the pinned visualizer" in first
    assert "VISUALIZER_LAUNCHED --backend filament" in first and "--preview" in first
    assert len((root / "visual-builds").read_text().splitlines()) == 1
    forwarded = run(root, "run_visualization.sh", "--preview", "--render-scale", "2",
                    "--frames", "7", "--screenshot", "/tmp/qhrr-launcher-test.ppm", env=env)
    assert "--frames 7 --screenshot /tmp/qhrr-launcher-test.ppm" in forwarded
    run(root, "run_visualization.sh", "--preview", env=env)
    assert len((root / "visual-builds").read_text().splitlines()) == 1
    (root / "build-filament/native/bin/assets/pbr.filamat").unlink()
    run(root, "run_visualization.sh", "--preview", env=env)
    assert len((root / "visual-builds").read_text().splitlines()) == 2
    run(root, "run_visualization.sh", "--preview", "--setup", env=env)
    assert len((root / "visual-builds").read_text().splitlines()) == 3
    (root / "build-filament/native/bin/assets/qhrr_pbr.filamat").unlink()
    run(root, "run_visualization.sh", "--preview", env=env)
    assert len((root / "visual-builds").read_text().splitlines()) == 4
    print(
        "PASS: visualizer rebuilds missing binary/shaders, retains Vulkan backend, supports forced setup"
    )
    shutil.rmtree(root / "build-filament")
    bad = run(root, "run_visualization.sh", "--attach", "--no-terminals", expected=1, env=env)
    assert "cannot be combined" in bad and not (root / "build-filament").exists()
    build_script.write_text(
        '#!/usr/bin/env bash\necho "intentional shader build failure" >&2\nexit 7\n'
    )
    failed = run(root, "run_visualization.sh", "--preview", expected=7, env=env)
    assert "intentional shader build failure" in failed and "VISUALIZER_LAUNCHED" not in failed
    print("PASS: invalid launcher options/build failures do not launch or switch backend")

    # Exercise only the cleanup helper with isolated names; never touch can0 or
    # the real project's running daemon. Fake workers live in this fixture.
    import json
    worker = root / 'build/stale_worker'
    worker.parent.mkdir(exist_ok=True)
    worker.write_text('''#!/usr/bin/env python3
import json, mmap, os, signal, sys, time
from pathlib import Path
if sys.argv[1] == '--terminal-output':
    session, mode, ready = '', 'ignore', sys.argv[-1]
elif sys.argv[1] == '--config':
    session = json.loads(Path(sys.argv[2]).read_text())['shm_name']
    mode, ready = sys.argv[3:5]
else:
    session, mode, ready = sys.argv[1:4]
child = None
if mode == 'tree':
    child = os.fork()
    if not child:
        os._exit(0)
    def stop(*_):
        os.waitpid(child, 0)
        sys.exit(0)
    signal.signal(signal.SIGTERM, stop)
elif mode == 'ignore':
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
elif mode == 'mapped':
    mapping = mmap.mmap(os.open('/dev/shm' + session, os.O_RDWR), 0)
Path(ready).write_text(json.dumps({'pid': os.getpid(), 'child': child}))
while True:
    time.sleep(1)
''')
    worker.chmod(0o755)
    stale_daemon = root / 'build/daemon'
    shutil.copy2(worker, stale_daemon)
    outside = root / 'unrelated/stale_worker'
    outside.parent.mkdir()
    shutil.copy2(worker, outside)
    session = f'/qhrr05-launcher-clean-{os.getpid()}'
    cleanup_manifest = root / 'cleanup.json'
    cleanup_manifest.write_text(json.dumps({'runtime_mode': 'hardware', 'shm_name': session}))

    def cleanup(expected=0):
        result = subprocess.run(
            ['bash', '-c', 'source "$1"; qhrr_cleanup_session "$2" "$3"', 'cleanup',
             str(scripts / 'ensure_build.sh'), str(root), str(cleanup_manifest)],
            capture_output=True, text=True, timeout=12)
        assert (result.returncode == 0) == (expected == 0), result.stdout + result.stderr
        return result.stdout + result.stderr

    workers = []

    def start_worker(path, args, name):
        ready = root / (name + '.ready')
        process = subprocess.Popen([str(path), *args, str(ready)])
        workers.append(process)
        deadline = time.monotonic() + 3
        while not ready.exists():
            assert process.poll() is None
            assert time.monotonic() < deadline
            time.sleep(.01)
        return process, json.loads(ready.read_text())

    shm_file = Path('/dev/shm') / session[1:]
    other_shm = Path(str(shm_file) + '-other')
    socket_paths = [Path(f'/tmp/qhrr05-{kind}-{os.getuid()}') / (session[1:] + '.sock')
                    for kind in ('daemon', 'input')]
    socket_paths += [Path(f'/tmp/qhrr05-log-{os.getuid()}-{session[1:]}.sock')]
    try:
        owner, details = start_worker(stale_daemon, ['--config', str(cleanup_manifest), 'tree'], 'owner')
        relay, _ = start_worker(stale_daemon,
            ['--terminal-output', '0', str(owner.pid), 'worker', '2.0',
             str(root / 'logs/old-session.log'), str(owner.pid)], 'relay')
        stubborn, _ = start_worker(worker, [session, 'ignore'], 'stubborn')
        other, _ = start_worker(worker, [session + '-other', 'wait'], 'other')
        unrelated, _ = start_worker(outside, [session, 'wait'], 'unrelated')
        os.kill(owner.pid, signal.SIGSTOP)
        shm_file.write_bytes(b'stale')
        other_shm.write_bytes(b'keep')
        for path in socket_paths:
            path.parent.mkdir(mode=0o700, exist_ok=True)
            with socket.socket(socket.AF_UNIX) as sock:
                sock.bind(str(path))
        result = cleanup()
        assert owner.wait(timeout=2) == 0, result
        assert stubborn.wait(timeout=2) == -signal.SIGKILL, result
        assert relay.wait(timeout=2) == -signal.SIGKILL, result
        assert not Path(f"/proc/{details['child']}").exists(), 'zombie child was not reaped'
        assert other.poll() is None and unrelated.poll() is None, 'unrelated process was stopped'
        assert not shm_file.exists() and other_shm.read_bytes() == b'keep'
        assert all(not path.exists() for path in socket_paths)
        assert 'Force-stopping unresponsive PIDs' in result
        cleanup()  # Repeated cleanup is harmless.
        print('PASS: stopped owner resumes/reaps zombies, unresponsive workers stop, stale IPC removed; other sessions survive')

        shm_file.write_bytes(b'owned by reader')
        reader, _ = start_worker(outside, [session, 'mapped'], 'reader')
        result = cleanup(expected=1)
        assert 'still mapped by PID' in result and shm_file.exists() and reader.poll() is None
        reader.terminate()
        reader.wait(timeout=2)
        cleanup()
        print('PASS: live unrecognized SHM reader prevents unlink and startup')
    finally:
        for process in workers:
            if process.poll() is None:
                process.kill()
            process.wait(timeout=3)
        for path in [shm_file, other_shm, *socket_paths]:
            path.unlink(missing_ok=True)
