"""Exercise launcher recovery in a temporary checkout, never deleting the user's build."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

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
    assert result.returncode == expected, result.stdout + result.stderr
    return result.stdout + result.stderr


with tempfile.TemporaryDirectory(prefix="qhrr-launcher-build-") as folder:
    root = Path(folder)
    scripts = root / "scripts"
    scripts.mkdir()
    for name in (
        "ensure_build.sh",
        "run_simulation.sh",
        "simulation_config.py",
        "run_visualization.sh",
        "visualization_process.sh",
    ):
        shutil.copy2(ROOT / "scripts" / name, scripts / name)
    (root / "config/daemon").mkdir(parents=True)
    (root / "config/daemon/daemon.yaml").write_text(
        'runtime_mode: hardware\nshm_name: /launcher_test\nprocesses:\n'
        '  - name: control\n    exec_path: control\n    working_directory: ../..\n    args: [control.yaml, "{shm_name}"]\n')
    (root / "config/can_hal").mkdir()
    (root / "config/can_hal/can_hal.yaml").write_text('interface: can0\nmotor_config: motor.yaml\n')
    (root / "main.cpp").write_text(
        '#include <iostream>\nint main() {std::cout << "NATIVE_LAUNCHED\\n";}\n'
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
    ready = run(root, "run_simulation.sh", *args)
    assert "automatically building" not in ready and "NATIVE_LAUNCHED" in ready
    (root / "build" / "simulation").unlink()
    assert "automatically building" in run(root, "run_simulation.sh", *args)
    print("PASS: daemon builds missing children, reuses complete builds, runs from another cwd")

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
    print("PASS: complete build deletion and concurrent launchers perform one recovery build")

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
