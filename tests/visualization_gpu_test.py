"""Opt-in real RTX 5090 / X11 test. Run inside run_can_hal_vcan_test.sh.

Uses private SHM and temporary daemon configs; never attaches to a user session.
Requires the normal build, visualization build and analysis Python dependencies.
"""

import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys
import tempfile

import yaml
from daemon_test_config import default_simulation_manifest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/analysis"))
from log_data import snapshots


def execute(args, timeout=90):
    result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout + result.stderr


def check_ppm(path):
    with path.open("rb") as image:
        assert image.readline() == b"P6\n"
        width, height = map(int, image.readline().split())
        assert image.readline() == b"255\n"
        pixels = image.read()
    assert len(pixels) == width * height * 3
    assert statistics.pstdev(pixels[::3]) > 10, "render output appears blank"
    assert len(set(zip(pixels[::3], pixels[1::3], pixels[2::3]))) > 500


def physics_case(directory, renderer_delay):
    directory.mkdir()
    shm = f"/qhrr-visualization-gpu-{os.getpid()}-{renderer_delay}"
    logger = yaml.safe_load((ROOT / "config/shm_logger/shm_logger.yaml").read_text())
    logger["logging"]["output_dir"] = str(directory / "recordings")
    logger["logging"]["storage"]["minimum_free_gib"] = 0
    logger["logging"]["flight_recorder"]["enabled"] = False
    logger_path = directory / "logger.yaml"
    logger_path.write_text(yaml.safe_dump(logger))
    config = default_simulation_manifest(directory / "session", ROOT / "config/daemon/visualization.yaml")
    config["shm_name"] = shm
    config["log_directory"] = str(directory / "logs")
    config["processes"] = [p for p in config["processes"] if p["name"] != "input"]
    if renderer_delay < 0:
        config["processes"] = [p for p in config["processes"] if p["name"] != "visualization"]
    for process in config["processes"]:
        process["exec_path"] = str((ROOT / "build" / process["exec_path"]).resolve())
        process["working_directory"] = str(ROOT)
        if process["name"] == "simulation":
            process["args"] += ["--duration", "8"]
        if process["name"] == "visualization":
            process["args"] += [
                "--hidden",
                "--width",
                "640",
                "--height",
                "480",
                "--slow-render-ms",
                str(renderer_delay),
            ]
        if process["name"] == "shm_logger":
            process["args"][1] = str(logger_path)
    path = directory / "daemon.yaml"
    path.write_text(yaml.safe_dump(config))
    output = execute([str(ROOT / "build/daemon"), "--config", str(path), "--no-terminals"], 40)
    records = list((directory / "recordings").glob("*"))
    assert len(records) == 1, output
    rows = [
        (stamp, values)
        for stamp, channel, values in snapshots(records[0])
        if channel == "qhrr/state" and values["simulation/ground_truth/time"] >= 2
    ]
    assert len(rows) > 1000
    first_stamp, first = rows[0]
    last_stamp, last = rows[-1]
    seconds = (last_stamp - first_stamp) / 1e9
    ratio = (last["simulation/ground_truth/time"] - first["simulation/ground_truth/time"]) / seconds
    control_hz = (last["control/command_sequence"] - first["control/command_sequence"]) / seconds
    model_hz = (last["shm/model_sequence"] - first["shm/model_sequence"]) / seconds
    assert 0.9 < ratio < 1.1, (ratio, output)
    assert 450 < control_hz < 550, control_hz
    # ABI13: the controller publishes its measured-state model at its own tick rate.
    assert 450 < model_hz < 550 and model_hz <= control_hz + 2, (model_hz, control_hz)
    result = dict(physics_realtime_ratio=ratio, control_hz=control_hz, model_hz=model_hz)
    if renderer_delay >= 0:
        stats = re.search(r"\[visualizer\] frames=(\d+).*max_shm_hold_ns=(\d+)", output)
        assert stats and int(stats[1]) >= 5, output
        assert "filament / vulkan" in output and "NVIDIA GeForce RTX 5090" in output, output
        result.update(rendered_frames=int(stats[1]), max_shm_hold_ns=int(stats[2]))
    assert not Path("/dev/shm", shm[1:]).exists()
    return result


with tempfile.TemporaryDirectory(prefix="qhrr-visualization-gpu-") as temporary:
    directory = Path(temporary)
    for backend in ("filament", "classic"):
        path = directory / f"{backend}.ppm"
        output = execute(
            [
                str(ROOT / "scripts/run_visualization.sh"),
                "--preview",
                "--backend",
                backend,
                "--hidden",
                "--width",
                "640",
                "--height",
                "480",
                "--frames",
                "30",
                "--screenshot",
                str(path),
            ]
        )
        check_ppm(path)
        print(backend, output, flush=True)
    baseline = physics_case(directory / "baseline", -1)
    delayed = physics_case(directory / "delayed", 200)
    assert abs(baseline["physics_realtime_ratio"] - delayed["physics_realtime_ratio"]) < 0.05
    print(json.dumps(dict(baseline=baseline, render_delay_200ms=delayed), indent=2), flush=True)
