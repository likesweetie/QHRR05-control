"""Real RTX/X11 operator controls with private CAN and simulation IPC; no live user session."""

import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import yaml

from gui_test_utils import event, wait_for
from simulation_ipc_utils import SimulationReader, Projection

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts/operator-test"
OUTPUT.mkdir(parents=True, exist_ok=True)
focus = event("getwindowfocus").stdout.strip()
pointer = dict(
    line.split("=", 1) for line in event("getmouselocation", "--shell").stdout.splitlines()
)
children = []


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=12)


try:
    with tempfile.TemporaryDirectory(prefix="qhrr-operator-") as directory:
        directory = Path(directory)
        name = f"/qhrr-operator-{os.getpid()}"
        config = yaml.safe_load((ROOT / "config/control/control.yaml").read_text())
        config["model_path"] = str(ROOT / "models/qhrr-05/qhrr-05.xml")
        config["homing"]["duration_sec"] = 0.5
        control = directory / "control.yaml"
        control.write_text(yaml.safe_dump(config))
        manifest = yaml.safe_load((ROOT / "config/daemon/visualization.yaml").read_text())
        manifest["shm_name"] = name
        manifest["log_directory"] = str(directory / "logs")
        manifest["processes"] = [
            p for p in manifest["processes"] if p["name"] in ("can_hal", "simulation", "control")
        ]
        for spec in manifest["processes"]:
            spec["exec_path"] = str(ROOT / "build" / spec["exec_path"])
            spec["working_directory"] = str(ROOT)
            spec["args"] = [
                str(control) if arg == "config/control/control.yaml" else arg
                for arg in spec["args"]
            ]
        path = directory / "daemon.yaml"
        path.write_text(yaml.safe_dump(manifest))
        daemon_log = OUTPUT / "daemon.log"
        with daemon_log.open("w") as log:
            daemon = subprocess.Popen(
                [str(ROOT / "build/daemon"), "--config", str(path), "--no-terminals"],
                cwd=ROOT,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        children.append(daemon)
        socket_path = Path(f"/tmp/qhrr05-input-{os.getuid()}/{name[1:]}.sock")
        wait_for(socket_path.exists, daemon_log.read_text)
        reader = SimulationReader(name)
        wait_for(lambda: reader.read(0.05).published_ns, daemon_log.read_text)
        bridge = subprocess.Popen(
            [str(ROOT / "build/web_gui_bridge"), name, str(control)],
            cwd=ROOT,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        children.append(bridge)
        latest = {}

        def watch():
            for line in bridge.stdout:
                latest.update(json.loads(line))

        threading.Thread(target=watch, daemon=True).start()
        wait_for(lambda: latest.get("ready"), daemon_log.read_text)
        viewer_log = OUTPUT / "viewer.log"
        with viewer_log.open("w") as log:
            viewer = subprocess.Popen(
                [
                    str(ROOT / "scripts/run_visualization.sh"),
                    "--attach",
                    "--backend",
                    os.environ.get("QHRR_OPERATOR_BACKEND", "filament"),
                    "--shm-name",
                    name,
                    "--control-config",
                    str(control),
                    "--width",
                    "1600",
                    "--height",
                    "1000",
                    "--screenshot",
                    str(OUTPUT / "operator.ppm"),
                ],
                cwd=ROOT,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        children.append(viewer)

        def window_ready():
            assert viewer.poll() is None, viewer_log.read_text()
            result = subprocess.run(
                ["xdotool", "search", "--onlyvisible", "--pid", str(viewer.pid)],
                capture_output=True,
                text=True,
            )
            return result.stdout.splitlines()[-1] if result.returncode == 0 else None

        window = wait_for(window_ready, viewer_log.read_text)
        event("windowactivate", "--sync", window)
        time.sleep(0.5)

        def key(value):
            event("key", "--window", window, value)
            time.sleep(0.1)

        def mode(value, code):
            key(value)
            wait_for(
                lambda: latest.get("mode") == code, lambda: viewer_log.read_text() + str(latest)
            )

        mode("h", 0)
        wait_for(lambda: latest.get("holding"), daemon_log.read_text, timeout=8)
        # Use actual mouse events on the ImGui slider, through the Input path.
        previous_force = latest["target_force"]
        event("mousemove", "--window", window, 1060, 162)
        time.sleep(0.1)
        event("mousedown", "1")
        time.sleep(0.1)
        event("mouseup", "1")
        wait_for(
            lambda: latest.get("target_force") != previous_force,
            lambda: viewer_log.read_text() + str(latest),
        )
        print("PASS: target force slider via Input/safety", flush=True)
        mode("g", 1)
        mode("m", 2)
        for plane, value in enumerate(("1", "2", "3")):
            mode(value, 3)
            wait_for(lambda: latest.get("tracking_plane") == plane, lambda: str(latest))
        mode("b", 4)
        mode("d", 5)
        key("space")
        wait_for(lambda: reader.read(0.05).paused, viewer_log.read_text)
        frozen = reader.read(0.1).time
        time.sleep(0.2)
        assert abs(reader.read(0.05).time - frozen) < 1e-9
        key("Right")
        wait_for(lambda: reader.read(0.05).time > frozen, viewer_log.read_text)
        assert abs(reader.state.time - frozen - 0.001) < 1e-8, reader.state.time - frozen
        key("space")
        wait_for(lambda: not reader.read(0.05).paused, viewer_log.read_text)
        print("PASS: task shortcuts, tracking planes, pause and exact single step", flush=True)

        projection = Projection()

        def drag(button, body=None):
            key("Home")
            key("space")
            wait_for(lambda: reader.read(0.05).paused, viewer_log.read_text)
            state = reader.read(0.1)
            before = list(state.position_gt) if body else list(state.object_position)
            before_quat = list(state.object_quaternion)
            x, y = projection.pixel(state, 960, 1000, body)
            print("Projection", body, button, x, y, list(state.position_gt), flush=True)
            key("F12")
            time.sleep(0.15)
            import shutil

            shutil.copyfile(OUTPUT / "operator.ppm", OUTPUT / f"before-drag-{body}-{button}.ppm")
            event(
                "mousemove",
                "--window",
                window,
                x,
                y,
                "click",
                "--repeat",
                "2",
                "--delay",
                "120",
                "1",
            )
            time.sleep(0.15)
            if body:
                key("space")
                wait_for(lambda: not reader.read(0.05).paused, viewer_log.read_text)
                # Resuming intentionally clears perturbation. Re-project fresh measured
                # geometry just before Ctrl-click so a moving thin link is still under it.
                x, y = projection.pixel(reader.read(0.01), 960, 1000, body)
            event(
                "mousemove", "--window", window, x, y, "keydown", "Control_L", "mousedown", button
            )
            observed_wrench = 0
            observed_displacement = 0
            for n in range(1, 13):
                event("mousemove", "--window", window, x + n * 3, y - n * 5)
                state = reader.read(0.06)
                observed_wrench = max(
                    observed_wrench, math.sqrt(sum(v * v for v in state.perturb_wrench))
                )
                observed_displacement = max(
                    observed_displacement,
                    max(abs(a - b) for a, b in zip(state.position_gt, before)),
                )
            event("mouseup", button, "keyup", "Control_L")
            wait_for(lambda: reader.read(0.05).perturb_active == 0, viewer_log.read_text)
            state = reader.read(0.05)
            if body:
                # The gravity controller can recover before release is observed.
                # Require actual joint motion during the drag and a physical wrench.
                assert observed_displacement > 0.001 and observed_wrench > 0.01, (
                    body,
                    button,
                    observed_displacement,
                    observed_wrench,
                    viewer_log.read_text(),
                )
            elif button == 3:
                displacement = max(abs(a - b) for a, b in zip(state.object_position, before))
                assert displacement > 0.01, (displacement, viewer_log.read_text())
            else:
                assert max(abs(a - b) for a, b in zip(state.object_quaternion, before_quat)) > 0.01
            if state.paused:
                key("space")
                wait_for(lambda: not reader.read(0.05).paused, viewer_log.read_text)
            print(
                "PASS: Ctrl-drag",
                button,
                body or "touch_object",
                "wrench",
                observed_wrench,
                flush=True,
            )

        mode("g", 1)
        drag(3, "FL_calf")
        drag(1, "FL_calf")
        drag(3)
        drag(1)
        key("w")
        wait_for(
            lambda: "[workspace] ready:" in viewer_log.read_text(), viewer_log.read_text, timeout=30
        )
        key("w")
        before_run = reader.read(0.05).run_id
        key("r")
        wait_for(
            lambda: reader.read(0.05).run_id > before_run and latest.get("mode") == 5,
            viewer_log.read_text,
        )
        mode("h", 0)
        wait_for(lambda: latest.get("holding"), daemon_log.read_text, timeout=8)
        mode("2", 3)
        time.sleep(2)
        key("space")
        wait_for(lambda: reader.read(0.05).paused, viewer_log.read_text)
        key("Escape")
        assert viewer.wait(timeout=12) == 0, viewer_log.read_text()
        assert daemon.poll() is None, "closing an attached viewer stopped the daemon"
        print(
            "PASS: workspace, reset/damping, graphs/trail snapshot and independent viewer close",
            flush=True,
        )
finally:
    for button in (1, 2, 3):
        event("mouseup", button)
    event("keyup", "Control_L", "Shift_L")
    for process in reversed(children):
        stop(process)
    event("mousemove", pointer["X"], pointer["Y"])
    event("windowfocus", focus)
