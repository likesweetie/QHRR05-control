"""Kill a private simulator, then restart its daemon/session through the stale socket."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
name = f"/qhrr-restart-{os.getpid()}"
socket_path = Path(f"/tmp/qhrr05-simulation-{os.getuid()}/{name[1:]}.sock")


def wait_for(predicate, log):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(log.read_text())


with tempfile.TemporaryDirectory(prefix="qhrr-restart-") as folder:
    directory = Path(folder)
    spec = {
        "name": "simulation",
        "exec_path": str(ROOT / "build/simulation"),
        "working_directory": str(ROOT),
        "wait_ready": True,
        "args": [
            "--headless",
            "--without-can",
            "--shm-name",
            name,
            "--model",
            str(ROOT / "models/qhrr-05/scene.xml"),
            "--control-config",
            str(ROOT / "config/control/control.yaml"),
        ],
    }
    config = directory / "daemon.json"
    config.write_text(
        json.dumps(
            {
                "shm_name": name,
                "monitoring": {"display_interval_sec": 2, "window_sec": 10, "sample_hz": 100},
                "processes": [spec],
                "debug_processes": [spec],
            }
        )
    )
    daemon = None
    try:
        for attempt in range(2):
            log = directory / f"attempt-{attempt}.log"
            with log.open("w") as output:
                daemon = subprocess.Popen(
                    [
                        str(ROOT / "build/daemon"),
                        "--config",
                        str(config),
                        "--without-can",
                        "--no-terminals",
                    ],
                    stdout=output,
                    stderr=subprocess.STDOUT,
                )
            wait_for(lambda: "independent control superloop" in log.read_text(), log)
            if attempt == 0:
                child_file = Path(f"/proc/{daemon.pid}/task/{daemon.pid}/children")
                wait_for(lambda: bool(child_file.read_text().strip()), log)
                children = child_file.read_text().split()
                assert len(children) == 1, children
                os.kill(int(children[0]), signal.SIGKILL)
                assert daemon.wait(timeout=10) != 0, log.read_text()
                assert socket_path.exists(), "abrupt simulator exit did not leave its socket"
                assert not Path(
                    "/dev/shm", name[1:]
                ).exists(), "daemon did not release its hardware SHM"
            else:
                assert "removed stale endpoint (no listener)" in log.read_text(), log.read_text()
                daemon.send_signal(signal.SIGINT)
                assert daemon.wait(timeout=10) == 0, log.read_text()
                assert not socket_path.exists(), "graceful restart did not remove its endpoint"
        print(
            "PASS: real simulator SIGKILL -> daemon failure cleanup -> same-session restart recovers stale IPC"
        )
    finally:
        if daemon is not None and daemon.poll() is None:
            daemon.send_signal(signal.SIGINT)
            daemon.wait(timeout=10)
