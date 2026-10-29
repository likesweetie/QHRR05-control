"""Download exact, SHA256-pinned official LLVM packages into a local toolchain."""

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from urllib.request import urlopen

root = Path(__file__).resolve().parents[1]
destination = Path(sys.argv[1])
packages = json.loads((root / "visualization/toolchain-lock.json").read_text())


def fetch(package):
    path = destination / "llvm18-debs" / Path(package["Filename"]).name
    if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != package["SHA256"]:
        print("[toolchain] " + package["Package"], flush=True)
        with urlopen("https://apt.llvm.org/jammy/" + package["Filename"], timeout=90) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != package["SHA256"]:
            raise RuntimeError("LLVM package SHA256 mismatch: " + package["Package"])
        path.write_bytes(data)
    return path


with ThreadPoolExecutor(max_workers=4) as pool:
    paths = list(pool.map(fetch, packages))
for path in paths:
    subprocess.run(["dpkg-deb", "--extract", str(path), str(destination)], check=True)
