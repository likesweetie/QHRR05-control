#!/usr/bin/env python3
"""Explicit offline-asset preparation: download a pinned CC0 HDRI, prefilter with cmgen."""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
SOURCE_URL = "https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/1k/studio_small_09_1k.hdr"
SOURCE_SHA256 = "e7cfda5f4e98e623db12b8bfd0184e048488e4855d9c83e2751fb44a32e80c45"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmgen", type=Path, default=ROOT / "build-filament/native/bin/cmgen")
    args = parser.parse_args()
    if not args.cmgen.is_file():
        parser.error("cmgen is missing; build the visualization first")
    cache = ROOT / "build-filament/environment-source"
    cache.mkdir(parents=True, exist_ok=True)
    source = cache / "studio_small_09_1k.hdr"
    if not source.exists():
        print("Downloading CC0 Studio Small 09 from Poly Haven", flush=True)
        with urlopen(SOURCE_URL, timeout=60) as response:
            payload = response.read()
        if hashlib.sha256(payload).hexdigest() != SOURCE_SHA256:
            raise RuntimeError("downloaded HDRI SHA256 mismatch")
        source.write_bytes(payload)
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise RuntimeError("cached HDRI SHA256 mismatch; source has been modified")
    processed = cache / "processed"
    subprocess.run(
        [str(args.cmgen), "--format=ktx", "--size=256", f"--deploy={processed}", str(source)],
        check=True,
    )
    destination = ROOT / "models/qhrr-05/visualization/environment/studio_small_09_ibl.ktx"
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(processed / "processed_ibl.ktx", destination)
    print(f"Prepared {destination} ({destination.stat().st_size} bytes)", flush=True)
    print("SHA256: " + hashlib.sha256(destination.read_bytes()).hexdigest(), flush=True)


if __name__ == "__main__":
    main()
