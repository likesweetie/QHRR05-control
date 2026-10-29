#!/usr/bin/env python3
"""Prepare calibrated 4K roughness maps from the pinned CC0 ambientCG Metal009 archive."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile
from PIL import Image, ImageFilter, ImageStat

ROOT = Path(__file__).resolve().parents[1]
URL = 'https://ambientcg.com/get?file=Metal009_4K-PNG.zip'
SHA256 = '2e740afb0818258d177a71280076a9a18903e23fff5e2b7563322f6c0e99c6e4'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download', action='store_true', help='explicitly allow fetching the source archive')
    args = parser.parse_args()
    cache = ROOT / '.cache/materials'
    archive = cache / 'Metal009_4K-PNG.zip'
    if not archive.is_file():
        if not args.download:
            parser.error('source archive missing; rerun with --download to fetch the pinned CC0 asset')
        cache.mkdir(parents=True, exist_ok=True)
        request = urllib.request.Request(URL, headers={'User-Agent': 'QHRR05-material-preparation/1.0'})
        with urllib.request.urlopen(request, timeout=60) as source:
            payload = source.read()
        if hashlib.sha256(payload).hexdigest() != SHA256:
            raise RuntimeError('downloaded Metal009 archive hash mismatch')
        archive.write_bytes(payload)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != SHA256:
        raise RuntimeError('cached Metal009 archive hash mismatch')
    destination = ROOT / 'models/qhrr-05/visualization/textures'
    source_name = 'Metal009_4K-PNG_Roughness.png'
    with zipfile.ZipFile(archive) as source:
        (cache / source_name).write_bytes(source.read(source_name))
    roughness = Image.open(cache / source_name).convert('L')
    if roughness.size != (4096, 4096):
        raise RuntimeError('expected 4K source maps')
    # Store effective linear roughness directly; the MJCF multiplier is one.
    roughness = roughness.filter(ImageFilter.GaussianBlur(.7))
    outputs = {}
    for name, center, amplitude in (
        ('brushed_metal_roughness.png', 89, 8),
        ('graphite_metal_roughness.png', 97, 6),
    ):
        image = roughness.point([round(center + amplitude * max(-1, min(1, (v - 120) / 50)))
                                 for v in range(256)])
        image.save(destination / name, optimize=True)
        low, high = image.getextrema()
        outputs[name] = {'sha256': hashlib.sha256((destination / name).read_bytes()).hexdigest(),
                         'roughness_min': low / 255, 'roughness_max': high / 255,
                         'roughness_mean': ImageStat.Stat(image).mean[0] / 255}
    metadata = {'asset': 'ambientCG Metal009', 'author': 'Lennart Demes',
                'source': 'https://ambientcg.com/view?id=Metal009', 'download': URL,
                'license': 'CC0-1.0', 'license_url': 'https://docs.ambientcg.com/license/',
                'archive_sha256': SHA256, 'resolution': [4096, 4096],
                'processing': 'roughness only; Gaussian blur 0.7px; low-contrast linear remapping; constant base color',
                'roughness_factor': 1.0, 'texture_tile_m': .20,
                'outputs': outputs}
    (destination / 'brushed_metal.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('Prepared calibrated 4K CC0 silver/graphite roughness maps')


if __name__ == '__main__':
    main()
