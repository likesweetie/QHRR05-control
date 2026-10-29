#!/usr/bin/env python3
"""Prepare 4K brushed-metal maps from the pinned CC0 ambientCG Metal009 archive."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile
from PIL import Image

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
    names = ['Metal009_4K-PNG_Color.png', 'Metal009_4K-PNG_Roughness.png']
    with zipfile.ZipFile(archive) as source:
        for name in names:
            (cache / name).write_bytes(source.read(name))
    color = Image.open(cache / names[0]).convert('L')
    roughness = Image.open(cache / names[1]).convert('L')
    if color.size != (4096, 4096) or roughness.size != color.size:
        raise RuntimeError('expected 4K source maps')
    # Preserve the source's fine brushing without painting dark steel albedo or broad roughness bands.
    color = color.point([round(max(0, min(255, 236 + .20 * (v - 149)))) for v in range(256)]).convert('RGB')
    roughness = roughness.point([round(209 + 46 * max(0, min(1, (v - 80) / 100))) for v in range(256)])
    color.save(destination / 'brushed_metal_color.png', optimize=True)
    roughness.save(destination / 'brushed_metal_roughness.png', optimize=True)
    metadata = {'asset': 'ambientCG Metal009', 'author': 'Lennart Demes',
                'source': 'https://ambientcg.com/view?id=Metal009', 'download': URL,
                'license': 'CC0-1.0', 'license_url': 'https://docs.ambientcg.com/license/',
                'archive_sha256': SHA256, 'resolution': [4096, 4096],
                'processing': 'neutral albedo; bounded roughness; original brush direction preserved',
                'texture_tile_m': .08,
                'output_sha256': {name: hashlib.sha256((destination / name).read_bytes()).hexdigest()
                                  for name in ['brushed_metal_color.png', 'brushed_metal_roughness.png']}}
    (destination / 'brushed_metal.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('Prepared 4K CC0 brushed-metal color and roughness maps')


if __name__ == '__main__':
    main()
