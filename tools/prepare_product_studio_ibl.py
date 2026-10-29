#!/usr/bin/env python3
"""Build a neutral, low-frequency product-studio IBL offline using pinned Filament cmgen."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 1024, 512


def unit(vector):
    length = math.sqrt(sum(v * v for v in vector))
    return tuple(v / length for v in vector)


# Soft rectangular sources with dark negative fill give conductors readable highlights.
# The three sources are broad, non-periodic cards rather than a striped room HDRI.
CARDS = (
    (unit((-0.6, 0.65, 0.45)), 4.0, (0.36, 0.70), (1.0, 0.99, 0.98)),
    (unit((0.7, 0.45, -0.55)), 1.2, (0.52, 0.38), (0.97, 0.99, 1.0)),
    (unit((0.1, 0.95, -0.2)), 3.0, (0.75, 0.22), (1.0, 1.0, 1.0)),
)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def feather(value, width):
    t = max(0., min(1., (width + .07 - abs(value)) / .14))
    return t*t*(3-2*t)


def radiance(direction):
    ambient = 0.10 + 0.04 * (direction[1] + 1) / 2
    rgb = [ambient] * 3
    for center, strength, size, color in CARDS:
        facing = sum(a*b for a,b in zip(direction, center))
        if facing <= 0:
            continue
        horizontal = unit(cross((0, 1, 0), center))
        vertical = cross(center, horizontal)
        u = math.atan2(sum(a*b for a,b in zip(direction,horizontal)), facing)
        v = math.atan2(sum(a*b for a,b in zip(direction,vertical)), facing)
        power = strength * feather(u,size[0]) * feather(v,size[1])
        for i in range(3):
            rgb[i] += power * color[i]
    return rgb


def rgbe(rgb):
    largest = max(rgb)
    mantissa, exponent = math.frexp(largest)
    scale = mantissa * 256 / largest
    return tuple(min(255, int(value * scale)) for value in rgb) + (exponent + 128,)


def write_hdr(path):
    with path.open('wb') as output:
        output.write(f'#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y {HEIGHT} +X {WIDTH}\n'.encode())
        for y in range(HEIGHT):
            theta = math.pi * (y + .5) / HEIGHT
            sin_theta, cos_theta = math.sin(theta), math.cos(theta)
            row = []
            for x in range(WIDTH):
                phi = 2 * math.pi * (x + .5) / WIDTH
                direction = (sin_theta * math.cos(phi), cos_theta, sin_theta * math.sin(phi))
                row.append(rgbe(radiance(direction)))
            output.write(bytes((2, 2, WIDTH >> 8, WIDTH & 255)))
            for channel in range(4):
                for begin in range(0, WIDTH, 128):
                    values = bytes(pixel[channel] for pixel in row[begin:begin + 128])
                    output.write(bytes((len(values),)) + values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cmgen', type=Path, default=ROOT / 'build-filament/native/bin/cmgen')
    args = parser.parse_args()
    if not args.cmgen.is_file():
        parser.error('cmgen is missing; build the pinned visualization tools first')
    cache = ROOT / 'build-filament/product-studio-source'
    cache.mkdir(parents=True, exist_ok=True)
    source = cache / 'product_studio.hdr'
    write_hdr(source)
    processed = cache / 'processed'
    subprocess.run([str(args.cmgen), '--format=ktx', '--size=512',
                    f'--deploy={processed}', str(source)], check=True)
    payload = (processed / 'processed_ibl.ktx').read_bytes()
    destination = ROOT / 'models/qhrr-05/visualization/environment/product_studio_ibl.ktx'
    temporary = destination.with_suffix('.tmp')
    temporary.write_bytes(payload)
    temporary.replace(destination)
    metadata = {'generator': 'tools/prepare_product_studio_ibl.py',
                'source': 'three feathered rectangular softboxes with negative fill; generated locally',
                'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'ibl_sha256': hashlib.sha256(payload).hexdigest(),
                'cube_size': 512, 'source_size': [WIDTH, HEIGHT]}
    destination.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Prepared {destination} ({len(payload)} bytes)')


if __name__ == '__main__':
    main()
