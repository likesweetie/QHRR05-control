#!/usr/bin/env python3
"""Reproduce seamless microcement and bead-blasted aluminium PBR maps, no dependencies."""
import math
from pathlib import Path
import random
import struct
import zlib

SIZE = 512
OUTPUT = Path(__file__).resolve().parents[1] / 'models/qhrr-05/visualization/textures'


def write_png(path, channels, rows):
    def chunk(kind, data):
        return (
            struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data))
        )

    header = struct.pack('>IIBBBBB', SIZE, SIZE, 8, 2 if channels == 3 else 0, 0, 0, 0)
    path.write_bytes(
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', header)
        + chunk(b'IDAT', zlib.compress(b''.join(b'\0' + bytes(row) for row in rows), 9))
        + chunk(b'IEND', b'')
    )


def noise(scale, seed):
    rng = random.Random(seed)
    lattice = [[rng.uniform(-1, 1) for _ in range(scale)] for _ in range(scale)]
    result = []
    for y in range(SIZE):
        gy = y * scale / SIZE
        iy = int(gy)
        fy = gy - iy
        fy = fy * fy * (3 - 2 * fy)
        row = []
        for x in range(SIZE):
            gx = x * scale / SIZE
            ix = int(gx)
            fx = gx - ix
            fx = fx * fx * (3 - 2 * fx)
            a = lattice[iy % scale][ix % scale]
            b = lattice[iy % scale][(ix + 1) % scale]
            c = lattice[(iy + 1) % scale][ix % scale]
            d = lattice[(iy + 1) % scale][(ix + 1) % scale]
            row.append((a + (b - a) * fx) * (1 - fy) + (c + (d - c) * fx) * fy)
        result.append(row)
    return result


def byte(value):
    return max(0, min(255, round(value)))


def normal_map(height, strength):
    rows = []
    for y in range(SIZE):
        row = []
        for x in range(SIZE):
            dx = (height[y][(x + 1) % SIZE] - height[y][(x - 1) % SIZE]) * strength
            dy = (height[(y + 1) % SIZE][x] - height[(y - 1) % SIZE][x]) * strength
            length = math.sqrt(1 + dx * dx + dy * dy)
            row.extend(byte(127.5 * (value / length + 1)) for value in (-dx, -dy, 1))
        rows.append(row)
    return rows


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    cloud = noise(6, 4703)
    aggregate = noise(64, 9061)
    fine = noise(256, 8210)
    floor_height = [
        [0.45 * aggregate[y][x] + 0.25 * fine[y][x] for x in range(SIZE)] for y in range(SIZE)
    ]
    anodize_height = noise(256, 15342)
    aluminum_grain = noise(256, 43012)
    images = {
        name: []
        for name in (
            'floor_color',
            'floor_roughness',
            'floor_ao',
            'anodize_color',
            'anodize_roughness',
            'aluminum_roughness',
        )
    }
    for y in range(SIZE):
        rows = {name: [] for name in images}
        for x in range(SIZE):
            # Mid-gray microcement keeps the lit floor below white; no grid or painted AO.
            variation = 0.7 * cloud[y][x] + 0.4 * aggregate[y][x] + 0.3 * fine[y][x]
            rows['floor_color'].extend(byte(base + variation) for base in (135, 136, 137))
            rows['floor_roughness'].append(byte(232 + 3 * cloud[y][x] + 3 * fine[y][x]))
            rows['floor_ao'].append(byte(253 + min(0, aggregate[y][x]) * 3))
            # Near-neutral albedo preserves the MJCF tint; roughness carries most detail.
            bead = anodize_height[y][x]
            rows['anodize_color'].extend([252] * 3)
            # Non-periodic fine grain; no sinusoidal brush bands or height displacement.
            rows['aluminum_roughness'].append(byte(247 + 3 * aluminum_grain[y][x]))
            rows['anodize_roughness'].append(byte(239 + 3 * bead + 1.5 * math.sin(2 * math.pi * y / 8)))
        for name in images:
            images[name].append(rows[name])
    for name, rows in images.items():
        write_png(OUTPUT / f'{name}.png', 3 if name.endswith('color') else 1, rows)
    write_png(OUTPUT / 'floor_normal.png', 3, normal_map(floor_height, 0.018))
    write_png(OUTPUT / 'anodize_normal.png', 3, normal_map(anodize_height, 0.018))


if __name__ == '__main__':
    main()
