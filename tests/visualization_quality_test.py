"""Opt-in actual GPU quality/capture checks; no daemon, SHM or robot connection."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / 'build-filament/native/bin/qhrr_visualizer'


def execute(args, expected=0):
    result = subprocess.run(args, cwd=ROOT, text=True, capture_output=True, timeout=90)
    assert result.returncode == expected, result.stdout + result.stderr
    return result.stdout + result.stderr


# Reject explicit zero, non-finite, out-of-range and unsupported quality instead of defaulting.
for args in (('--msaa', '0'), ('--msaa', '3'), ('--render-scale', '0'),
             ('--render-scale', 'nan'), ('--render-scale', '2.1'),
             ('--backend', 'classic', '--render-scale', '2')):
    output = execute([str(BINARY), '--model', 'models/qhrr-05/scene_pbr.xml', *args], expected=1)
    assert '[visualizer] ERROR:' in output and 'FEngine' not in output

with tempfile.TemporaryDirectory(prefix='qhrr-quality-') as directory:
    images = []
    for samples, scale in ((1, 1), (4, 1), (8, 2)):
        path = Path(directory) / f'{samples}.ppm'
        output = execute([str(ROOT / 'scripts/run_visualization.sh'), '--preview', '--hidden',
                          '--width', '801', '--height', '501', '--frames', '8',
                          '--msaa', str(samples), '--render-scale', str(scale),
                          '--screenshot', str(path)])
        assert f'verified color/depth MSAA={samples}x' in output
        assert f'MSAA={samples}x fixed render scale={scale}x' in output
        assert 'window-equivalent MSAA and fixed supersampling' in output
        assert f'internal~={801 * scale}x{501 * scale}' in output
        assert output.count('uploaded model resources once') == 1
        assert 'material=QHRR brushed PBR, anisotropy + specular AA' in output
        with path.open('rb') as stream:
            assert stream.readline() == b'P6\n'
            assert stream.readline() == b'801 501\n'
            assert stream.readline() == b'255\n'
            image = stream.read()
        assert len(image) == 801 * 501 * 3  # Odd RGB stride exercises readback alignment.
        assert max(image) - min(image) > 100
        images.append(image)
        print(f'PASS: MSAA {samples}x, scale {scale}x, odd-stride full-quality screenshot')
    changed = sum(a != b for a, b in zip(images[0], images[-1]))
    assert changed > 1000, 'AA/supersampling settings did not affect the rendered image'
    print(f'PASS: quality affects {changed} channels; explicit invalid settings rejected')
