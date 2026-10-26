"""Observe real vcan traffic, including foreign IDs, without changing the host bus."""

import json
import os
from pathlib import Path
import re
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

hal = Path(sys.argv[1]).resolve()
root = Path(__file__).resolve().parents[1]
pattern = re.compile(
    r'BUS LOAD est\. \[([#.]{30})\] ([\d.]+)% \| reference (\d+) bit/s([^\n]*)\n'
    r'Classic CAN / max stuffing \| window ([\d.]+) ms \| frames (\d+) \| bits (\d+)'
    r' \| monitor drops=(\d+) \| error reports=(\d+)'
)


def records(log):
    return list(pattern.finditer(log.read_text()))


def wait_for(predicate, process, log):
    deadline = time.monotonic() + 5

    while time.monotonic() < deadline:
        assert process.poll() is None, log.read_text()
        value = predicate()

        if value:
            return value

        time.sleep(0.01)

    raise AssertionError(log.read_text())


with tempfile.TemporaryDirectory(prefix='qhrr05-bus-load-') as directory:
    directory = Path(directory)

    for bitrate in (1000000, 10000):
        name = f'/qhrr05-load-{os.getpid()}-{bitrate}'
        config = directory / f'{bitrate}.json'
        log = directory / f'{bitrate}.log'
        config.write_text(
            json.dumps(
                {
                    'interface': 'vcan0',
                    'frequency_hz': 10,
                    'statistics_hz': 20,
                    'bus_load_bitrate': bitrate,
                    'motor_ids': [1, 2, 3],
                    'motor_config': str(root / 'config/motor_config/ak10_9_v3.yaml'),
                }
            )
        )

        manifest = directory / f'{bitrate}-processes.json'
        manifest.write_text(
            json.dumps(
                {
                    'shm_name': name,
                    'monitoring': {
                        'display_interval_sec': 2.0,
                        'window_sec': 10.0,
                        'sample_hz': 100.0,
                    },
                    'processes': [
                        {
                            'name': 'can_hal',
                            'exec_path': str(hal),
                            'args': [str(config), '{shm_name}'],
                            'wait_ready': True,
                        }
                    ],
                }
            )
        )

        with log.open('w') as output:
            process = subprocess.Popen(
                [str(hal.parent / 'daemon'), '--config', str(manifest), '--no-terminals'],
                stdout=output,
                stderr=output,
            )

        try:
            wait_for(lambda: len(records(log)) >= 3, process, log)
            before = len(records(log))
            baseline = records(log)[:before]
            assert any(float(row[2]) == 0 for row in baseline), log.read_text()
            assert sum(int(row[6]) for row in baseline) == 6, 'Local TX was lost or counted twice'

            # IDs outside the HAL's three motor filters must still contribute.
            with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sender:
                sender.bind(('vcan0',))

                for identifier in (
                    0x321,
                    socket.CAN_EFF_FLAG | 0x12345,
                    socket.CAN_RTR_FLAG | 0x322,
                ):
                    frame = struct.pack('=IB3x8s', identifier, 8, bytes(8))

                    for _ in range(40):
                        assert sender.send(frame) == len(frame)

            def collected():
                rows = records(log)[before:]
                return rows if sum(int(row[6]) for row in rows) >= 120 else None

            observed = wait_for(collected, process, log)
            total_frames = sum(int(row[6]) for row in observed)
            total_bits = sum(int(row[7]) for row in observed)
            management_frames = total_frames - 120
            assert management_frames >= 0 and management_frames % 3 == 0, log.read_text()
            assert total_bits == 40 * (135 + 160 + 55) + management_frames * 135, log.read_text()

            for row in observed:
                bar, percent, reference, suffix, window_ms, frames, bits, drops, errors = (
                    row.groups()
                )
                expected = 100 * int(bits) / (int(reference) * float(window_ms) / 1000)
                assert abs(float(percent) - expected) < 0.2, row[0]
                filled = round(min(100, float(percent)) * 30 / 100)
                assert abs(bar.count('#') - filled) <= 1, row[0]
                assert int(reference) == bitrate and int(drops) == 0 and int(errors) == 0, row[0]

            if bitrate == 10000:
                assert any(float(row[2]) > 100 and row[1] == '#' * 30 for row in observed)
                assert any('OVER CAPACITY' in row[4] for row in observed)

            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=5) == 0, log.read_text()
            assert not (Path('/dev/shm') / name[1:]).exists()
            print(
                'PASS:',
                bitrate,
                'bit/s reference, all-ID load, no duplicate TX, EFF/RTR, bar/percent',
            )
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
