#!/usr/bin/env python3
"""Portable feasibility comparison: baseline CSV observer, continuous MCAP, then continuous+flight.

Timing is observed producer publication timing, not a trace of every controller cycle.
The same CSV observer runs in all cases so the reported delta isolates the new recorder.
"""

import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import sys
import yaml
from log_data import ROOT, inspect


def observed_timing(csv_path, nominal_hz):
    import numpy as np

    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    elapsed, gaps, previous = [], 0, None
    for row in rows:
        seq, stamp = int(row['command_seq']), float(row['control_time'])
        if previous and seq != previous[0]:
            if seq == previous[0] + 1:
                elapsed.append((stamp - previous[1]) * 1e6)
            elif seq > previous[0]:
                gaps += seq - previous[0] - 1
        if not previous or seq != previous[0]:
            previous = seq, stamp
    intervals = np.array(elapsed)
    result = {
        'csv_samples': len(rows),
        'observed_single_step_intervals': len(intervals),
        'unobserved_control_publications': gaps,
        'timing_scope': 'Adjacent observed sequences only; producer mutex timing unavailable',
    }
    if len(intervals):
        jitter = np.abs(intervals - 1e6 / nominal_hz)
        result.update(
            {
                'observed_jitter_p99_us': float(np.percentile(jitter, 99)),
                'observed_jitter_p999_us': float(np.percentile(jitter, 99.9)),
                'observed_period_max_us': float(intervals.max()),
            }
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--rig', choices=['synthetic', 'simulation', 'can'], default='synthetic')
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--cases',
        nargs='+',
        choices=['baseline', 'continuous', 'continuous_flight'],
        default=['baseline', 'continuous', 'continuous_flight'],
    )
    parser.add_argument(
        '--inject-contention',
        action='store_true',
        help='Synthetic producer holds its mutex for 20ms during the middle 10%%',
    )
    args = parser.parse_args()
    if args.duration < 1:
        raise ValueError('duration must be at least one second')
    if args.inject_contention and args.rig != 'synthetic':
        raise ValueError('contention injection is available only with the synthetic producer')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    results = {}
    for name, continuous, flight in [
        ('baseline', False, False),
        ('continuous', True, False),
        ('continuous_flight', True, True),
    ]:
        if name not in args.cases:
            continue
        folder = output / name
        folder.mkdir()
        control_file = folder / 'producer-control'
        control_file.write_text('')
        logger_config = yaml.safe_load((ROOT / 'config/shm_logger/shm_logger.yaml').read_text())
        logging = logger_config['logging']
        logging['output_dir'] = str(folder / 'records')
        logging['storage']['minimum_free_gib'] = 0
        logging['continuous']['enabled'] = continuous
        logging['flight_recorder']['enabled'] = flight
        logger_path = folder / 'shm_logger.yaml'
        logger_path.write_text(yaml.safe_dump(logger_config))
        shm_name = f'/qhrr05-benchmark-{os.getpid()}-{name}'
        config = yaml.safe_load((ROOT / 'config/daemon/daemon.yaml').read_text())
        config['shm_name'] = shm_name
        processes = config['processes' if args.rig == 'can' else 'debug_processes']
        if args.rig == 'synthetic':
            processes = [
                {
                    'name': 'producer',
                    'exec_path': str(ROOT / 'build/logging_test_producer'),
                    'args': ['--attach', '{shm_name}', str(args.duration + 60), str(control_file)],
                }
            ]
        else:
            for process in processes:
                process['working_directory'] = str(ROOT)
                if process['name'] == 'simulation':
                    process['args'] += ['--headless', '--duration', '1000000000']
                if process['name'] == 'shm_logger':
                    process['args'][process['args'].index('--config') + 1] = str(logger_path)
        processes = [p for p in processes if p['name'] != 'shm_logger']
        if continuous or flight:
            processes.append(
                {
                    'name': 'shm_logger',
                    'exec_path': str(ROOT / 'build/shm_logger'),
                    'args': [
                        '--config',
                        str(logger_path),
                        '--shm-name',
                        '{shm_name}',
                        '--daemon-config',
                        '{daemon_config}',
                    ],
                    'critical': False,
                }
            )
        csv_path = folder / 'timing.csv'
        processes.append(
            {
                'name': 'csv_observer',
                'exec_path': str(ROOT / 'build/shm_logger'),
                'args': ['--csv', '{shm_name}', str(csv_path)],
            }
        )
        config['debug_processes'] = config['processes'] = processes
        manifest = folder / 'daemon.yaml'
        manifest.write_text(yaml.safe_dump(config))
        command = [str(ROOT / 'build/daemon'), '--config', str(manifest), '--no-terminals']
        if args.rig == 'can':
            command = [
                'unshare',
                '--user',
                '--map-root-user',
                '--net',
                '--',
                'sh',
                '-ec',
                'ip link add dev vcan0 type vcan; ip link set dev vcan0 up; exec "$@"',
                'sh',
                *command,
            ]
        else:
            command.append('--without-can')
        print(f'[benchmark] {name}: {args.duration:g}s, rig={args.rig}', flush=True)
        with (folder / 'runtime.log').open('w') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        started = time.monotonic()
        try:
            while time.monotonic() - started < args.duration:
                if process.poll() is not None:
                    raise RuntimeError((folder / 'runtime.log').read_text())
                if args.inject_contention:
                    phase = (time.monotonic() - started) / args.duration
                    control_file.write_text('hold' if 0.45 < phase < 0.55 else '')
                time.sleep(0.1)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)
        if process.returncode:
            raise RuntimeError((folder / 'runtime.log').read_text())
        control_hz = (
            500.0
            if args.rig == 'synthetic'
            else yaml.safe_load((ROOT / 'config/control/control.yaml').read_text())['control'][
                'frequency_hz'
            ]
        )
        result = observed_timing(csv_path, control_hz)
        sessions = list((folder / 'records').glob('*/session.json'))
        if sessions:
            session = sessions[0].parent
            recording = inspect(session)
            result['mcap_channels'] = recording['channels']
            result['logger_quality'] = json.loads((session / 'quality_report.json').read_text())
        results[name] = result
    report = {
        'rig': args.rig,
        'duration_per_case_seconds': args.duration,
        'results': results,
        'not_measured': [
            'true per-cycle deadline misses',
            'producer mutex wait/hold',
            'PlotJuggler rendering cost',
            'physical storage-stall latency bound',
        ],
    }
    (output / 'benchmark_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(f'[qhrr-log-benchmark] ERROR: {error}', file=sys.stderr)
        sys.exit(2)
