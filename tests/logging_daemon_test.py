"""Optional logger crash/restart and startup failure must not stop the daemon-owned producer."""

from pathlib import Path
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from log_data import inspect


def wait_for(check, message, timeout=12):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = check()
        if value:
            return value
        time.sleep(0.025)
    raise AssertionError(message())


with tempfile.TemporaryDirectory(prefix='qhrr05-log-daemon-') as tmp:
    folder = Path(tmp)
    config = yaml.safe_load((ROOT / 'config/shm_logger/shm_logger.yaml').read_text())
    config['logging']['output_dir'] = str(folder / 'records')
    config['logging']['storage']['minimum_free_gib'] = 0
    config['logging']['flight_recorder']['capacity_mib'] = 1
    logger_config = folder / 'logger.yaml'
    logger_config.write_text(yaml.safe_dump(config))
    for kind in ('crash', 'startup-failure'):
        shm_name = f'/qhrr05-log-daemon-{os.getpid()}-{kind}'
        producer = {
            'name': 'producer',
            'exec_path': str(ROOT / 'build/logging_test_producer'),
            'args': ['--attach', '{shm_name}', '1000'],
        }
        logger = {
            'name': 'shm_logger',
            'critical': False,
            'restart_limit': 1,
            'restart_delay_ms': 100,
            'exec_path': str(ROOT / 'build/shm_logger'),
            'args': [
                '--config',
                str(logger_config),
                '--shm-name',
                '{shm_name}',
                '--daemon-config',
                '{daemon_config}',
            ],
        }
        if kind == 'startup-failure':
            logger['exec_path'] = str(folder / 'missing-logger')
        path = folder / f'{kind}.json'
        path.write_text(
            json.dumps(
                {
                    'runtime_mode': 'simulation', 'shm_name': shm_name,
                    'monitoring': {'display_interval_sec': 2, 'window_sec': 10, 'sample_hz': 100},
                    'debug_processes': [producer, logger],
                }
            )
        )
        log = folder / f'{kind}.log'
        with log.open('w') as out:
            process = subprocess.Popen(
                [
                    str(ROOT / 'build/daemon'),
                    '--config',
                    str(path),
                    '--without-can',
                    '--no-terminals',
                ],
                stdout=out,
                stderr=subprocess.STDOUT,
            )
        try:
            child = int(
                wait_for(
                    lambda: (
                        m[1]
                        if (m := re.search(r'producer started pid=(\d+)', log.read_text()))
                        else None
                    ),
                    log.read_text,
                )
            )
            if kind == 'crash':
                old = int(
                    wait_for(
                        lambda: (
                            m[1]
                            if (m := re.search(r'shm_logger started pid=(\d+)', log.read_text()))
                            else None
                        ),
                        log.read_text,
                    )
                )
                wait_for(
                    lambda: len(list((folder / 'records').glob('*/recording.mcap'))) >= 1,
                    log.read_text,
                )
                time.sleep(0.15)
                os.kill(old, signal.SIGKILL)
                wait_for(
                    lambda: len(re.findall(r'shm_logger started pid=(\d+)', log.read_text())) == 2,
                    log.read_text,
                )
                wait_for(
                    lambda: len(list((folder / 'records').glob('*/recording.mcap'))) >= 2,
                    log.read_text,
                )
                print(
                    'PASS: killed logger restarts into a new session; control producer stays alive'
                )
            else:
                wait_for(lambda: 'optional restart limit reached' in log.read_text(), log.read_text)
                print('PASS: missing optional logger is explicit and does not abort runtime')
            assert process.poll() is None
            os.kill(child, 0)
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=8) == 0, log.read_text()
            assert not (Path('/dev/shm') / shm_name[1:]).exists()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=8)
    sessions = sorted((folder / 'records').glob('*/session.json'))
    assert len(sessions) == 2
    complete = [p.parent for p in sessions if json.loads(p.read_text())['result'] != 'recording']
    assert len(complete) == 1
    assert inspect(complete[0])['channels']['qhrr/state']['samples'] > 0
