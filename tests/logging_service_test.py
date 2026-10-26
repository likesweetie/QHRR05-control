"""Real service acceptance: CLI, 5/2s history, rotation, contention, failure and restart."""

from contextlib import contextmanager
from pathlib import Path
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from log_data import inspect, snapshots
from log_analyze import metrics, state_dataframe


def wait_for(check, message, timeout=12):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = check()
        if value:
            return value
        time.sleep(0.025)
    raise AssertionError(message())


def rpc(path, command, ok=True):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(5)
        client.connect(str(path))
        client.sendall(command.encode())
        response = json.loads(client.recv(65536))
    if ok:
        assert response['ok'], response
    return response


def settings(folder):
    config = yaml.safe_load((ROOT / 'config/shm_logger/shm_logger.yaml').read_text())
    logging = config['logging']
    logging['output_dir'] = str(folder / 'logs')
    logging['storage']['minimum_free_gib'] = 0
    logging['continuous']['rotation_seconds'] = 0.5
    logging['flight_recorder']['capacity_mib'] = 16
    logging['sources'] = {
        'files': {
            'model': str(ROOT / 'models/qhrr-05/scene.xml'),
            'control': str(ROOT / 'tests/fixtures/control.yaml'),
        }
    }
    path = folder / 'logger.yaml'
    path.write_text(yaml.safe_dump(config))
    return path


@contextmanager
def running(folder, suffix, config=None, duration=30, start_producer=True):
    folder.mkdir(parents=True, exist_ok=True)
    path = config or settings(folder)
    name = f'/qhrr05-logging-{os.getpid()}-{suffix}'
    sock = Path(f'/tmp/qhrr05-log-{os.getuid()}-{name[1:]}.sock')
    control = folder / 'producer-control'
    control.write_text('')
    with (folder / 'logger.log').open('w') as out:
        logger = subprocess.Popen(
            [str(ROOT / 'build/shm_logger'), '--config', str(path), '--shm-name', name],
            stdout=out,
            stderr=subprocess.STDOUT,
        )
    producer = None
    try:
        wait_for(sock.exists, lambda: (folder / 'logger.log').read_text())
        assert rpc(sock, 'status')['state'] == 'INITIALIZING'
        if start_producer:
            producer = subprocess.Popen(
                [str(ROOT / 'build/logging_test_producer'), name, str(duration), str(control)],
                stdout=subprocess.DEVNULL,
            )
            wait_for(
                lambda: rpc(sock, 'status')['state'] in ('RECORDING', 'FAULT'),
                lambda: (folder / 'logger.log').read_text(),
            )
        yield logger, producer, sock, control
    finally:
        if logger.poll() is None:
            logger.send_signal(signal.SIGTERM)
            logger.wait(timeout=8)
        if producer and producer.poll() is None:
            producer.send_signal(signal.SIGTERM)
            producer.wait(timeout=4)


with tempfile.TemporaryDirectory(prefix='qhrr05-logging-service-') as tmp:
    root = Path(tmp)
    folder = root / 'acceptance'
    with running(folder, 'acceptance') as (logger, producer, sock, control):
        wait_for(
            lambda: rpc(sock, 'status')['snapshot_successes'] > 2600,
            lambda: (folder / 'logger.log').read_text(),
        )
        session = Path(rpc(sock, 'status')['session_directory'])
        cli = subprocess.run(
            [str(ROOT / 'build/qhrr-logctl'), '--socket', str(sock), 'mark', 'contact-test'],
            capture_output=True,
            text=True,
            timeout=5,
        )
        assert cli.returncode == 0 and json.loads(cli.stdout)['ok'], cli.stderr
        assert rpc(sock, 'trigger')['ok']
        assert not rpc(sock, 'trigger', ok=False)['ok']
        wait_for(
            lambda: len(list((session / 'faults').glob('*.mcap.json'))) >= 1,
            lambda: (folder / 'logger.log').read_text(),
        )
        dump = json.loads(next((session / 'faults').glob('*.mcap.json')).read_text())
        assert dump['ok'] and not dump['truncated'], dump
        assert dump['trigger_ns'] - dump['first_message_ns'] >= 4.97e9, dump
        assert dump['last_message_ns'] - dump['trigger_ns'] >= 1.97e9, dump
        assert rpc(sock, 'health')['triggers_rejected'] >= 1
        assert rpc(sock, 'sessions')['sessions']
        control.write_text('hold')
        wait_for(
            lambda: rpc(sock, 'status')['shm_lock_misses'] > 10,
            lambda: (folder / 'logger.log').read_text(),
        )
        control.write_text('pause')
        wait_for(
            lambda: rpc(sock, 'health')['triggers_accepted'] >= 2,
            lambda: (folder / 'logger.log').read_text(),
        )
        control.write_text('')
        before = rpc(sock, 'status')['snapshot_successes']
        wait_for(
            lambda: rpc(sock, 'status')['snapshot_successes'] > before + 10,
            lambda: (folder / 'logger.log').read_text(),
        )
        assert producer.poll() is None
        assert rpc(sock, 'stop')['state'] == 'READY'
        report = inspect(session)
        assert (
            report['config_hashes_valid'] and report['channels']['qhrr/state']['samples'] > 2600
        ), report
        assert report['capture_timestamp_mismatches'] == 0 and report['timestamp_regressions'] == 0
        assert len(report['files']) > 5
        checked = subprocess.run(
            [str(ROOT / 'build/qhrr-log-inspect'), str(session), '--fields'],
            capture_output=True,
            text=True,
            timeout=15,
        )
        assert checked.returncode == 0, checked.stderr
        analyzed = subprocess.run(
            [
                str(ROOT / 'build/qhrr-log-analyze'),
                str(session),
                '--signals',
                'motor/HR/position_rad',
                '--align-hz',
                '200',
                '--csv',
                str(folder / 'aligned.csv'),
                '--plot',
                str(folder / 'plot.png'),
                '--report',
                str(folder / 'analysis.json'),
            ],
            capture_output=True,
            text=True,
            timeout=25,
        )
        assert analyzed.returncode == 0, analyzed.stderr
        assert (folder / 'plot.png').stat().st_size > 1000
        data = state_dataframe(session)
        measured = metrics(data)
        assert measured['observer_force_rmse_n'] > 0 and measured['HR_motor_target_rmse_rad'] > 0.09
        codes = [
            values['event/code']
            for _, channel, values in snapshots(session)
            if channel == 'qhrr/events'
        ]
        assert {'USER_MARK', 'MANUAL_TRIGGER', 'SHM_STALE'} <= set(codes), codes
        assert rpc(sock, 'start')['state'] == 'RECORDING'
        assert Path(rpc(sock, 'status')['session_directory']) != session
        assert (session / 'session.json').exists()
        assert rpc(sock, 'stop')['state'] == 'READY'
        print(
            'PASS: CLI, 5s/2s dump, duplicates, rotation, contention/stale, metadata, offline decode and start/stop'
        )

    folder = root / 'disk-full'
    folder.mkdir()
    config = settings(folder)
    node = yaml.safe_load(config.read_text())
    node['logging']['storage']['minimum_free_gib'] = 1048576
    config.write_text(yaml.safe_dump(node))
    with running(folder, 'diskfull', config=config) as (logger, producer, sock, _):
        status = wait_for(
            lambda: (s if (s := rpc(sock, 'status'))['state'] == 'FAULT' else None),
            lambda: (folder / 'logger.log').read_text(),
        )
        assert 'storage budget' in status['last_error'], status
        assert producer.poll() is None and logger.poll() is None
        assert not rpc(sock, 'start', ok=False)['ok']
        print(
            'PASS: startup disk budget failure observable; producer and control interface survive'
        )

    folder = root / 'write-failure'
    with running(folder, 'writefailure') as (logger, producer, sock, _):
        status = rpc(sock, 'status')
        session = Path(status['session_directory'])
        os.chmod(session, 0o500)
        try:
            failed = wait_for(
                lambda: (
                    s if (s := rpc(sock, 'status'))['state'] in ('DEGRADED', 'FAULT') else None
                ),
                lambda: (folder / 'logger.log').read_text(),
            )
            assert failed['last_error'] and producer.poll() is None
        finally:
            os.chmod(session, 0o700)
        rpc(sock, 'stop')
        assert list(session.glob('recording*.mcap'))
        print('PASS: rotation permission failure stops continuous output without killing producer')

    folder = root / 'hardware'
    folder.mkdir()
    config = settings(folder)
    node = yaml.safe_load(config.read_text())
    node['logging']['runtime_mode'] = 'hardware'
    config.write_text(yaml.safe_dump(node))
    with running(folder, 'hardware', config=config) as (logger, producer, sock, _):
        wait_for(
            lambda: rpc(sock, 'status')['snapshot_successes'] > 50,
            lambda: (folder / 'logger.log').read_text(),
        )
        session = rpc(sock, 'status')['session_directory']
        rpc(sock, 'stop')
        fields = inspect(session)['fields']['qhrr/state']
        assert not any('simulation/ground_truth' in key for key in fields)
        print('PASS: hardware schema omits Ground Truth rather than logging fake zeros')
