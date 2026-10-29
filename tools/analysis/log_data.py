"""Read pinned DataTamer MCAPs. No SHM attachment, CAN access, or runtime replay."""

from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/data_tamer/python'))
import data_tamer_parser as dt


def recording_files(path, faults=False):
    path = Path(path)
    if path.is_file():
        if path.suffix != '.mcap':
            raise ValueError(f'Expected MCAP: {path}')
        return [path]
    pattern = 'faults/*.mcap' if faults else 'recording*.mcap'
    files = list(path.glob(pattern))
    if not files:
        raise ValueError(f'No {pattern} in {path}')
    # Rotation numbers are numerical; filesystem/glob order is not chronology.
    return sorted(
        files,
        key=lambda p: int(p.stem.rsplit('_', 1)[1]) if p.stem.rsplit('_', 1)[-1].isdigit() else 0,
    )


def snapshots(path, faults=False):
    for file in recording_files(path, faults):
        yield from dt.iter_mcap(str(file))


def inspect(path, faults=False):
    files = recording_files(path, faults)
    channels, fields, previous_time, previous_sequences = {}, {}, {}, {}
    sequence_fields = (
        'shm/motor_sequence',
        'shm/model_sequence',
        'control/command_sequence',
        'shm/simulation_sequence',  # Legacy recordings before simulation IPC separation.
        'simulation/ipc_sequence',
        'motor/command_sequence',
        'motor/feedback_sequence',
    )
    sequence_gaps, sequence_regressions, time_regressions, capture_regressions = {}, {}, 0, 0
    for file in files:
        for stamp, channel, values in dt.iter_mcap(str(file)):
            stats = channels.setdefault(
                channel, {'samples': 0, 'first_ns': stamp, 'last_ns': stamp}
            )
            stats['samples'] += 1
            stats['last_ns'] = stamp
            fields.setdefault(channel, set()).update(values)
            if stamp < previous_time.get(channel, stamp):
                time_regressions += 1
            previous_time[channel] = stamp
            capture = values.get('snapshot/capture_ns')
            if capture is not None and capture != stamp:
                capture_regressions += 1
            if channel != 'qhrr/state':
                continue
            for key in sequence_fields:
                if key not in values:
                    continue
                value = values[key]
                before = previous_sequences.get(key, value)
                sequence_gaps[key] = sequence_gaps.get(key, 0) + max(0, value - before - 1)
                sequence_regressions[key] = sequence_regressions.get(key, 0) + int(value < before)
                previous_sequences[key] = value
    for stats in channels.values():
        seconds = (stats['last_ns'] - stats['first_ns']) / 1e9
        stats['recorded_rate_hz'] = (stats['samples'] - 1) / seconds if seconds > 0 else None
    result = {
        'files': [str(p) for p in files],
        'channels': channels,
        'fields': {key: sorted(value) for key, value in fields.items()},
        'source_sequence_gaps': sequence_gaps,
        'sequence_regressions': sequence_regressions,
        'timestamp_regressions': time_regressions,
        'capture_timestamp_mismatches': capture_regressions,
        'gap_scope': 'missed publications in this recording; not CAN packet loss',
    }
    session = Path(path) if Path(path).is_dir() else Path(path).parent
    metadata = session / 'session.json'
    if metadata.exists():
        result['session'] = json.loads(metadata.read_text())
        from hashlib import sha256

        valid = True
        for source in result['session'].get('sources', {}).values():
            if 'copy' in source:
                copy = session / source['copy']
                valid &= (
                    copy.is_file() and sha256(copy.read_bytes()).hexdigest() == source['sha256']
                )
        result['config_hashes_valid'] = valid
    else:
        result['metadata_missing'] = True
    return result


def state_dataframe(path):
    import pandas as pd

    rows = []
    for stamp, channel, values in snapshots(path):
        if channel == 'qhrr/state':
            rows.append({'timestamp_ns': stamp, **values})
    if not rows:
        raise ValueError('No qhrr/state samples')
    return pd.DataFrame(rows).sort_values('timestamp_ns', kind='stable')
