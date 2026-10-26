#!/usr/bin/env python3
"""Offline field listing, alignment, force errors, observed timing and session comparison."""

import argparse
import json
from pathlib import Path
import sys
from log_data import state_dataframe, inspect


def metrics(frame):
    import numpy as np

    result = {'samples': len(frame)}
    if 'control/published_ns' in frame and 'control/command_sequence' in frame:
        unique = frame.drop_duplicates('control/command_sequence')
        sequence = unique['control/command_sequence'].diff()
        intervals = unique['control/published_ns'].diff()[sequence == 1].to_numpy() / 1000
        intervals = intervals[intervals > 0]
        result['observed_single_step_intervals'] = len(intervals)
        result['timing_scope'] = (
            'Only adjacent observed command sequences; unobserved cycles unavailable'
        )
        if len(intervals):
            center = float(np.median(intervals))
            jitter = np.abs(intervals - center)
            result['observed_period_median_us'] = center
            result['observed_jitter_p99_us'] = float(np.percentile(jitter, 99))
            result['observed_jitter_p999_us'] = float(np.percentile(jitter, 99.9))
            result['observed_period_max_us'] = float(intervals.max())
    if 'simulation/valid' in frame and frame['simulation/valid'].any():
        valid = frame['simulation/valid'].fillna(False).astype(bool)
        if 'control/magnetic/observer_valid' in frame:
            valid &= frame['control/magnetic/observer_valid'].fillna(False).astype(bool)
        errors = []
        for axis in range(3):
            est = f'control/magnetic/estimated_force[{axis}]'
            truth = f'simulation/ground_truth/contact_force[{axis}]'
            if est in frame and truth in frame:
                delta = (frame.loc[valid, est] - frame.loc[valid, truth]).dropna().to_numpy()
                if len(delta):
                    errors.append(delta)
                    result[f'force_axis_{axis}_bias_n'] = float(delta.mean())
        if errors:
            all_errors = np.concatenate(errors)
            result['observer_error_samples'] = len(all_errors)
            result['observer_force_rmse_n'] = float(np.sqrt(np.mean(all_errors**2)))
            result['observer_force_max_abs_n'] = float(np.max(np.abs(all_errors)))
    for axis in ('HR', 'HP', 'KP'):
        target, actual = f'motor/{axis}/command/position', f'motor/{axis}/position_rad'
        if target in frame and actual in frame:
            # Both values are raw motor coordinates; do not add joint-coordinate offsets.
            error = (frame[target] - frame[actual]).dropna().to_numpy()
            if len(error):
                result[f'{axis}_motor_target_rmse_rad'] = float(np.sqrt(np.mean(error**2)))
    return result


def align(frame, fields, hz):
    import numpy as np
    import pandas as pd

    if hz <= 0:
        raise ValueError('Alignment rate must be positive')
    selected = (
        frame[['timestamp_ns', *fields]].drop_duplicates('timestamp_ns').set_index('timestamp_ns')
    )
    origin = int(selected.index[0])
    selected.index = (selected.index - origin) / 1e9
    grid = np.arange(float(selected.index[0]), float(selected.index[-1]), 1 / hz)
    # No extrapolation. Source time/sequence fields remain available in the original frame.
    union = selected.index.union(pd.Index(grid)).sort_values()
    return (
        selected.reindex(union)
        .interpolate(method='index', limit_area='inside')
        .reindex(grid)
        .rename_axis('elapsed_seconds')
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session')
    parser.add_argument('--fields', action='store_true')
    parser.add_argument('--signals', nargs='+')
    parser.add_argument('--align-hz', type=float)
    parser.add_argument('--csv')
    parser.add_argument('--plot')
    parser.add_argument('--report')
    parser.add_argument('--compare', nargs='+')
    args = parser.parse_args()
    if args.fields:
        print(json.dumps(inspect(args.session)['fields'], indent=2))
        return 0
    frame = state_dataframe(args.session)
    chosen = args.signals or [f'motor/{axis}/position_rad' for axis in ('HR', 'HP', 'KP')]
    missing = set(chosen) - set(frame.columns)
    if missing:
        raise ValueError(f'Unknown fields: {sorted(missing)}')
    if args.align_hz:
        non_numeric = [key for key in chosen if not frame[key].dtype.kind in 'fiu']
        if non_numeric:
            raise ValueError(f'Cannot interpolate categorical signals: {non_numeric}')
        data = align(frame, chosen, args.align_hz)
    else:
        data = frame[['timestamp_ns', *chosen]]
    if args.csv:
        with Path(args.csv).open('x') as stream:
            data.to_csv(stream, index=bool(args.align_hz))
    report = metrics(frame)
    if args.compare:
        report = {args.session: report, **{p: metrics(state_dataframe(p)) for p in args.compare}}
    if args.report:
        with Path(args.report).open('x') as stream:
            json.dump(report, stream, indent=2)
    if args.plot:
        if Path(args.plot).exists():
            raise FileExistsError(args.plot)
        import matplotlib

        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        if args.align_hz:
            data.plot()
        else:
            chart = data.copy()
            chart['elapsed_seconds'] = (chart['timestamp_ns'] - chart['timestamp_ns'].iloc[0]) / 1e9
            chart.plot(x='elapsed_seconds', y=chosen)
        plt.tight_layout()
        plt.savefig(args.plot)
        plt.close()
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(f'[qhrr-log-analyze] ERROR: {error}', file=sys.stderr)
        sys.exit(2)
