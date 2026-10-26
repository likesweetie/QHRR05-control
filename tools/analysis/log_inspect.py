#!/usr/bin/env python3
"""Validate MCAP/schema and print recorded counts, time bounds and source discontinuities."""

import argparse
import json
import sys
from log_data import inspect


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session')
    parser.add_argument('--faults', action='store_true')
    parser.add_argument('--fields', action='store_true')
    parser.add_argument('--output')
    args = parser.parse_args()
    result = inspect(args.session, args.faults)
    if not args.fields:
        result.pop('fields')
    text = json.dumps(result, indent=2)
    if args.output:
        from pathlib import Path

        with Path(args.output).open('x') as stream:
            stream.write(text + '\n')
    print(text)
    return int(
        bool(
            result['timestamp_regressions']
            or result['capture_timestamp_mismatches']
            or not result.get('config_hashes_valid', True)
        )
    )


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(f'[qhrr-log-inspect] ERROR: {error}', file=sys.stderr)
        sys.exit(2)
