"""Small X11 GUI helpers shared by integration tests; no hidden retries or fallback."""

import subprocess
import time


def event(*args):
    return subprocess.run(['xdotool', *map(str, args)], check=True, capture_output=True, text=True)


def wait_for(predicate, message, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.05)
    raise AssertionError(message())


def windows(pattern='.'):
    result = subprocess.run(
        ['xdotool', 'search', '--onlyvisible', '--name', pattern], capture_output=True, text=True
    )
    if result.returncode not in (0, 1):
        result.check_returncode()
    return result.stdout.splitlines() if result.returncode == 0 else []


def find_window(pattern):
    found = windows(pattern)
    return found[-1] if found else None


def geometry(window):
    fields = dict(
        line.split('=', 1)
        for line in event('getwindowgeometry', '--shell', window).stdout.splitlines()
        if '=' in line
    )
    return tuple(int(fields[key]) for key in ('X', 'Y', 'WIDTH', 'HEIGHT'))
