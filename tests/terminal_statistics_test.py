"""PTY regression: alternate screen cleanup, resize, signals, framing and persisted logs."""

import errno
import fcntl
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import termios
import time
import tty

binary = Path(sys.argv[1])
marker = b'\x1b]777;qhrr-statistics;'

with tempfile.TemporaryDirectory(prefix='qhrr05-terminal-statistics-') as directory:
    for kind in ('normal', 'broken', 'interrupt'):
        broken = kind == 'broken'
        source, writer = pty.openpty()
        output, screen = pty.openpty()
        tty.setraw(writer)
        fcntl.ioctl(screen, termios.TIOCSWINSZ, struct.pack('HHHH', 34 if broken else 0, 140, 0, 0))
        os.write(screen, b'SHELL BEFORE\n')
        log_path = Path(directory) / f'{kind}.log'
        log_path.touch()
        supervisor = subprocess.Popen(['/bin/sleep', '30'])
        process = subprocess.Popen(
            [
                str(binary),
                '--terminal-output',
                str(source),
                str(os.getpid()),
                'test-process',
                '1.1',
                str(log_path),
                str(supervisor.pid),
            ],
            pass_fds=(source,),
            stdout=screen,
            stderr=screen,
        )
        os.close(source)
        os.close(screen)
        received = bytearray()

        def read_more(timeout=0.1):
            ready, _, _ = select.select([output], [], [], timeout)
            if ready:
                received.extend(os.read(output, 65536))

        try:
            if not broken:
                deadline = time.monotonic() + 3
                while b'Waiting for terminal size' not in received:
                    assert time.monotonic() < deadline, received.decode(errors='replace')
                    read_more()
                fcntl.ioctl(output, termios.TIOCSWINSZ, struct.pack('HHHH', 34, 140, 0, 0))

            os.write(writer, b'SYSTEM before\n' + marker[:7])
            time.sleep(0.05)
            os.write(writer, marker[7:] + b'\x1b[36mSTATISTICS snapshot\x1b[0m\n')

            if not broken:
                os.write(writer, b'\aSYSTEM after\n')
                deadline = time.monotonic() + 2.4
                while time.monotonic() < deadline:
                    read_more()
                # Changing the viewport must clear obsolete rows and replay cached log messages.
                fcntl.ioctl(output, termios.TIOCSWINSZ, struct.pack('HHHH', 22, 90, 0, 0))
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    read_more()

                if kind == 'interrupt':
                    process.send_signal(signal.SIGINT)
                    assert (
                        supervisor.wait(timeout=2) == -signal.SIGINT
                    ), 'Ctrl+C did not reach daemon supervisor'
                    assert process.poll() is None, 'Relay exited before draining shutdown messages'
                os.write(writer, b'SYSTEM final\n')

            os.close(writer)
            writer = None
            while True:
                ready, _, _ = select.select([output], [], [], 3)
                assert ready, 'Relay failed to close after producer EOF'
                try:
                    block = os.read(output, 65536)
                except OSError as error:
                    if error.errno == errno.EIO:
                        break
                    raise
                if not block:
                    break
                received.extend(block)

            code = process.wait(timeout=3)
            text = received.decode(errors='replace')
            assert text.count('\x1b[?1049h') == 1 and text.count('\x1b[?1049l') == 1, text
            restore = text.index('\x1b[?1049l')
            assert (
                '\x1b[0m\x1b[r\x1b[?25h' in text[:restore]
            ), 'Style/scroll/cursor state was not restored'
            assert '\x1b[23;0t' in text[restore:], 'Terminal title stack was not restored'
            assert (
                'STATISTICS snapshot' not in text[restore:]
            ), 'Dashboard escaped into the normal screen'
            if broken:
                assert code == 1 and 'incomplete terminal statistics frame' in text, text
            else:
                assert code == 0, text
                assert 3 <= text.count('test-process | PID') <= 4, text
                assert 'CPU=' in text and 'RSS=' in text and 'threads=' in text, text
                assert 'STATISTICS snapshot' in text and 'recent messages' in text, text
                assert text.count('\x1b[2J') <= 4, 'Unnecessary full redraw on a stable viewport'
                assert text.count('SYSTEM before') >= 2, 'Resize discarded the recent log pane'
                assert log_path.read_bytes() == b'SYSTEM before\nSYSTEM after\nSYSTEM final\n'
            print('PASS:', kind, 'screen/title/cursor restoration, logs, resize and signal cleanup')
        finally:
            if writer is not None:
                os.close(writer)
            os.close(output)
            if process.poll() is None:
                process.kill()
                process.wait()
            if supervisor.poll() is None:
                supervisor.terminate()
                supervisor.wait()
