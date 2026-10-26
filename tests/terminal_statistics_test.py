"""Exercise the real relay with PTYs, fragmented stats, retained logs, and bounded redraws."""

import errno
import fcntl
import os
from pathlib import Path
import pty
import select
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
    for broken in (False, True):
        source, writer = pty.openpty()
        output, screen = pty.openpty()
        tty.setraw(writer)
        fcntl.ioctl(screen, termios.TIOCSWINSZ, struct.pack('HHHH', 34 if broken else 0, 140, 0, 0))
        log_path = Path(directory) / f'{broken}.log'
        log_path.touch()
        process = subprocess.Popen(
            [
                str(binary),
                '--terminal-output',
                str(source),
                str(os.getpid()),
                'test-process',
                '1.1',
                str(log_path),
            ],
            pass_fds=(source,),
            stdout=screen,
            stderr=screen,
        )
        os.close(source)
        os.close(screen)
        received = bytearray()

        try:
            if not broken:
                deadline = time.monotonic() + 3

                while b'Waiting for terminal size' not in received:
                    assert time.monotonic() < deadline, received.decode(errors='replace')
                    ready, _, _ = select.select([output], [], [], 0.1)

                    if ready:
                        received.extend(os.read(output, 65536))

                fcntl.ioctl(output, termios.TIOCSWINSZ, struct.pack('HHHH', 34, 140, 0, 0))

            os.write(writer, b'SYSTEM before\n' + marker[:7])
            time.sleep(0.05)
            os.write(writer, marker[7:] + b'STATISTICS snapshot\n')

            if not broken:
                os.write(writer, b'\aSYSTEM after\n')
                deadline = time.monotonic() + 2.6

                while time.monotonic() < deadline:
                    ready, _, _ = select.select([output], [], [], 0.1)

                    if ready:
                        received.extend(os.read(output, 65536))

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

            if broken:
                assert code == 1 and 'incomplete terminal statistics frame' in text, text
            else:
                assert code == 0, text
                assert 2 <= text.count('test-process | PID') <= 3, text
                assert 'CPU=' in text and 'RSS=' in text and 'threads=' in text, text
                assert 'STATISTICS snapshot' in text and 'statistics update in place' in text, text
                assert '\x1b[2J' not in text, 'Periodic full-screen clears erased system messages'
                assert log_path.read_bytes() == b'SYSTEM before\nSYSTEM after\nSYSTEM final\n'

            print(
                'PASS:',
                (
                    'explicit broken-frame failure'
                    if broken
                    else 'process statistics, fragmented frames, retained logs, bounded redraw'
                ),
            )
        finally:
            if writer is not None:
                os.close(writer)
            os.close(output)
            if process.poll() is None:
                process.kill()
                process.wait()
