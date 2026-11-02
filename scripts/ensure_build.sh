#!/usr/bin/env bash
# Build helpers shared by launchers; session cleanup is used by the hardware launcher.

qhrr_cleanup_session() {
    python3 - "$1" "$2" <<'PY'
import fcntl
import os
from pathlib import Path
import re
import signal
import stat
import sys
import time
import yaml

root, manifest = (Path(value).resolve() for value in sys.argv[1:])
try:
    config = yaml.safe_load(manifest.read_text())
    if config.get('runtime_mode') != 'hardware':
        raise ValueError('run_daemon.sh requires runtime_mode: hardware')
    session = config['shm_name']
    if not isinstance(session, str) or not re.fullmatch(r'/[A-Za-z0-9_.-]+', session) or session in ('/.', '/..'):
        raise ValueError('invalid daemon shm_name')
    uid = os.getuid()
    build_dirs = {root / 'build', root / 'build-filament/native/bin'}
    daemon = root / 'build/daemon'
    web_scripts = {root / 'scripts/run_web_gui.sh', root / 'apps/interfaces/web_gui/frontend/server.mjs'}
    lock_dir = root / '.cache/launch-locks'
    lock_dir.mkdir(parents=True, exist_ok=True)
    lock = (lock_dir / (session[1:] + '.lock')).open('a')
    fcntl.flock(lock, fcntl.LOCK_EX)

    def read_process(pid):
        directory = Path('/proc') / str(pid)
        try:
            if directory.stat().st_uid != uid:
                return None
            fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            args = [value.decode(errors='replace') for value in (directory / 'cmdline').read_bytes().split(b'\0') if value]
            try:
                exe = Path(os.readlink(directory / 'exe').removesuffix(' (deleted)'))
                cwd = Path(os.readlink(directory / 'cwd'))
            except OSError:
                exe, cwd = Path('/'), root
            return {'pid': pid, 'parent': int(fields[1]), 'state': fields[0],
                    'start': fields[19], 'args': args, 'exe': exe, 'cwd': cwd}
        except (OSError, ValueError, IndexError):
            return None

    def absolute(process, value):
        path = Path(value)
        return (process['cwd'] / path).resolve() if not path.is_absolute() else path.resolve()

    protected = set()
    pid = os.getpid()
    while pid and pid not in protected:
        protected.add(pid)
        parent = read_process(pid)
        pid = parent['parent'] if parent else 0
    targets = {}

    def discover():
        processes = {int(p.name): read_process(int(p.name)) for p in Path('/proc').iterdir() if p.name.isdigit()}
        processes = {pid: p for pid, p in processes.items() if p is not None}
        owners = []
        for pid, p in processes.items():
            args = p['args']
            paths = {p['exe']}
            interpreted = p['exe'].name in ('bash', 'sh', 'node') or p['exe'].name.startswith('python')
            for value in args[:2] if interpreted else args[:1]:
                if value and not value.startswith('-'):
                    try:
                        path = absolute(p, value)
                        if path.is_file() and (interpreted or os.access(path, os.X_OK)):
                            paths.add(path)
                    except (OSError, ValueError):
                        continue  # Some applications replace argv[0] with a display title.
            native = any(path.parent in build_dirs for path in paths)
            matching = (session in args or '--shm-name=' + session in args) and (native or bool(paths & web_scripts))
            if daemon in paths and '--terminal-output' not in args:
                try:
                    path = absolute(p, args[args.index('--config') + 1]) if '--config' in args else root / 'config/daemon/daemon.yaml'
                    if yaml.safe_load(path.read_text()).get('shm_name') == session:
                        matching = True
                        owners.append(pid)
                except (OSError, ValueError, IndexError, AttributeError, yaml.YAMLError):
                    pass
            if matching and pid not in protected:
                targets[pid] = p['start']
        # Capture children before terminating the owner, including stopped workers
        # and zombies that must be reaped by that owner.
        while True:
            added = [p for pid, p in processes.items() if pid not in targets and pid not in protected and p['parent'] in targets]
            if not added:
                break
            targets.update({p['pid']: p['start'] for p in added})
        for pid, p in processes.items():
            args = p['args']
            if '--terminal-output' not in args:
                continue
            i = args.index('--terminal-output')
            try:
                child, owner = int(args[i + 2]), int(args[i + 6])
                if absolute(p, args[i - 1]) != daemon:
                    continue
                orphan = child not in processes and absolute(p, args[i + 5]).is_relative_to(root / 'logs')
                if pid not in protected and (child in targets or owner in targets or orphan):
                    targets[pid] = p['start']
            except (ValueError, IndexError):
                continue
        return owners

    def live(pids=None):
        result = []
        for pid in targets if pids is None else pids:
            p = read_process(pid)
            if p and p['start'] == targets.get(pid) and p['state'] != 'Z':
                result.append(p)
        return result

    def send(processes, sig):
        for p in processes:
            try:
                # pidfd pins the process identity even if the PID is recycled.
                fd = os.pidfd_open(p['pid'])
                try:
                    current = read_process(p['pid'])
                    if current and current['start'] == p['start']:
                        signal.pidfd_send_signal(fd, sig)
                        if sig == signal.SIGTERM and current['state'] in ('T', 't'):
                            signal.pidfd_send_signal(fd, signal.SIGCONT)
                finally:
                    os.close(fd)
            except ProcessLookupError:
                pass

    def wait_for_exit(seconds, pids=None):
        deadline = time.monotonic() + seconds
        while live(pids) and time.monotonic() < deadline:
            time.sleep(.05)

    owners = discover()
    if targets:
        print(f'[launcher] Cleaning previous {session} session: PIDs ' + ','.join(map(str, sorted(targets))), flush=True)
    send(live(owners), signal.SIGTERM)
    wait_for_exit(3.5, owners)
    discover()
    send(live(), signal.SIGTERM)
    wait_for_exit(2)
    remaining = live()
    if remaining:
        print('[launcher] Force-stopping unresponsive PIDs: ' + ','.join(str(p['pid']) for p in remaining), flush=True)
        send(remaining, signal.SIGKILL)
        wait_for_exit(1)
    discover()
    if live():
        raise RuntimeError('previous session processes did not exit; SHM was left intact')

    shm = Path('/dev/shm') / session[1:]
    # Never unlink a mapping still used by an unrecognized live application.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        p = read_process(int(entry.name))
        if not p or p['state'] == 'Z':
            continue
        try:
            if any(line.rstrip().endswith(' ' + str(shm)) for line in (entry / 'maps').read_text().splitlines()):
                raise RuntimeError(f'{session} is still mapped by PID {entry.name}; refusing to unlink it')
        except (FileNotFoundError, ProcessLookupError):
            pass
        except PermissionError:
            if session in p['args'] or '--shm-name=' + session in p['args']:
                raise RuntimeError(f'cannot verify session resources held by PID {entry.name}')
            # Unrelated protected desktop services need not be stopped or elevated.
    sockets = [Path(f'/tmp/qhrr05-{kind}-{uid}') / (session[1:] + '.sock') for kind in ('daemon', 'input')]
    sockets.append(Path(f'/tmp/qhrr05-log-{uid}-{session[1:]}.sock'))
    for path in [shm, *sockets]:
        try:
            info = path.lstat()
        except FileNotFoundError:
            continue
        expected = stat.S_ISREG(info.st_mode) if path == shm else stat.S_ISSOCK(info.st_mode)
        if info.st_uid != uid or not expected:
            raise RuntimeError(f'refusing to remove unexpected session resource: {path}')
        path.unlink()
        print(f'[launcher] Removed stale {path}', flush=True)
    print(f'[launcher] Previous session cleanup complete: {session}', flush=True)
except (OSError, ValueError, KeyError, AttributeError, RuntimeError, yaml.YAMLError) as error:
    sys.exit(f'[launcher] Session cleanup failed: {error}')
PY
}

qhrr_native_ready() {
    local build_dir="$1" target binary
    shift
    for target in "$@"; do
        binary="$target"
        [[ "$target" != qhrr_logctl ]] || binary=qhrr-logctl
        [[ -x "$build_dir/$binary" ]] || return 1
    done
}

qhrr_visualization_ready() {
    local directory="$1/build-filament/native/bin" asset
    [[ -x "$directory/qhrr_visualizer" ]] || return 1
    [[ -s "$directory/assets/.qhrr-manifest" ]] || return 1
    [[ -s "$directory/assets/qhrr_pbr.filamat" ]] || return 1
    while IFS= read -r asset; do
        [[ "$asset" =~ ^[a-zA-Z0-9_.-]+$ ]] || return 1
        [[ -s "$directory/assets/$asset" ]] || return 1
    done < "$directory/assets/.qhrr-manifest"
}

qhrr_install_visualization_assets() {
    local source="$1/build-filament/native/src/render/filament/assets"
    local destination="$1/build-filament/native/bin/assets" file
    [[ -s "$source/ibl.ktx" ]] || {
        echo '[build] Compiled visualization assets are missing' >&2
        return 1
    }
    cmake -E copy_directory "$source" "$destination"
    # Copy on every recovery build, even when the executable did not need relinking.
    # Atomic manifest replacement prevents an interrupted copy from appearing ready.
    printf 'ibl.ktx\n' > "$destination/.qhrr-manifest.tmp"
    for file in "$source"/*.filamat; do
        [[ -s "$file" ]] || {
            echo '[build] Compiled shader is missing' >&2
            return 1
        }
        printf '%s\n' "${file##*/}" >> "$destination/.qhrr-manifest.tmp"
    done
    mv -- "$destination/.qhrr-manifest.tmp" "$destination/.qhrr-manifest"
}

qhrr_build_tools() {
    local tool
    for tool in cmake flock; do
        command -v "$tool" >/dev/null || {
            echo "[build] required tool is missing: $tool" >&2
            return 1
        }
    done
    [[ "${QHRR_BUILD_JOBS:-4}" =~ ^[1-9][0-9]*$ ]] || {
        echo '[build] QHRR_BUILD_JOBS must be a positive integer' >&2
        return 1
    }
}

qhrr_ensure_native_build() (
    set -euo pipefail
    local project_dir="$1"
    shift
    qhrr_build_tools
    mkdir -p "$project_dir/.cache/build-locks"
    exec 9>"$project_dir/.cache/build-locks/native.lock"
    flock 9
    # Always let CMake check source/header/build-file dependencies, even when
    # executables exist. The lock also protects concurrent incremental builds.
    if qhrr_native_ready "$project_dir/build" "$@"; then
        echo "[build] Checking native source changes: $*"
    else
        echo "[build] Native executable missing; automatically building: $*"
    fi
    if [[ -f "$project_dir/build/CMakeCache.txt" ]]; then
        cmake -S "$project_dir" -B "$project_dir/build"
    else
        cmake -S "$project_dir" -B "$project_dir/build" -DCMAKE_BUILD_TYPE=Debug
    fi
    cmake --build "$project_dir/build" --parallel "${QHRR_BUILD_JOBS:-4}" --target "$@"
    qhrr_native_ready "$project_dir/build" "$@" || {
        echo '[build] Native build finished without the required executables; not launching' >&2
        exit 1
    }
)

qhrr_ensure_visualization_build() (
    set -euo pipefail
    local project_dir="$1" force="${2:-0}"
    if [[ "$force" == 0 ]] && qhrr_visualization_ready "$project_dir"; then
        exit 0
    fi
    qhrr_build_tools
    mkdir -p "$project_dir/.cache/build-locks"
    exec 9>"$project_dir/.cache/build-locks/visualization.lock"
    flock 9
    if [[ "$force" == 0 ]] && qhrr_visualization_ready "$project_dir"; then
        exit 0
    fi
    echo '[build] Preparing the pinned visualizer and runtime assets before launch'
    echo '[build] A clean build may download the pinned sources/toolchain and take several minutes'
    "$project_dir/scripts/build_visualization.sh" --jobs "${QHRR_BUILD_JOBS:-8}"
    qhrr_install_visualization_assets "$project_dir"
    qhrr_visualization_ready "$project_dir" || {
        echo '[build] Visualization build finished without the executable/assets; not launching' >&2
        exit 1
    }
)
