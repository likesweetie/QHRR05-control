#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
setup=false
bridge_path="$project_dir/build/web_gui_bridge"
server_options=()

usage() {
    cat <<'HELP'
usage: run_web_gui.sh [--setup] [--port N] [--shm-name /NAME]
                      [--control-config YAML] [--input-socket PATH] [--bridge PATH]

Start the local web control panel for an already-running daemon.
Default URL: http://127.0.0.1:8080

--setup  Build the telemetry bridge and install locked npm dependencies first.
         This downloads packages; ordinary startup uses installed dependencies.

The frontend is built on every startup. Relative paths use the repository root.
The script does not start or stop the robot daemon. Ctrl+C stops the web server.
HELP
}

while (($#)); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --setup) setup=true; shift ;;
        --port|--shm-name|--control-config|--input-socket|--bridge)
            if (($# < 2)); then
                echo "[web_gui] $1 requires a value" >&2
                exit 1
            fi

            if [[ "$1" == --bridge ]]; then
                bridge_path="$2"
            fi

            server_options+=("$1" "$2")
            shift 2
            ;;
        *) echo "[web_gui] unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

cd -- "$project_dir"

for executable in node npm; do
    if ! command -v "$executable" >/dev/null 2>&1; then
        echo "[web_gui] required executable not found: $executable" >&2
        exit 1
    fi
done

node -e '
const [major, minor] = process.versions.node.split(".").map(Number);
if (major < 22 || (major === 22 && minor < 12)) {
    console.error("[web_gui] Node.js 22.12 or newer is required; found " + process.versions.node);
    process.exit(1);
}'

if [[ "$setup" == true ]]; then
    if ! command -v cmake >/dev/null 2>&1; then
        echo '[web_gui] --setup requires cmake' >&2
        exit 1
    fi

    echo '[web_gui] preparing the telemetry bridge'
    cmake -S "$project_dir" -B "$project_dir/build"
    cmake --build "$project_dir/build" --parallel 2 --target web_gui_bridge
    echo '[web_gui] installing dependencies from package-lock.json'
    npm --prefix "$project_dir/apps/interfaces/web_gui/frontend" ci --no-audit --no-fund
fi

if [[ ! -x "$bridge_path" ]]; then
    echo "[web_gui] telemetry bridge is missing or not executable: $bridge_path" >&2
    echo '[web_gui] prepare it with: ./scripts/run_web_gui.sh --setup' >&2
    exit 1
fi

if [[ ! -x "$project_dir/apps/interfaces/web_gui/frontend/node_modules/.bin/vite" ]]; then
    echo '[web_gui] frontend dependencies are missing; run ./scripts/run_web_gui.sh --setup' >&2
    exit 1
fi

if ! npm --prefix "$project_dir/apps/interfaces/web_gui/frontend" ls --depth=0 >/dev/null; then
    echo '[web_gui] installed dependencies do not match package.json; use --setup to reinstall' >&2
    exit 1
fi

echo '[web_gui] building the control panel'
npm --prefix "$project_dir/apps/interfaces/web_gui/frontend" run build
echo '[web_gui] starting the local server; an active daemon session is required'
exec node "$project_dir/apps/interfaces/web_gui/frontend/server.mjs" "${server_options[@]}"
