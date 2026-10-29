#!/usr/bin/env bash
# Local Clang 18 provisioning for Ubuntu 22.04; does not install/change host packages.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
toolchain_dir="$project_dir/build-filament/toolchain"
mkdir -p "$toolchain_dir/llvm18-debs" "$toolchain_dir/bin"
echo '[toolchain] Downloading pinned Clang 18.1.8 into build-filament/toolchain (host packages unchanged)'
python3 "$project_dir/tools/fetch_visualization_toolchain.py" "$toolchain_dir"
for compiler in clang clang++; do
    cat > "$toolchain_dir/bin/$compiler-18" <<'WRAPPER'
#!/usr/bin/env bash
set -euo pipefail
toolchain_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LD_LIBRARY_PATH="$toolchain_dir/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
compiler="$(basename -- "$0")"
exec "$toolchain_dir/usr/bin/$compiler" "$@"
WRAPPER
    chmod +x "$toolchain_dir/bin/$compiler-18"
done
"$toolchain_dir/bin/clang++-18" --version
