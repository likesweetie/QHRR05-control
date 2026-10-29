#!/usr/bin/env bash
# This child can also be started by the daemon; it never manages physics.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$project_dir/build-filament/native/bin/qhrr_visualizer"
source "$project_dir/scripts/ensure_build.sh"
qhrr_ensure_visualization_build "$project_dir"
backend="${QHRR_VISUALIZATION_BACKEND:-filament}"
case "$backend" in
    filament)
        icd="${QHRR_NVIDIA_ICD:-/usr/share/vulkan/icd.d/nvidia_icd.json}"
        [[ -r "$icd" ]] || { echo "NVIDIA Vulkan ICD missing: $icd" >&2; exit 1; }
        export VK_DRIVER_FILES="$icd" VK_ICD_FILENAMES="$icd"
        ibl="$project_dir/models/qhrr-05/visualization/environment/product_studio_ibl.ktx"
        [[ -r "$ibl" ]] || { echo "Product-studio IBL is missing: $ibl" >&2; exit 1; }
        model_args=(--model "$project_dir/models/qhrr-05/scene_pbr.xml" --pbr-materials --ibl "$ibl")
        ;;
    classic) model_args=(--model "$project_dir/models/qhrr-05/scene.xml") ;;
    *) echo "unknown visualization backend: $backend" >&2; exit 1 ;;
esac
quality_args=()
if [[ -n "${QHRR_VISUALIZATION_MSAA:-}" ]]; then quality_args+=(--msaa "$QHRR_VISUALIZATION_MSAA"); fi
if [[ -n "${QHRR_VISUALIZATION_RENDER_SCALE:-}" ]]; then quality_args+=(--render-scale "$QHRR_VISUALIZATION_RENDER_SCALE"); fi
exec "$binary" "${quality_args[@]}" --backend "$backend" --fps "${QHRR_VISUALIZATION_FPS:-60}" "${model_args[@]}" "$@"
