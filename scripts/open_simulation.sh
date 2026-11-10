#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_PYTHON="${ISAACSIM_PYTHON:-/home/nkj/isaacsim-5.1.0/python.sh}"
STAGE="${MICROMANI_STAGE:-$ROOT_DIR/outputs/assembly_physics/micromani_assembly.usda}"
SESSION="${MICROMANI_RAW_SESSION:-$ROOT_DIR/outputs/gui_verification_$(date +%Y%m%d_%H%M%S)}"
exec "$ISAAC_PYTHON" "$ROOT_DIR/isaacsim_tools/micromani_record_session.py" \
  --stage "$STAGE" --calibration "$ROOT_DIR/config/simulation_data_contract.json" \
  --session "$SESSION" --task 'Synthetic GUI verification' --camera-preview-port "${MICROMANI_PREVIEW_PORT:-49011}" --api-port "${MICROMANI_API_PORT:-49012}" --smoke-frames 60 --view-camera /World/Platform_Overview --keep-open "$@"
