#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-record}"
shift || true
: "${ISAACSIM_PYTHON:?Set ISAACSIM_PYTHON to Isaac Sim python.sh}"
: "${MICROMANI_STAGE:?Set MICROMANI_STAGE to the configured USD scene}"
SESSION="${MICROMANI_SESSION_NAME:-micromani_$(date +%Y%m%d_%H%M%S)}"
RAW="${MICROMANI_RAW_SESSION:-$ROOT_DIR/datasets/raw/$SESSION}"
CALIBRATION="${MICROMANI_CALIBRATION:-$ROOT_DIR/config/simulation_data_contract.json}"
ARGS=(--stage "$MICROMANI_STAGE" --calibration "$CALIBRATION" --session "$RAW" --task "${MICROMANI_TASK:-Bimanual micro manipulation}" --fps "${MICROMANI_FPS:-30}")
if [[ "$MODE" == smoke ]]; then
    ARGS+=(--headless --smoke-frames "${MICROMANI_FRAMES:-6}")
elif [[ "$MODE" != record ]]; then
    echo "Usage: $0 record|smoke [recorder arguments]" >&2
    exit 2
fi
"$ISAACSIM_PYTHON" "$ROOT_DIR/isaacsim_tools/micromani_record_session.py" "${ARGS[@]}" "$@"
if [[ "$MODE" == smoke ]]; then
    : "${LEROBOT_PYTHON:?Set LEROBOT_PYTHON to an existing LeRobot environment}"
    "$LEROBOT_PYTHON" "$ROOT_DIR/isaacsim_tools/micromani_convert_to_lerobot.py" --raw-session "$RAW" --output-root "$ROOT_DIR/datasets/lerobot/$SESSION"
fi
