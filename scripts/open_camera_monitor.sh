#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec /usr/bin/python3 "$ROOT_DIR/isaacsim_tools/micromani_camera_monitor.py" --port "${MICROMANI_PREVIEW_PORT:-49011}" "$@"
