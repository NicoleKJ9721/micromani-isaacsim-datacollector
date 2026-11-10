#!/usr/bin/env bash
# Start the monitor independently; it reconnects automatically if the simulator restarts.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ROOT_DIR/scripts/open_camera_monitor.sh" &
exec "$ROOT_DIR/scripts/open_simulation.sh" "$@"
