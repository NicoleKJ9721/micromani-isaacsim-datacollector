#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXT_DIR="${ROOT_DIR}/exts"
EXT_NAME="micromani.data_collector"

cat <<EOF
MicroMani Isaac Sim extension is ready.

Recommended launch command:

  /path/to/isaac-sim.sh --ext-folder "${EXT_DIR}" --enable "${EXT_NAME}"

If Isaac Sim is installed in a known directory, run:

  ISAACSIM_ROOT=/path/to/isaac-sim "${BASH_SOURCE[0]}" --print-launcher

EOF

if [[ "${1:-}" == "--print-launcher" ]]; then
  if [[ -z "${ISAACSIM_ROOT:-}" ]]; then
    echo "Set ISAACSIM_ROOT first." >&2
    exit 1
  fi
  echo "${ISAACSIM_ROOT}/isaac-sim.sh --ext-folder \"${EXT_DIR}\" --enable \"${EXT_NAME}\""
fi
