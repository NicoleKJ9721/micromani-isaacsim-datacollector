#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"
PYTHON="${LEROBOT_PYTHON:-$ROOT_DIR/.venv-lerobot/bin/python}"
if [[ ! -x "$PYTHON" ]]; then PYTHON=python3; fi

"$PYTHON" -m py_compile \
  isaacsim_tools/micromani_lerobot_schema.py \
  isaacsim_tools/micromani_collect_isaac.py \
  isaacsim_tools/micromani_configure_cameras.py \
  isaacsim_tools/micromani_configure_ft_sensors.py \
  isaacsim_tools/micromani_convert_to_lerobot.py \
  isaacsim_tools/micromani_replay_lerobot.py \
  isaacsim_tools/micromani_pico_gateway.py \
  isaacsim_tools/micromani_pico_mapping.py \
  isaacsim_tools/test_micromani_plugin_buttons.py \
  isaacsim_tools/validate_micromani_build.py \
  exts/micromani.data_collector/micromani/data_collector/extension.py

bash -n \
  isaacsim_tools/run_micromani_lerobot_pipeline.sh \
  isaacsim_tools/backup_cloud_instance.sh \
  scripts/install_isaac_extension.sh

"$PYTHON" scripts/validate_project_delivery.py

echo "Local checks passed."
