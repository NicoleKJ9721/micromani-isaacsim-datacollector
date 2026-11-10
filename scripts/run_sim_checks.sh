#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
# 请选择已有 LeRobot 环境；缺少依赖直接报错，不自动安装。
PYTHON="${LEROBOT_PYTHON:-$ROOT_DIR/.venv-lerobot/bin/python}"
if [[ ! -x "$PYTHON" ]]; then PYTHON=python3; fi
"$PYTHON" -m py_compile isaacsim_tools/micromani_record_session.py isaacsim_tools/micromani_export_session.py isaacsim_tools/micromani_sim_contract.py isaacsim_tools/micromani_episode.py isaacsim_tools/micromani_replay_lerobot.py
"$PYTHON" -m unittest discover -s tests -v
