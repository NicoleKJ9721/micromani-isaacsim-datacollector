#!/bin/bash
# 在独立输出目录构建本体/传感器/外观，保留原URDF和已存在场景。
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
: "${ISAACSIM_PYTHON:?Set ISAACSIM_PYTHON to Isaac Sim python.sh}"
OUT="${1:?Usage: build_equipped_scene.sh NEW_OUTPUT_DIRECTORY}"
if [[ -e "$OUT" ]]; then
  echo "Output already exists: $OUT" >&2
  exit 1
fi
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
python3 - "$ROOT_DIR" "$OUT" <<'PY'
import sys,xml.etree.ElementTree as E
from pathlib import Path
root,out=map(Path,sys.argv[1:])
tree=E.parse(root/'urdf/微纳机器人URDF.urdf')
tree.getroot().set('name','micromanipulator')
tree.write(out/'micromanipulator.urdf',encoding='utf-8',xml_declaration=True)
PY
MICROMANI_ASSET_ROOT="$ROOT_DIR" MICROMANI_URDF="$OUT/micromanipulator.urdf" \
 MICROMANI_OUT_DIR="$OUT/import" "$ISAACSIM_PYTHON" "$ROOT_DIR/isaacsim_tools/import_micromanipulator.py" > "$OUT/import.log" 2>&1
"$ISAACSIM_PYTHON" "$ROOT_DIR/isaacsim_tools/configure_and_test_hardware.py" \
 --stage "$OUT/import/micromanipulator_scene.usd" --out "$OUT/equipped" > "$OUT/equipped.log" 2>&1
python3 - "$OUT/equipped/hardware_report.json" <<'PY'
import json,sys
report=json.load(open(sys.argv[1]))
if not report.get('passed'):
    raise SystemExit('Hardware verification failed; inspect hardware_report.json')
print('Hardware verification passed:',sys.argv[1])
PY
