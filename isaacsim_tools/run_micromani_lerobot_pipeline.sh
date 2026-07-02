#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_HOST="${MICROMANI_REMOTE_HOST:-117.50.205.12}"
REMOTE_USER="${MICROMANI_REMOTE_USER:-ubuntu}"
SSH_KEY="${MICROMANI_SSH_KEY:-$HOME/.ssh/isaacsim_uyun_backup_ed25519}"
REMOTE_DATA_DIR="${MICROMANI_REMOTE_DATA_DIR:-/home/ubuntu/docker/isaac-sim/data/micromani}"
REMOTE_ISAAC_DIR="/isaac-sim/.local/share/ov/data/micromani"
LOCAL_RAW_DIR="${MICROMANI_LOCAL_RAW_DIR:-$ROOT_DIR/datasets/raw}"
LOCAL_LEROBOT_DIR="${MICROMANI_LOCAL_LEROBOT_DIR:-$ROOT_DIR/datasets/lerobot/micromani_isaac_smoke}"
REPORT_DIR="${MICROMANI_REPORT_DIR:-$ROOT_DIR/outputs}"
LEROBOT_PYTHON="${LEROBOT_PYTHON:-}"
FRAMES="${MICROMANI_FRAMES:-18}"
FPS="${MICROMANI_FPS:-30}"
SESSION_NAME="${MICROMANI_SESSION_NAME:-micromani_smoke_$(date +%Y%m%d_%H%M%S)}"
REPO_ID="${MICROMANI_REPO_ID:-micromani/isaac_micro_bimanual}"
LOCAL_BRIDGE_PORT="${MICROMANI_LOCAL_BRIDGE_PORT:-49012}"
SSH_CTL="${MICROMANI_SSH_CTL:-/tmp/micromani_isaac_tunnel.sock}"
FT_MODEL="${MICROMANI_FT_MODEL:-HKVL-36A}"

SSH=(ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no -o BatchMode=yes "$REMOTE_USER@$REMOTE_HOST")
SCP=(scp -i "$SSH_KEY" -o StrictHostKeyChecking=no -o BatchMode=yes)
RSYNC=(rsync -az -e "ssh -i $SSH_KEY -o StrictHostKeyChecking=no -o BatchMode=yes")

detect_lerobot_python() {
  if [[ -n "$LEROBOT_PYTHON" ]]; then
    printf '%s\n' "$LEROBOT_PYTHON"
    return 0
  fi
  local candidates=(
    "$ROOT_DIR/.venv/bin/python"
    "/opt/homebrew/Caskroom/mambaforge/base/envs/lerobot/bin/python"
    "$(command -v python3 || true)"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -x "$candidate" ]] && "$candidate" -c 'import lerobot' >/dev/null 2>&1; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  echo "Could not find a Python interpreter with lerobot installed. Set LEROBOT_PYTHON=/path/to/python." >&2
  return 1
}

LEROBOT_PYTHON="$(detect_lerobot_python)"

mkdir -p "$LOCAL_RAW_DIR" "$REPORT_DIR"

echo "[1/6] Uploading collection scripts"
"${SSH[@]}" "mkdir -p '$REMOTE_DATA_DIR'"
"${SCP[@]}" \
  "$ROOT_DIR/isaacsim_tools/micromani_lerobot_schema.py" \
  "$ROOT_DIR/isaacsim_tools/micromani_configure_ft_sensors.py" \
  "$ROOT_DIR/isaacsim_tools/micromani_collect_isaac.py" \
  "$REMOTE_USER@$REMOTE_HOST:/home/$REMOTE_USER/"
"${SSH[@]}" "sudo -n install -m 0644 /home/$REMOTE_USER/micromani_lerobot_schema.py '$REMOTE_DATA_DIR/micromani_lerobot_schema.py' && sudo -n install -m 0755 /home/$REMOTE_USER/micromani_configure_ft_sensors.py '$REMOTE_DATA_DIR/micromani_configure_ft_sensors.py' && sudo -n install -m 0755 /home/$REMOTE_USER/micromani_collect_isaac.py '$REMOTE_DATA_DIR/micromani_collect_isaac.py' && rm -f /home/$REMOTE_USER/micromani_lerobot_schema.py /home/$REMOTE_USER/micromani_configure_ft_sensors.py /home/$REMOTE_USER/micromani_collect_isaac.py"

echo "[2/6] Configuring HKVL-36 force/torque sensors"
"${SSH[@]}" "sudo -n docker exec isaac-sim sh -lc 'rm -f $REMOTE_ISAAC_DIR/hkvl36_force_torque_sensors.json && /isaac-sim/python.sh $REMOTE_ISAAC_DIR/micromani_configure_ft_sensors.py --stage $REMOTE_ISAAC_DIR/micromanipulator_scene.usd --model $FT_MODEL --sample-fps $FPS --report $REMOTE_ISAAC_DIR/hkvl36_force_torque_sensors.json && test -s $REMOTE_ISAAC_DIR/hkvl36_force_torque_sensors.json'"

echo "[3/6] Collecting raw Isaac episode"
"${SSH[@]}" "sudo -n docker exec isaac-sim sh -lc '/isaac-sim/python.sh $REMOTE_ISAAC_DIR/micromani_collect_isaac.py --frames $FRAMES --fps $FPS --session-name $SESSION_NAME --force-torque-model $FT_MODEL'"

echo "[4/6] Downloading raw episode"
"${RSYNC[@]}" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DATA_DIR/lerobot_raw/$SESSION_NAME/" "$LOCAL_RAW_DIR/$SESSION_NAME/"

echo "[5/6] Converting to LeRobot dataset"
"$LEROBOT_PYTHON" "$ROOT_DIR/isaacsim_tools/micromani_convert_to_lerobot.py" \
  --raw-session "$LOCAL_RAW_DIR/$SESSION_NAME" \
  --output-root "$LOCAL_LEROBOT_DIR" \
  --repo-id "$REPO_ID" \
  --overwrite

echo "[6/6] Replaying LeRobot actions to Isaac control bridge"
ssh -S "$SSH_CTL" -O exit "$REMOTE_USER@$REMOTE_HOST" 2>/dev/null || true
ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no -o BatchMode=yes -M -S "$SSH_CTL" \
  -fN -L "$LOCAL_BRIDGE_PORT:127.0.0.1:49011" "$REMOTE_USER@$REMOTE_HOST"
trap 'ssh -S "$SSH_CTL" -O exit "$REMOTE_USER@$REMOTE_HOST" 2>/dev/null || true' EXIT
"$LEROBOT_PYTHON" "$ROOT_DIR/isaacsim_tools/micromani_replay_lerobot.py" \
  --dataset-root "$LOCAL_LEROBOT_DIR" \
  --repo-id "$REPO_ID" \
  --host "127.0.0.1" \
  --port "$LOCAL_BRIDGE_PORT" \
  --max-frames "$FRAMES" \
  --report "$REPORT_DIR/micromani_replay_report.json"
ssh -S "$SSH_CTL" -O exit "$REMOTE_USER@$REMOTE_HOST" 2>/dev/null || true
trap - EXIT

cat > "$REPORT_DIR/micromani_pipeline_report.json" <<JSON
{
  "session_name": "$SESSION_NAME",
  "remote_host": "$REMOTE_HOST",
  "frames": $FRAMES,
  "fps": $FPS,
  "force_torque_model": "$FT_MODEL",
  "raw_session": "$LOCAL_RAW_DIR/$SESSION_NAME",
  "lerobot_dataset": "$LOCAL_LEROBOT_DIR",
  "replay_report": "$REPORT_DIR/micromani_replay_report.json"
}
JSON

echo "Pipeline report: $REPORT_DIR/micromani_pipeline_report.json"
