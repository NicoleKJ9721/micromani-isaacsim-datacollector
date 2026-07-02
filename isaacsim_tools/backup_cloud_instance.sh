#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLOUD_USER="ubuntu"
CLOUD_HOST="117.50.205.12"
SSH_KEY="${HOME}/.ssh/isaacsim_uyun_backup_ed25519"
REMOTE="${CLOUD_USER}@${CLOUD_HOST}"
STAMP="$(date '+%Y%m%d-%H%M%S')"
BACKUP_DIR="${PROJECT_ROOT}/backups/cloud-isaacsim-${STAMP}"
PROGRESS_DOC="${PROJECT_ROOT}/docs/cloud_isaacsim_progress.md"

SSH_OPTS=(
  -i "${SSH_KEY}"
  -o StrictHostKeyChecking=no
  -o BatchMode=yes
  -o ConnectTimeout=15
)

if [ ! -r "${SSH_KEY}" ]; then
  echo "Missing SSH key: ${SSH_KEY}" >&2
  exit 1
fi

mkdir -p "${BACKUP_DIR}/remote" "$(dirname "${PROGRESS_DOC}")"

ssh_remote() {
  ssh "${SSH_OPTS[@]}" "${REMOTE}" "$@"
}

pull_remote_path() {
  remote_path="$1"
  local_path="$2"

  if ! ssh_remote "test -e '${remote_path}'"; then
    echo "Skip missing remote path: ${remote_path}"
    return 0
  fi

  mkdir -p "$(dirname "${local_path}")"
  if command -v rsync >/dev/null 2>&1 && ssh_remote "command -v rsync >/dev/null 2>&1"; then
    rsync -az -e "ssh -i ${SSH_KEY} -o StrictHostKeyChecking=no -o BatchMode=yes" \
      "${REMOTE}:${remote_path}" "${local_path}"
  else
    scp -i "${SSH_KEY}" -o StrictHostKeyChecking=no -o BatchMode=yes -r \
      "${REMOTE}:${remote_path}" "${local_path}"
  fi
}

cat > "${BACKUP_DIR}/remote_status.txt" <<EOF
Backup time: $(date '+%Y-%m-%d %H:%M:%S %Z')
Remote: ${REMOTE}

EOF

ssh_remote 'set -e
echo "== host =="
hostname
date
uname -a
echo
echo "== gpu =="
(nvidia-smi || true)
echo
echo "== network listeners =="
(ss -lntup | egrep "49100|47998|49010|49011|8210" || true)
echo
echo "== docker =="
(sudo -n docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" || true)
echo
echo "== compose =="
(sed -n "1,240p" /home/ubuntu/docker-compose.yml || true)
echo
echo "== micromani files =="
(find /home/ubuntu/micromani /home/ubuntu/docker/isaac-sim/data/micromani -maxdepth 3 -type f 2>/dev/null | sort || true)
' >> "${BACKUP_DIR}/remote_status.txt"

pull_remote_path "/home/ubuntu/micromani/" "${BACKUP_DIR}/remote/home_ubuntu_micromani/"
pull_remote_path "/home/ubuntu/docker/isaac-sim/data/micromani/" "${BACKUP_DIR}/remote/isaac-sim-data-micromani/"
pull_remote_path "/home/ubuntu/docker-compose.yml" "${BACKUP_DIR}/remote/docker-compose.yml"

REPORT="${BACKUP_DIR}/remote/isaac-sim-data-micromani/micromanipulator_import_report.json"
BRIDGE="${BACKUP_DIR}/remote/isaac-sim-data-micromani/micromani_bridge_status.json"
FT_REPORT="${BACKUP_DIR}/remote/isaac-sim-data-micromani/hkvl36_force_torque_sensors.json"
PIPELINE_REPORT="${PROJECT_ROOT}/outputs/micromani_pipeline_report.json"
REPLAY_REPORT="${PROJECT_ROOT}/outputs/micromani_replay_report.json"
BUILD_VALIDATION_REPORT="${MICROMANI_BUILD_VALIDATION_REPORT:-${PROJECT_ROOT}/outputs/micromani_build_validation_20260703.json}"

cat > "${PROGRESS_DOC}" <<EOF
# Isaac Sim 云端开发进度

更新时间：$(date '+%Y-%m-%d %H:%M:%S %Z')

## 当前目标

把本地双臂微纳机器人 URDF 导入云端 Isaac Sim，检查自由度和碰撞体，配置 3 个相机、左右腕 HKVL-36 六维力/力矩传感器，并形成 Isaac Sim 插件、LeRobot 数据采集链路和 PICO-4/OpenXR 通讯网关。

## 已完成

- 已把本地项目上传到云端：\`/home/ubuntu/micromani/MicroManiURDF\`。
- 已生成 Isaac Sim 兼容版本：\`/home/ubuntu/micromani/micromanipulator_isaac\`。
- 已导入并生成场景：\`/home/ubuntu/docker/isaac-sim/data/micromani/micromanipulator_scene.usd\`。
- 已配置控制桥：UDP \`49010\`、TCP \`49011\`。
- 已把左右夹爪的两个手指关节合并为单个逻辑输入：\`left_gripper\` 和 \`right_gripper\`。
- 已配置 3 个相机：左腕部、右腕部、全局相机，均为 \`640x480\`、\`30 FPS\`。
- 已配置左右臂 HKVL-36A 六维力/力矩传感器，安装点为 \`joint6_left\` 和 \`joint6_right\` 中心。
- 每次 LeRobot 数据采集都会同步保存 \`observation.force_torque\` 12 维力/力矩字段，并额外导出 JSONL/CSV 力数据。
- 已创建 Isaac Sim 插件 \`micromani.data_collector-0.1.0\`，云端 Isaac Sim 5.1 headless 加载验证通过。
- 已实现 PICO-4/OpenXR Unity 发送端模板：\`pico4_unity/MicroManiPicoTeleop.cs\`。
- 已实现 PICO 网关：默认监听 UDP \`49100\`，转发到 Isaac 控制桥 UDP \`49010\` 或 TCP \`49011\`。
- 已验证插件 UI 可构建，插件 PICO 网关按钮可启动/停止网关进程。
- 已安装本地 Mac 版 Isaac Sim WebRTC Streaming Client，但当前不再强制映射画面，除非后续开发需要。

## 待完成

- PICO-4 实机还需要现场联调，验证手柄输入范围、网络延迟、坐标习惯和操作者体验。
- 云平台安全组若要让 PICO-4 从外部直连，需要放行 UDP \`49100\`；如绕过网关直连控制桥，还需要放行 UDP \`49010\` 或 TCP \`49011\`。
- 如后续需要浏览器画面，需要额外部署 Isaac 的 web viewer；当前 \`49100\` 是 WebRTC 信令端口，不是网页端口。

## 备份位置

本次云端备份目录：\`${BACKUP_DIR}\`

EOF

if command -v python3 >/dev/null 2>&1 && [ -f "${REPORT}" ]; then
  python3 - "${REPORT}" "${BRIDGE}" "${FT_REPORT}" >> "${PROGRESS_DOC}" <<'PY'
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1])
bridge_path = Path(sys.argv[2])
ft_report_path = Path(sys.argv[3])
report = json.loads(report_path.read_text())
bridge = json.loads(bridge_path.read_text()) if bridge_path.exists() else {}
ft_report = json.loads(ft_report_path.read_text()) if ft_report_path.exists() else {}

print("## 导入检查结果")
print()
print(f"- Articulation root: `{report.get('articulation_root')}`")
print(f"- 物理句柄有效：`{report.get('physics_handle_valid')}`")
print(f"- Isaac 原始 DOF：`{report.get('num_dof')}`")
print(f"- 预期有效控制 DOF：`{report.get('expected_effective_control_dofs')}`")
print(f"- 关节 prim 数量：`{report.get('joint_prim_count')}`")
print(f"- 碰撞 prim 数量：`{report.get('collision_prim_count')}`")
print(f"- mimic 关节数量：`{report.get('mimic_prim_count')}`")
print()
print("### 控制顺序")
print()
for i, name in enumerate(report.get("control_order", []), 1):
    print(f"{i}. `{name}`")
print()
print("### 相机")
print()
for camera in report.get("cameras", []):
    path = camera.get("path")
    res = camera.get("resolution")
    fps = camera.get("fps")
    fov = camera.get("target_fov_m")
    mount = camera.get("mount")
    print(f"- `{path}`：{res[0]}x{res[1]} @ {fps} FPS，目标视场 {fov} m，{mount}")
print()

if ft_report:
    print("### HKVL-36 六维力/力矩传感器")
    print()
    hardware = ft_report.get("hardware_spec", {})
    print(f"- 型号：`{ft_report.get('model')}`")
    print(f"- 力量程：`+/-{hardware.get('force_range_n')} N`")
    print(f"- 力矩量程：`+/-{hardware.get('torque_range_nm')} Nm`")
    print(f"- 分辨率：`{hardware.get('resolution_percent_fs')}% FS`")
    print(f"- 精度：`{hardware.get('accuracy_percent_fs')}% FS`")
    print(f"- 采样频率配置：`{ft_report.get('configured_sensors', [{}])[0].get('sample_fps_configured')}` FPS")
    print(f"- 云端配置报告：`/home/ubuntu/docker/isaac-sim/data/micromani/hkvl36_force_torque_sensors.json`")
    for sensor in ft_report.get("configured_sensors", []):
        print(
            f"- `{sensor.get('name')}`：`{sensor.get('sensor_prim_path')}`，"
            f"绑定 `{sensor.get('joint_name')}`，坐标系 `{sensor.get('reference_frame')}`"
        )
    print()

if bridge:
    print("## 通讯桥状态")
    print()
    print(f"- UDP 端口：`{bridge.get('udp_port')}`")
    print(f"- TCP 端口：`{bridge.get('tcp_port')}`")
    print(f"- 已接收控制包：`{bridge.get('packet_count')}`")
    print(f"- 最近来源：`{bridge.get('last_source')}`")
    print(f"- 最近错误：`{bridge.get('last_error')}`")
    print()
PY
fi

if command -v python3 >/dev/null 2>&1 && [ -f "${PIPELINE_REPORT}" ]; then
  python3 - "${PIPELINE_REPORT}" "${REPLAY_REPORT}" >> "${PROGRESS_DOC}" <<'PY'
import json
import sys
from pathlib import Path

pipeline_path = Path(sys.argv[1])
replay_path = Path(sys.argv[2])
pipeline = json.loads(pipeline_path.read_text())
replay = json.loads(replay_path.read_text()) if replay_path.exists() else {}
dataset_root = Path(pipeline.get("lerobot_dataset", ""))
validation_path = dataset_root / "micromani_validation.json"
validation = json.loads(validation_path.read_text()) if validation_path.exists() else {}
raw_session = Path(pipeline.get("raw_session", ""))
manifest_path = raw_session / "manifest.json"
manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}

print("## LeRobot 数据采集闭环")
print()
print("- 已参照 LeRobot 的记录流程完成虚拟微纳机器人数据采集链路：读取观测、生成动作、发送动作、保存观测/动作帧、转换成 LeRobot v3 数据集、再回放到 Isaac Sim。")
print("- 当前数据结构包含视觉、14 维状态、14 维动作和 12 维左右腕 HKVL-36 力/力矩。")
print(f"- 最终联合测试 session：`{pipeline.get('session_name')}`。")
print(f"- 原始数据：`{pipeline.get('raw_session')}`。")
print(f"- LeRobot 数据集：`{pipeline.get('lerobot_dataset')}`。")
print("- 验证结果：")
print(f"  - 采集帧数：`{pipeline.get('frames')}`")
print(f"  - FPS：`{pipeline.get('fps')}`")
print(f"  - `observation.state`：`{validation.get('sample_state_shape', ['?'])[0]}` 维")
print(f"  - `action`：`{validation.get('sample_action_shape', ['?'])[0]}` 维")
force_shape = validation.get("sample_force_torque_shape")
if force_shape:
    print(f"  - `observation.force_torque`：`{force_shape[0]}` 维")
if validation.get("camera_shapes"):
    print("  - 三路相机重载后均为 `3x480x640`")
sidecars = validation.get("force_torque_sidecars")
if sidecars:
    print(f"  - 力/力矩 sidecar：`{sidecars.get('jsonl')}`、`{sidecars.get('csv')}`、`{sidecars.get('metadata')}`")
stats = manifest.get("force_torque_stats", {})
if stats:
    left_sat = stats.get("left_wrist", {}).get("saturated_count")
    right_sat = stats.get("right_wrist", {}).get("saturated_count")
    print(f"  - 左右 HKVL-36A 传感器饱和计数：`left={left_sat}, right={right_sat}`")
if replay:
    print(f"  - 回放到 Isaac TCP 控制桥：发送 `{replay.get('frames_sent')}` 帧，收到 `{replay.get('acks')}` 个 ACK，错误 `{replay.get('errors')}`")
print()
print("### 当前限制")
print()
print("- 云实例本机 `127.0.0.1:49011` TCP 控制桥可稳定返回 ACK；公网 `117.50.205.12:49011` 可以建立 socket，但没有返回桥接 ACK。因此当前回放流程使用 SSH 本地隧道转发到云端本机端口。")
print("- PICO-4 Unity/OpenXR 发送端和 PICO 网关已经实现并通过模拟输入/插件按钮烟测；但 PICO-4 头显和手柄实机尚未联调。")
print("- 两个腕部相机已经挂在 `link6_left` 和 `link6_right` 下，并能采集夹爪区域；但当前画面偏暗且有局部遮挡。大规模采集前建议继续调灯光、材质和腕部相机外参。")
print("- 力/力矩流当前与控制/相机同步为 30 Hz；HKVL-36 硬件支持更高采样，但当前数据集保存的是策略可直接使用的同步力觉观测。")
print()
print("## Isaac Sim 插件")
print()
print("- 插件目录：`/Users/nikejian/Projects/MicroManiURDF/exts/micromani.data_collector`。")
print("- 云端同步目录：`/home/ubuntu/docker/isaac-sim/data/micromani/exts/micromani.data_collector`。")
print("- 启动方式：`isaac-sim.sh --ext-folder /path/to/MicroManiURDF/exts --enable micromani.data_collector`。")
print("- 菜单入口：`Tools > MicroMani > MicroMani Data Collector`。")
print("- 常用按钮：打开场景、保存场景、配置相机、配置 HKVL-36、启动/停止控制桥、启动/停止 PICO 网关、运行端到端烟测、打开数据目录、打开文档。")
print("- 验证状态：云端 Isaac Sim 5.1 headless 环境中成功启用/关闭 `micromani.data_collector-0.1.0`，UI 构建通过，PICO 网关按钮启动/停止通过。")
print()
PY
fi

if command -v python3 >/dev/null 2>&1 && [ -f "${BUILD_VALIDATION_REPORT}" ]; then
  python3 - "${BUILD_VALIDATION_REPORT}" >> "${PROGRESS_DOC}" <<'PY'
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text())
checks = report.get("checks", {})
articulation = report.get("articulation", {})
stage_summary = report.get("stage_summary", {})
force = report.get("force_torque_runtime_sample", {})
cameras = report.get("camera_runtime_sample", {})
grippers = report.get("gripper_command_validation", {})

print("## 当前主场景构建验收")
print()
print(f"- 验收报告：`outputs/micromani_build_validation_20260703.json`")
print(f"- `build_success`：`{report.get('build_success')}`")
print(f"- 硬性检查全部通过：`{all(checks.values()) if checks else False}`")
print(f"- Articulation 物理句柄：`{articulation.get('physics_handle_valid')}`")
print(f"- 原始 DOF：`{articulation.get('num_dof')}`")
print(f"- 有效控制 DOF：`{articulation.get('expected_effective_control_dofs')}`")
print(f"- 碰撞 prim：`{stage_summary.get('collision_prim_count')}`")
print(f"- mesh prim：`{stage_summary.get('mesh_prim_count')}`")
print(f"- HKVL-36 运行时力/力矩维度：`{force.get('vector_len')}`，有限值：`{force.get('finite')}`，饱和：`{force.get('saturated_any')}`")
for name, item in grippers.items():
    print(f"- `{name}` 夹爪运行误差：`{item.get('max_abs_error')}` m，收敛步数 `{item.get('settle_frames')}`")
for name, item in cameras.items():
    print(f"- `{name}` 相机：shape `{item.get('shape')}`，非空 `{item.get('non_empty')}`，mean `{item.get('mean')}`")
print()
PY
fi

cp "${PROGRESS_DOC}" "${BACKUP_DIR}/cloud_isaacsim_progress.md"
ln -sfn "${BACKUP_DIR}" "${PROJECT_ROOT}/backups/latest-cloud-isaacsim"

echo "Backup complete: ${BACKUP_DIR}"
echo "Progress doc: ${PROGRESS_DOC}"
