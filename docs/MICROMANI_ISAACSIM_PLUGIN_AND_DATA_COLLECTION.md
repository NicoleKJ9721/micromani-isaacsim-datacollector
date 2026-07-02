# MicroMani Isaac Sim 插件、配置与数据采集说明

更新时间：2026-07-03 03:05 CST

## 1. 项目目标

本项目把双臂微纳机器人 URDF 导入 Isaac Sim，形成可重复部署的虚拟操作数据采集环境。当前目标不是只保存一个场景文件，而是形成一套后续可迁移到专用采集电脑上的工程包：Isaac Sim extension、场景配置脚本、控制桥、LeRobot 数据转换脚本、力/力矩数据 sidecar、使用文档。

## 2. 当前功能清单

- 双臂微纳机器人 URDF / STL 资产。
- Isaac Sim 场景导入与检查。
- 14 维有效控制接口：
  - 左臂 6 轴 + 左夹爪 1 轴。
  - 右臂 6 轴 + 右夹爪 1 轴。
  - Isaac 内部仍展开为 16 个原始 DOF，左右夹爪各自的两个手指关节由一个逻辑夹爪轴同步控制。
- 三路相机：
  - 左腕相机：640x480, 30 FPS。
  - 右腕相机：640x480, 30 FPS。
  - 全局相机：640x480, 30 FPS。
- 两个 HKVL-36A 六维力/力矩传感器：
  - 左腕：`joint6_left` 中心。
  - 右腕：`joint6_right` 中心。
  - 输出：`Fx,Fy,Fz,Mx,My,Mz`。
- UDP/TCP 控制桥：
  - UDP: `49010`。
  - TCP: `49011`。
  - 外部控制包使用 14 维 `axes`。
- LeRobot v3 数据集：
  - `observation.state`: 14D。
  - `action`: 14D。
  - `observation.force_torque`: 12D。
  - 三路图像 observation。
- 力/力矩 sidecar：
  - `force_torque/episode_000000.jsonl`。
  - `force_torque/episode_000000.csv`。
  - `force_torque/metadata.json`。
- Isaac Sim extension：
  - `micromani.data_collector`。
  - 菜单入口：`Tools > MicroMani > MicroMani Data Collector`。
  - 常用按钮：打开场景、保存场景、配置相机、配置 HKVL-36、启动/停止控制桥、运行端到端烟测、打开数据目录、打开文档。

## 3. HKVL-36A 配置

当前仿真默认使用 HKVL-36A，因为微纳机器人操作更关注低力接触安全。

| 参数 | 当前配置 |
| --- | --- |
| 型号 | HKVL-36A |
| Fx/Fy/Fz 量程 | +/-30 N |
| Mx/My/Mz 量程 | +/-1 Nm |
| 直径 | 36 mm |
| 高度 | 21 mm |
| 重量 | 0.11 kg |
| 过载 | 300% FS |
| 重复精度 | 0.1% FS |
| 分辨率 | 0.03% FS |
| 精度 | 0.5% FS |
| 硬件采样频率 | 150-1000 Hz 可定制 |
| 当前数据集采样 | 30 Hz，与控制和相机同步 |
| 通讯接口 metadata | RS485 / CAN-FD / EtherCAT |

Isaac Sim 中的力/力矩读数来自：

```text
Articulation.get_measured_joint_forces(joint_index + 1)
```

该接口读取 joint6 的关节反力/力矩，输出坐标系为对应 joint6 的 child joint frame。采集脚本会按 HKVL-36A 的量程限幅，并按 `0.03% FS` 分辨率做量化。

## 4. 数据格式

LeRobot features 中包含：

```text
observation.state          float32[14]
action                     float32[14]
observation.force_torque   float32[12]
observation.images.left_wrist   image[480,640,3]
observation.images.right_wrist  image[480,640,3]
observation.images.global       image[480,640,3]
```

`observation.force_torque` 顺序固定为：

```text
left_wrist_Fx, left_wrist_Fy, left_wrist_Fz, left_wrist_Mx, left_wrist_My, left_wrist_Mz,
right_wrist_Fx, right_wrist_Fy, right_wrist_Fz, right_wrist_Mx, right_wrist_My, right_wrist_Mz
```

## 5. Isaac Sim 插件安装

推荐不复制插件文件，而是直接把仓库的 `exts` 目录作为 Isaac Sim extension search path：

```bash
/path/to/isaac-sim.sh --ext-folder /path/to/MicroManiURDF/exts --enable micromani.data_collector
```

也可以先运行：

```bash
scripts/install_isaac_extension.sh
```

它会打印推荐启动命令。打开 Isaac Sim 后，菜单入口在：

```text
Tools > MicroMani > MicroMani Data Collector
```

## 6. 插件按钮说明

- `Open Scene`：打开当前 USD 场景。
- `Save Stage`：保存当前 USD stage。
- `Configure Cameras`：重新写入三路相机外参与 metadata。
- `Configure HKVL-36`：在左右 `link6` 下创建/更新 HKVL-36 传感器 prim，并把量程、分辨率、坐标系等 metadata 写入 USD。
- `Start Bridge`：在当前 Isaac Sim 进程内启动 UDP/TCP 控制桥。
- `Stop Bridge`：请求停止由插件启动的控制桥。
- `Run E2E Smoke`：调用 `isaacsim_tools/run_micromani_lerobot_pipeline.sh`，执行短流程采集、下载、LeRobot 转换和回放验证。
- `Open Dataset Dir`：打开本地 `datasets` 目录。
- `Open Docs`：打开本文档。
- `Refresh Status`：列出常用报告文件路径。

## 7. 常用命令

本地检查：

```bash
scripts/run_local_checks.sh
```

端到端烟测：

```bash
MICROMANI_FRAMES=6 \
MICROMANI_SESSION_NAME=micromani_ft_e2e_20260703_01 \
MICROMANI_LOCAL_LEROBOT_DIR=/Users/nikejian/Projects/MicroManiURDF/datasets/lerobot/micromani_ft_e2e_20260703_01 \
MICROMANI_REPO_ID=micromani/isaac_micro_bimanual_ft \
isaacsim_tools/run_micromani_lerobot_pipeline.sh
```

云端备份：

```bash
isaacsim_tools/backup_cloud_instance.sh
```

## 8. 当前验证状态

最终已验证 session：`micromani_ft_e2e_20260703_01`。

- 采集帧数：6。
- FPS：30。
- `observation.state`：14 维。
- `action`：14 维。
- `observation.force_torque`：12 维。
- 三路相机重载后均为 `3x480x640`。
- 力/力矩 sidecar 已生成 JSONL、CSV 和 metadata。
- 左右 HKVL-36A 饱和计数均为 0。
- LeRobot action 回放到 Isaac TCP 控制桥：发送 6 帧，收到 6 个 ACK，无错误。
- 最新本地备份：`backups/cloud-isaacsim-20260703-030453`。
- Isaac Sim extension 加载验证：云端 Isaac Sim 5.1 headless 环境中已成功启用 `micromani.data_collector-0.1.0`。

## 9. 当前限制

- 当前 PICO-4 发送端还未实现。已预留 14 维 `axes` 控制包，后续 Unity/OpenXR 端可直接对接 UDP `49010` 或 TCP `49011`。
- 公网 TCP `49011` 可以建立 socket，但没有稳定返回桥接 ACK；当前端到端回放通过 SSH 本地隧道连接云端 `127.0.0.1:49011`。
- 两个腕部相机已经能采集夹爪区域，但画面偏暗且有局部遮挡。大规模数据采集前建议继续调光照、材质和相机外参。
- HKVL-36 硬件支持高于 30 Hz 的采样；当前数据集保存的是与策略控制和相机同步的 30 Hz 力觉观测。若以后做高频力控，可增加一个高频 force sidecar。
- `Run E2E Smoke` 当前优先服务“本地 Mac 控制云端 Isaac Sim”的流程。迁移到专用采集电脑时，建议新增本机 Isaac Sim pipeline，避免 SSH 依赖。
