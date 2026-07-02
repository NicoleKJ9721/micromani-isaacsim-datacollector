# MicroMani Isaac Sim 插件、配置与数据采集说明

更新时间：2026-07-03

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
  - 常用按钮：打开场景、保存场景、配置相机、配置 HKVL-36、启动/停止控制桥、启动/停止 PICO 网关、运行端到端烟测、打开数据目录、打开文档。
- PICO-4 通讯预留与网关：
  - PICO/Unity/OpenXR 端示例脚本：`pico4_unity/MicroManiPicoTeleop.cs`。
  - 本机/云端网关：`isaacsim_tools/micromani_pico_gateway.py`。
  - 默认链路：PICO-4 Unity app -> UDP `49100` -> PICO gateway -> UDP `49010` 或 TCP `49011` -> Isaac Sim 控制桥。

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

当前转换脚本调用 `LeRobotDataset.create(...)` 生成本地 LeRobot v3 数据集。已验证数据集的 `meta/info.json` 中 `codebase_version` 为 `v3.0`，低维信号保存为 Parquet。当前烟测使用 `image` 特征，适合小规模闭环验证；大规模采集时建议使用 `--image-dtype video`，让视觉数据按 MP4 shard 存储，减少文件数量。

## 5. 夹爪自由度合并逻辑

URDF 内部左右夹爪各有两个 prismatic finger joint：

```text
left_gripper  -> joint_finger1_left + joint_finger2_left
right_gripper -> joint_finger1_right + joint_finger2_right
```

控制接口只暴露左右各 1 个逻辑夹爪轴。两个 finger joint 接收同一个标量，但 URDF 中两个 finger joint 的运动轴方向相反，因此物理运动表现为相对开合。这一点已经加入本地验收脚本，避免后续误改成两个独立夹爪自由度。

## 6. Isaac Sim 插件安装

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

## 7. 插件按钮说明

- `Open Scene`：打开当前 USD 场景。
- `Save Stage`：保存当前 USD stage。
- `Configure Cameras`：重新写入三路相机外参与 metadata。
- `Configure HKVL-36`：在左右 `link6` 下创建/更新 HKVL-36 传感器 prim，并把量程、分辨率、坐标系等 metadata 写入 USD。
- `Start Bridge`：在当前 Isaac Sim 进程内启动 UDP/TCP 控制桥。
- `Stop Bridge`：请求停止由插件启动的控制桥。
- `Start PICO Gateway`：启动 PICO/OpenXR UDP 网关，默认监听 `49100/UDP`，并转发到 Isaac 控制桥。
- `Stop PICO Gateway`：停止插件启动的 PICO 网关。
- `Run E2E Smoke`：调用 `isaacsim_tools/run_micromani_lerobot_pipeline.sh`，执行短流程采集、下载、LeRobot 转换和回放验证。
- `Open Dataset Dir`：打开本地 `datasets` 目录。
- `Open Docs`：打开本文档。
- `Refresh Status`：列出常用报告文件路径。

## 8. PICO-4 通讯使用

先在 Isaac Sim 插件中点击：

```text
Start Bridge
Start PICO Gateway
```

或者命令行运行：

```bash
python3 isaacsim_tools/micromani_pico_gateway.py \
  --listen-host 0.0.0.0 \
  --listen-port 49100 \
  --bridge-host 127.0.0.1 \
  --bridge-port 49010 \
  --transport udp
```

Unity/PICO-4 端把 `pico4_unity/MicroManiPicoTeleop.cs` 加到场景中的 GameObject 上，并把 `gatewayHost` 设置为 Isaac Sim 所在机器的 IP。默认 30 Hz 发送左右手柄摇杆、trigger、grip 和常用按钮。

当前默认按键约定：

- primary button：复位到 home action。
- left secondary button：打开左右夹爪。
- right secondary button：闭合左右夹爪。
- 左/右摇杆和 trigger/grip：增量控制对应机械臂的平移、旋转和夹爪。

## 9. 常用命令

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

项目级本地验收：

```bash
scripts/run_local_checks.sh
```

该脚本会检查 Python/Shell 语法、14 维控制 schema、URDF 夹爪相对运动、HKVL-36 参数、插件按钮测试脚本、PICO-4 网关文件、文档覆盖项和已有 LeRobot v3 数据集结构。

## 10. 当前验证状态

最新从 URDF 重新导入开始的全链路复测 session：`micromani_import_validation_20260703_02`。

- 采集帧数：8。
- FPS：30。
- `observation.state`：14 维。
- `action`：14 维。
- `observation.force_torque`：12 维。
- 三路相机重载后均为 `3x480x640`。
- 力/力矩 sidecar 已生成 JSONL、CSV 和 metadata。
- 左右 HKVL-36A 饱和计数均为 0。
- LeRobot action 回放到 Isaac TCP 控制桥：发送 8 帧，收到 8 个 ACK，无错误。
- 最新导入报告：`outputs/micromani_import_validation_report_20260703.json`。
- 最新插件按钮报告：`outputs/micromani_plugin_button_report_20260703.json`。
- 最新本地备份：`backups/cloud-isaacsim-20260703-034013`。
- Isaac Sim extension 加载验证：云端 Isaac Sim 5.1 headless 环境中已成功启用并关闭 `micromani.data_collector-0.1.0`。
- Isaac Sim extension UI 验证：云端 headless 环境中直接构建插件窗口，`visible=True`，状态为 `Ready.`。
- PICO 网关按钮验证：云端 headless 环境中通过插件方法启动 PICO gateway，进程 `running=True`；停止后 `stopped=True`。
- 本地 LeRobot 重载验证：`LeRobotDataset(..., return_uint8=True)` 可加载 8 帧数据，三路图像 shape 均为 `3x480x640`，力/力矩 shape 为 `12`。
- PICO-4 网关：本地模拟 UDP 输入已验证能转发 14 维 `axes`；插件按钮已验证可启动/停止网关；PICO-4 实机输入尚未验证。
- 详细复测报告：`docs/FUNCTIONAL_TEST_REPORT_20260703.md`。

## 11. 当前限制

- 当前 PICO-4 Unity/OpenXR 发送端和网关已经实现为工程模板，但没有 PICO-4 头显/手柄实机在本轮环境中可供联调。因此“协议和代码已具备”，但“PICO-4 实机可操作性”仍需拿设备验证。
- 公网 TCP `49011` 可以建立 socket，但没有稳定返回桥接 ACK；当前端到端回放通过 SSH 本地隧道连接云端 `127.0.0.1:49011`。
- 两个腕部相机已经能采集夹爪区域，数据不是空帧；当前画面偏暗且有局部遮挡。大规模数据采集前建议继续调光照、材质和相机外参。
- HKVL-36 硬件支持高于 30 Hz 的采样；当前数据集保存的是与策略控制和相机同步的 30 Hz 力觉观测。若以后做高频力控，可增加一个高频 force sidecar。
- `Run E2E Smoke` 当前优先服务“本地 Mac 控制云端 Isaac Sim”的流程。迁移到专用采集电脑时，建议新增本机 Isaac Sim pipeline，避免 SSH 依赖。
