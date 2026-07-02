# Isaac Sim 云端开发进度

更新时间：2026-07-03 03:40:27 CST

## 当前目标

把本地双臂微纳机器人 URDF 导入云端 Isaac Sim，检查自由度和碰撞体，配置 3 个相机、左右腕 HKVL-36 六维力/力矩传感器，并形成 Isaac Sim 插件、LeRobot 数据采集链路和 PICO-4/OpenXR 通讯网关。

## 已完成

- 已把本地项目上传到云端：`/home/ubuntu/micromani/MicroManiURDF`。
- 已生成 Isaac Sim 兼容版本：`/home/ubuntu/micromani/micromanipulator_isaac`。
- 已导入并生成场景：`/home/ubuntu/docker/isaac-sim/data/micromani/micromanipulator_scene.usd`。
- 已配置控制桥：UDP `49010`、TCP `49011`。
- 已把左右夹爪的两个手指关节合并为单个逻辑输入：`left_gripper` 和 `right_gripper`。
- 已配置 3 个相机：左腕部、右腕部、全局相机，均为 `640x480`、`30 FPS`。
- 已配置左右臂 HKVL-36A 六维力/力矩传感器，安装点为 `joint6_left` 和 `joint6_right` 中心。
- 每次 LeRobot 数据采集都会同步保存 `observation.force_torque` 12 维力/力矩字段，并额外导出 JSONL/CSV 力数据。
- 已创建 Isaac Sim 插件 `micromani.data_collector-0.1.0`，云端 Isaac Sim 5.1 headless 加载验证通过。
- 已实现 PICO-4/OpenXR Unity 发送端模板：`pico4_unity/MicroManiPicoTeleop.cs`。
- 已实现 PICO 网关：默认监听 UDP `49100`，转发到 Isaac 控制桥 UDP `49010` 或 TCP `49011`。
- 已验证插件 UI 可构建，插件 PICO 网关按钮可启动/停止网关进程。
- 已安装本地 Mac 版 Isaac Sim WebRTC Streaming Client，但当前不再强制映射画面，除非后续开发需要。

## 待完成

- PICO-4 实机还需要现场联调，验证手柄输入范围、网络延迟、坐标习惯和操作者体验。
- 云平台安全组若要让 PICO-4 从外部直连，需要放行 UDP `49100`；如绕过网关直连控制桥，还需要放行 UDP `49010` 或 TCP `49011`。
- 如后续需要浏览器画面，需要额外部署 Isaac 的 web viewer；当前 `49100` 是 WebRTC 信令端口，不是网页端口。

## 备份位置

本次云端备份目录：`/Users/nikejian/Projects/MicroManiURDF/backups/cloud-isaacsim-20260703-034013`

## 导入检查结果

- Articulation root: `/micromanipulator/root_joint`
- 物理句柄有效：`True`
- Isaac 原始 DOF：`16`
- 预期有效控制 DOF：`14`
- 关节 prim 数量：`19`
- 碰撞 prim 数量：`20`
- mimic 关节数量：`2`

### 控制顺序

1. `joint1_left`
2. `joint2_left`
3. `joint3_left`
4. `joint4_left`
5. `joint5_left`
6. `joint6_left`
7. `left_gripper`
8. `joint1_right`
9. `joint2_right`
10. `joint3_right`
11. `joint4_right`
12. `joint5_right`
13. `joint6_right`
14. `right_gripper`

### 相机

- `/micromanipulator/link6_left/left_wrist_camera`：640x480 @ 30 FPS，目标视场 [0.04, 0.04] m，child of link6_left, 45deg downward toward left gripper
- `/micromanipulator/link6_right/right_wrist_camera`：640x480 @ 30 FPS，目标视场 [0.04, 0.04] m，child of link6_right, 45deg downward toward right gripper
- `/World/global_camera`：640x480 @ 30 FPS，目标视场 [0.15, 0.15] m，world camera above both arms, oblique downward

### HKVL-36 六维力/力矩传感器

- 型号：`HKVL-36A`
- 力量程：`+/-30.0 N`
- 力矩量程：`+/-1.0 Nm`
- 分辨率：`0.03% FS`
- 精度：`0.5% FS`
- 采样频率配置：`30` FPS
- 云端配置报告：`/home/ubuntu/docker/isaac-sim/data/micromani/hkvl36_force_torque_sensors.json`
- `left_wrist`：`/micromanipulator/link6_left/HKVL36_FT_sensor`，绑定 `joint6_left`，坐标系 `child joint frame of joint6_left`
- `right_wrist`：`/micromanipulator/link6_right/HKVL36_FT_sensor`，绑定 `joint6_right`，坐标系 `child joint frame of joint6_right`

## 通讯桥状态

- UDP 端口：`49010`
- TCP 端口：`49011`
- 已接收控制包：`44`
- 最近来源：`tcp:127.0.0.1:43250`
- 最近错误：`None`

## LeRobot 数据采集闭环

- 已参照 LeRobot 的记录流程完成虚拟微纳机器人数据采集链路：读取观测、生成动作、发送动作、保存观测/动作帧、转换成 LeRobot v3 数据集、再回放到 Isaac Sim。
- 当前数据结构包含视觉、14 维状态、14 维动作和 12 维左右腕 HKVL-36 力/力矩。
- 最终联合测试 session：`micromani_import_validation_20260703_02`。
- 原始数据：`/Users/nikejian/Projects/MicroManiURDF/datasets/raw/micromani_import_validation_20260703_02`。
- LeRobot 数据集：`/Users/nikejian/Projects/MicroManiURDF/datasets/lerobot/micromani_import_validation_20260703_02`。
- 验证结果：
  - 采集帧数：`8`
  - FPS：`30`
  - `observation.state`：`14` 维
  - `action`：`14` 维
  - `observation.force_torque`：`12` 维
  - 三路相机重载后均为 `3x480x640`
  - 力/力矩 sidecar：`/Users/nikejian/Projects/MicroManiURDF/datasets/lerobot/micromani_import_validation_20260703_02/force_torque/episode_000000.jsonl`、`/Users/nikejian/Projects/MicroManiURDF/datasets/lerobot/micromani_import_validation_20260703_02/force_torque/episode_000000.csv`、`/Users/nikejian/Projects/MicroManiURDF/datasets/lerobot/micromani_import_validation_20260703_02/force_torque/metadata.json`
  - 左右 HKVL-36A 传感器饱和计数：`left=0, right=0`
  - 回放到 Isaac TCP 控制桥：发送 `8` 帧，收到 `8` 个 ACK，错误 `[]`

### 当前限制

- 云实例本机 `127.0.0.1:49011` TCP 控制桥可稳定返回 ACK；公网 `117.50.205.12:49011` 可以建立 socket，但没有返回桥接 ACK。因此当前回放流程使用 SSH 本地隧道转发到云端本机端口。
- PICO-4 Unity/OpenXR 发送端和 PICO 网关已经实现并通过模拟输入/插件按钮烟测；但 PICO-4 头显和手柄实机尚未联调。
- 两个腕部相机已经挂在 `link6_left` 和 `link6_right` 下，并能采集夹爪区域；但当前画面偏暗且有局部遮挡。大规模采集前建议继续调灯光、材质和腕部相机外参。
- 力/力矩流当前与控制/相机同步为 30 Hz；HKVL-36 硬件支持更高采样，但当前数据集保存的是策略可直接使用的同步力觉观测。

## Isaac Sim 插件

- 插件目录：`/Users/nikejian/Projects/MicroManiURDF/exts/micromani.data_collector`。
- 云端同步目录：`/home/ubuntu/docker/isaac-sim/data/micromani/exts/micromani.data_collector`。
- 启动方式：`isaac-sim.sh --ext-folder /path/to/MicroManiURDF/exts --enable micromani.data_collector`。
- 菜单入口：`Tools > MicroMani > MicroMani Data Collector`。
- 常用按钮：打开场景、保存场景、配置相机、配置 HKVL-36、启动/停止控制桥、启动/停止 PICO 网关、运行端到端烟测、打开数据目录、打开文档。
- 验证状态：云端 Isaac Sim 5.1 headless 环境中成功启用/关闭 `micromani.data_collector-0.1.0`，UI 构建通过，PICO 网关按钮启动/停止通过。

