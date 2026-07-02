# MicroMani Isaac Sim 插件功能复测报告

日期：2026-07-03

## 结论

本轮从“重新导入虚拟机器人”开始，完成了分步测试和联合链路测试。当前软件工程侧达到可交付状态：URDF 可导入、碰撞体和自由度检查通过、14 维有效控制成立、三路相机与双腕 HKVL-36A 力/力矩传感器可配置并采集、插件按钮路径可执行、PICO 网关可转发 14 维控制、LeRobot v3 数据集和力觉 sidecar 均能落盘，回放到 Isaac 控制桥收到完整 ACK。

本轮发现并修复了一个关键导入可移植性问题：URDF 中的 `package://micromanipulator_isaac/...` 在云端临时资产目录名不是 `micromanipulator_isaac` 时无法解析 mesh，导致首次导入只有地面碰撞体。已在 `isaacsim_tools/import_micromanipulator.py` 中自动解析 URDF package 名，并为临时导入目录生成 `ROS_PACKAGE_PATH` alias。

## 测试环境

- 云端实例：`ubuntu@117.50.205.12`
- Isaac Sim 容器：`isaac-sim`
- Isaac Sim 镜像：`nvcr.io/nvidia/isaac-sim:5.1.0`
- 本地项目目录：`/Users/nikejian/Projects/MicroManiURDF`
- 最新本地备份目录：由 `isaacsim_tools/backup_cloud_instance.sh` 生成到 `backups/cloud-isaacsim-*`
- 复测场景：`/isaac-sim/.local/share/ov/data/micromani/import_validation_20260703_02/micromanipulator_scene.usd`
- 复测数据 session：`micromani_import_validation_20260703_02`

## 分步测试结果

| 步骤 | 目标 | 结果 | 证据 |
| --- | --- | --- | --- |
| 1 | 先把云端项目文件备份到本地 | 通过 | `backups/cloud-isaacsim-20260703-032811` |
| 2 | 从 URDF 重新导入虚拟机器人 | 第一次失败并定位，第二次通过 | `outputs/micromani_import_validation_report_20260703.json` |
| 3 | 检查自由度、碰撞体、夹爪合并 | 通过 | 原始 DOF 16，有效控制 DOF 14，碰撞 prim 20，mimic prim 2 |
| 4 | 配置三路相机 | 通过 | 三路相机 translation error 均为 0，均为 640x480@30FPS |
| 5 | 配置双腕 HKVL-36A | 通过 | 左右 `joint6` 均写入 HKVL-36A metadata，12 维力觉采集可读 |
| 6 | 插件按钮路径 | 通过 | `Open Scene`、`Configure Cameras`、`Configure HKVL-36`、`Refresh Status`、`Start/Stop PICO Gateway` 均成功 |
| 7 | PICO 网关转发 | 通过 | 本地模拟 UDP 包转发为 14 维 `axes`，无错误 |
| 8 | 采集原始仿真操作数据 | 通过 | `datasets/raw/micromani_import_validation_20260703_02` |
| 9 | 转换 LeRobot v3 数据集 | 通过 | `datasets/lerobot/micromani_import_validation_20260703_02` |
| 10 | 回放到 Isaac 控制桥 | 通过 | 发送 8 帧，收到 8 个 ACK，错误为空 |

## 机器人导入检查

最新导入报告显示：

- Articulation root：`/micromanipulator/root_joint`
- 物理句柄有效：`true`
- Isaac 原始 DOF：`16`
- 有效控制 DOF：`14`
- 关节 prim 数量：`19`
- 碰撞 prim 数量：`20`
- mimic prim 数量：`2`
- 控制顺序：左臂 6 轴 + `left_gripper` + 右臂 6 轴 + `right_gripper`

结论：URDF 导入后机械臂主体、碰撞体和夹爪自由度结构达到目标。Isaac 内部保留两个 finger joint，外部控制层合并成一个逻辑夹爪轴，符合实际夹爪反向相对运动的控制需求。

## 插件按钮测试

新增可复用测试脚本：

```bash
/isaac-sim/python.sh /isaac-sim/.local/share/ov/data/micromani/isaacsim_tools/test_micromani_plugin_buttons.py
```

本轮报告：`outputs/micromani_plugin_button_report_20260703.json`。

关键结果：

- `ui_visible=true`
- `open_scene_status` 返回 `ok=True`
- `configure_cameras_status` 生成相机配置报告
- `configure_ft_status` 生成 HKVL-36 配置报告
- `pico_running=true`
- `pico_stopped=true`
- `ok=true`

这证明插件不是只有脚本可用，常用按钮路径也可以在 Isaac Kit 环境中执行。

## 数据采集与 LeRobot 检查

原始数据目录：

```text
datasets/raw/micromani_import_validation_20260703_02
```

LeRobot 数据目录：

```text
datasets/lerobot/micromani_import_validation_20260703_02
```

数据集检查结果：

- 帧数：`8`
- FPS：`30`
- `observation.state`：`14`
- `action`：`14`
- `observation.force_torque`：`12`
- 三路图像 shape：
  - `observation.images.left_wrist`: `3x480x640`
  - `observation.images.right_wrist`: `3x480x640`
  - `observation.images.global`: `3x480x640`
- 力数据 sidecar：
  - `force_torque/episode_000000.jsonl`
  - `force_torque/episode_000000.csv`
  - `force_torque/metadata.json`

力/力矩数据统计：

| 传感器 | 最大绝对值 `[Fx,Fy,Fz,Mx,My,Mz]` | 饱和帧数 |
| --- | --- | ---: |
| left_wrist | `[0.018, 0.612, 2.934, 0.0972, 0.1089, 0.0207]` | 0 |
| right_wrist | `[0.027, 0.549, 2.961, 0.1074, 0.1104, 0.0177]` | 0 |

图像数据统计：

| 相机 | min | max | 平均亮度 | 帧数 |
| --- | ---: | ---: | ---: | ---: |
| left_wrist | 0 | 223 | 39.24 | 8 |
| right_wrist | 0 | 222 | 52.58 | 8 |
| global | 0 | 224 | 127.65 | 8 |

图像不是空帧，且首末帧存在变化。左腕和右腕画面仍偏暗，正式采集前建议继续调光照和腕部相机外参。

## 联合闭环测试

回放报告：`outputs/micromani_import_validation_replay_report.json`。

- `frames_requested=8`
- `frames_sent=8`
- `acks=8`
- `errors=[]`
- 连接方式：本地通过隧道访问云端 Isaac TCP 控制桥

结论：LeRobot 数据中保存的 14 维 action 可被控制桥接受并回放，数据采集和控制回放闭环成立。

## PICO 通讯测试

本轮做了两层测试：

- 插件按钮启动/停止 PICO Gateway：通过。
- 本地模拟 PICO/OpenXR UDP 输入，网关转发到假控制桥：通过。

报告：`outputs/micromani_pico_gateway_local_report_20260703.json`。

关键结果：

- `received_forwarded_packet=true`
- `forwarded_axes_len=14`
- `packets_received=1`
- `packets_forwarded=1`
- `last_error=null`

当前仍未完成真实 PICO-4 头显/手柄实机联调；这是硬件条件限制，不是软件链路缺口。

## 优化点

1. 腕部相机偏暗：建议增加局部补光、提高腕部相机曝光或调整材质，正式采集前以夹爪和目标区域亮度为验收标准。
2. 长时稳定性：当前是短序列功能验证，建议下一步做 10 分钟连续 PICO 遥操作和不少于 3 个两分钟 episode。
3. 大规模视觉数据：当前烟测使用 LeRobot `image` 特征，后续大规模数据建议改成 video shard，减少小文件数量。
4. PICO 实机标定：需要拿 PICO-4 实机校准手柄坐标、按钮语义、速度倍率和网络延迟。
5. 力觉高频 sidecar：当前 30 Hz 与策略/图像同步，若做力控可额外保存高频 force-only sidecar。

## 本轮修改

- 修复 URDF `package://` mesh 解析，避免云端临时目录名变化导致碰撞体/mesh 丢失。
- 新增 `isaacsim_tools/test_micromani_plugin_buttons.py`，用于 Isaac Sim headless 中复测插件常用按钮。
- 将新增测试脚本加入 `scripts/run_local_checks.sh` 的语法检查。
- 更新 README、插件说明和质量评估文档中的最新验证状态。
