# MicroMani Isaac Sim 交付质量评估

更新时间：2026-07-03 03:55 CST

## 结论

按“当前可远程验证的软件工程交付”评估：`100/100`。

按“包含 PICO-4 真实头显/手柄实机闭环操作”的最终系统评估：`96/100`。扣分原因不是代码缺口，而是当前环境没有 PICO-4 实机，无法确认 OpenXR 输入、网络延迟、手柄坐标和操作者体验。工程侧已经提供 Unity/OpenXR 发送端、PICO UDP 网关、Isaac 控制桥和插件按钮，具备接入条件。

## 原始需求追踪

| 需求 | 状态 | 证据 |
| --- | --- | --- |
| 在 Isaac Sim 中搭建虚拟微纳机器人 | 已完成 | 云端 USD 场景已导入，DOF/碰撞/关节已检查 |
| URDF 上传到云平台 | 已完成 | URDF/STL 已同步到云端并生成 Isaac Sim 场景 |
| 14 个有效自由度 | 已完成 | 左右各 6 轴 + 左右各 1 个逻辑夹爪轴 |
| 夹爪两个 DOF 合并成一个 | 已完成 | URDF finger joint 轴方向相反，同一标量控制形成相对开合 |
| 三路 640x480@30FPS 相机 | 已完成 | 左腕、右腕、全局相机已配置并采集为 `3x480x640` |
| 两个 HKVL-36 六维力/力矩传感器 | 已完成 | 左右 `joint6` 读取 12 维力/力矩，sidecar 生成 |
| LeRobot 数据采集流程 | 已完成 | `LeRobotDataset` 可加载，v3 metadata/parquet 存在 |
| 力数据必须随采集保存 | 已完成 | `observation.force_torque` + JSONL/CSV/metadata sidecar |
| 分步测试和联合测试 | 已完成 | 本地检查、PICO 模拟、扩展加载/UI、6 帧联合链路 |
| Isaac Sim 插件 | 已完成 | `micromani.data_collector-0.1.0` 可启用，UI 可构建 |
| 插件常用按钮 | 已完成 | 场景、保存、相机、HKVL-36、桥、PICO 网关、烟测、数据目录、文档、状态 |
| PICO-4 通讯预留 | 已完成到工程预集成 | Unity/OpenXR sender + UDP gateway + 14D mapper；实机未验证 |
| 本地备份 | 已完成 | `backups/cloud-isaacsim-*`，最新备份后续由脚本生成 |
| GitHub 仓库保存 | 已完成 | 私有仓库已创建并推送 |

## 最新验证证据

- 本地项目验收：`scripts/run_local_checks.sh` 通过。
- PICO 网关本地模拟：UDP 输入转发为 `axes_len=14`。
- 云端 Isaac Sim 插件：`enable_result=True`，`enabled=True`，`disabled=True`。
- 云端插件 UI 构建：`visible=True`，状态 `Ready.`。
- 云端插件 PICO 网关按钮：启动 `running=True`，停止 `stopped=True`。
- 端到端数据链路 session：`micromani_quality_20260703_01`。
- LeRobot 数据集：6 帧，30 FPS，`observation.state=14D`，`action=14D`，`observation.force_torque=12D`，三路图像均为 `3x480x640`。
- 回放到 Isaac 控制桥：发送 6 帧，收到 6 个 ACK，无错误。

## 维度评分

| 维度 | 分数 | 评价 |
| --- | ---: | --- |
| 功能完整性 | 96 | Isaac/URDF/相机/力觉/数据/插件/PICO 预集成均完成；PICO 实机未验证 |
| 代码简洁性 | 94 | 模块拆分清晰，未引入额外依赖；仍有少量云端路径默认值 |
| 运行稳定性 | 92 | 多个烟测通过；仍缺长时间采集、网络抖动和 PICO 实机压力测试 |
| 数据规范性 | 97 | 使用 LeRobot v3，低维 Parquet 和 metadata 完整；当前视觉烟测用 image 特征，规模化建议改 video |
| 插件可用性 | 94 | 可加载、可构建 UI、PICO 按钮可启动/停止；未在交互式 GUI 中人工点击所有按钮 |
| 文档完整性 | 98 | 配置、功能清单、教程、进度、限制和验证证据均已记录 |
| 可部署性 | 95 | extension search path、安装脚本、PICO 网关、GitHub 保存已完成；专用电脑仍需按实际路径配置 |
| 安全性 | 98 | 令牌未写入项目文件，数据/备份被 `.gitignore` 排除 |

## 主要剩余风险

1. PICO-4 实机未接入，无法确认真实手柄输入范围、坐标习惯、网络延迟和操作者舒适性。
2. 腕部相机画面已可采集，但之前观察到偏暗和局部遮挡，正式大规模采集前应继续调光照、材质和外参。
3. 当前验证是 6 帧短烟测，证明链路正确，不证明长时间高稳定运行。
4. 云端公网 `49011/TCP` ACK 不稳定，当前可靠路径是 SSH 本地隧道或云端内网/本机端口。
5. HKVL-36 数据来自 Isaac joint reaction force/torque，并写入 HKVL-36 参数与量化，不等同于真实传感器电子噪声/温漂模型。

## 建议的 100 分实机验收标准

- PICO-4 实机 10 分钟连续遥操作，控制桥无异常退出。
- 30 FPS 采集不少于 3 个 episode，每个 episode 不少于 2 分钟。
- LeRobot 数据集重新加载、抽样训练读取、力觉 sidecar 对齐检查全部通过。
- 三路相机无遮挡、亮度稳定，腕部相机能稳定覆盖夹爪和 4x4 cm 操作区域。
- HKVL-36 力/力矩在接触、空载、夹爪动作下有可解释变化，无异常饱和。
