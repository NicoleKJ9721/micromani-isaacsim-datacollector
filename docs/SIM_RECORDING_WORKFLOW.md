# 本地仿真采集与 AppStation 数据规范适配

本流程参考 E2EAPP_MicroMani `3efb583` 的 episode 生命周期、数值契约、同帧对齐与可追溯要求。Isaac Sim 内部使用模型侧 SI 坐标；训练数据使用操作者侧 μm、mdeg、夹口间距 mm。数据来源始终标记 `isaac_sim`。

## 数据契约

新会话使用 `micromani.sim.operator_sides.v3`，参照实机的 `appstation.dual_arm.operator_sides.v2`。v3 将模型旋转字段按 RX、RZ、RY 标注，保持原有数值顺序。v1 继续支持续录、导出和回放。两者具有相似字段和单位，但仿真无真实编码器脉冲、硬件去皮校准与实测电子噪声。`hardwareCompatible=false` 防止仅凭 14 维 shape 宣称兼容；实机软件需要显式增加仿真导入适配才能接受本数据。

2026-11-07 用户确认：模型左右与硬件左右一致，操作者左对应模型右；双侧夹爪零位间距均为 0 mm。已写入 `config/simulation_data_contract.json`。模板保留空值，未确认侧序或零位时拒绝创建契约。

| 内容 | 仿真输入 | 导出数据 |
|---|---|---|
| 平移 | 关节 m | 相对于配置零位的 μm |
| 旋转 | 关节 rad | 关节角 mdeg（不是末端欧拉角） |
| 夹爪 | 两个手指位移的平均值 m | 零位间距 + 开合方向 × 平均位移 × 2000，mm |
| 侧序 | 模型左、模型右 | 操作者左（模型右）、操作者右（模型左） |
| 力 | 两腕关节反力，N / N·m | `observation.force_left/right`，各 6D，按操作者侧序 |
| 相机 | 模型安装角色 | `observation.images.global/wrist_left/wrist_right`；不随数值侧序交换 |
| 状态／动作 | 各 14D | `observation.state` / `action`，各 14D |

夹口间距由两个反向手指关节得到，当前最大 20 mm。原始帧及导出溯源保留四指接触力和每爪双指接触标志，顺序为模型左后模型右。不要复制实机工作站的 26 mm 起始夹口要求。`motion_sign` 默认沿 URDF 正方向；`motion_zero_si` 默认 URDF 零位。迁移到其他零位、模型或实机时，先重新标定，再创建新数据集。

力来源为 Isaac 关节反力，使用 joint metadata 的索引加 1，不能使用 DOF 索引。读数按现有 HKVL-36A 范围量化；不提供真实传感器校准或电子噪声保证。力保持模型 joint6 child frame，未声称与实机传感器坐标标定一致。

## 本地启动

先导入 URDF 并配置三路相机，生成 USD。新录制器要求现有场景已经配置完成；不连接历史云端服务。

```bash
cd /path/to/MicroManiURDF
export ISAACSIM_PYTHON=/home/nkj/isaacsim-5.1.0/python.sh
export MICROMANI_STAGE=/absolute/path/micromanipulator_scene.usd
export MICROMANI_SESSION_NAME=demo_001
export MICROMANI_TASK='Pick and place micro component'
bash isaacsim_tools/run_micromani_local.sh record
```

录制器独占默认 UDP `127.0.0.1:49010`。不要同时启动旧控制桥占用该端口。PICO gateway 可以继续向该地址发送 `{"axes": [14 个模型侧 SI 绝对目标]}`。跨机器接收时显式传入 `--host`；不要把实机单位的数据直接发送到这个 UDP 接口。

终端命令：

- `start`：开始一个新 episode。
- `save success` / `save failure` / `save unlabeled`：停止该段写入并保存结果标签。
- `discard`：将当前段标记为丢弃；保留原始文件用于追溯，导出时排除。
- `reset`：在未录制时将模型设置为 home 目标。不会自动假定工件已复位。
- `finish`：结束会话；未保存／丢弃的段必须先处理。

首段及 reset 后由操作者检查相机、夹口和场景状态再 `start`。没有新 UDP 指令时保持最近的绝对目标，并显式记录 `action_held` 和原指令时间；这是位置保持，不是假造新的人类动作。`initial_home_hold`、`manual_reset` 和 `synthetic_smoke` 与 UDP 操作来源分别记录。

关闭窗口、Ctrl+C、终端 EOF 或采集异常将当前段标记为 interrupted。使用相同参数及 `MICROMANI_RAW_SESSION=/absolute/path/session` 加 `--resume` 重新打开后，先 `discard` 中断段再继续。续录要求任务、FPS、主 USD 路径及哈希、完整数据契约一致；索引不重用。主 USD 哈希不覆盖外部引用资产，外部资产变化应另建数据集。

## 仿真时钟与质量门控

新流程使用每步 `1/fps` 的物理与渲染步长。先读同一仿真步的状态、力和三相机，再记录即将施加的绝对目标，最后执行动作并推进一步。

采集前仅刷新渲染、不推进物理，最多等待 12 次刷新，让三路相机原始 rendering_time 与当前物理时间一致。超时即失败；不再使用常量偏移调整旧帧时间。后续要求同速、同一步，相机时间必须推进。缺图、无效数值、不同步、时钟跳步／倒退和越界动作不能静默补零、补旧帧。相机异常会中断 episode。不能将名义 30 FPS 理解为墙钟性能保证；同步写盘可能降低运行速度，但不改变仿真时间间隔。

写入在单个仿真循环中串行执行，保存发生在前一帧完整写入之后；无需复制实机异步采样线程。中断段不允许导出。每段保存结果标签和契约快照；原始帧保存仿真时间、各源时间和动作来源。

## 导出与验证

使用已安装 LeRobot 的 Python：

```bash
export LEROBOT_PYTHON=/path/to/lerobot/python
"$LEROBOT_PYTHON" isaacsim_tools/micromani_convert_to_lerobot.py \
  --raw-session datasets/raw/demo_001 \
  --output-root datasets/lerobot/demo_001 \
  --image-dtype image
```

也支持 `--image-dtype video`，需要该 LeRobot 环境的编码依赖。导出包含全部 saved episodes，排除 discarded；原始 episode 索引与 LeRobot 连续索引的映射保存在 `meta/micromani_info.json`。逐帧来源记录在 `provenance/`。契约同时写入 `meta/info.json` 和旁路元数据；部分 LeRobot 版本忽略未知扩展字段，消费者必须显式读取契约。

导出前验证连续帧、时间、有限值、动作范围、图像尺寸及 RGB 模式。导出到临时目录后重新加载 LeRobot，验证动作与三相机 shape，再发布结果。`--overwrite` 成功时保留旧输出为 `.backup-*`；验证失败不会替换原输出。

## 回放

新契约在回放前转换回模型侧 m/rad/单指 m，并验证全部动作。旧无契约 SI 数据必须显式加 `--allow-legacy-model-si`；未知契约始终拒绝。

```bash
"$LEROBOT_PYTHON" isaacsim_tools/micromani_replay_lerobot.py \
  --dataset-root datasets/lerobot/demo_001 --host 127.0.0.1 \
  --transport udp --port 49010
```

UDP 用于新录制器，不提供 ACK；TCP 用于旧控制桥，ACK 只表示进入队列，不证明物理执行完成。正式数据不得把回放误标为真人示教。

## 验证入口

```bash
LEROBOT_PYTHON=/path/to/lerobot/python bash scripts/run_sim_checks.sh
bash scripts/run_local_checks.sh
# 真实 Isaac 仿真短测试，固定 home 目标，非示教数据：
LEROBOT_PYTHON=/path/to/lerobot/python bash isaacsim_tools/run_micromani_local.sh smoke
```

旧 `micromani_collect_isaac.py` 和云端流水线保留作为 legacy 原格式测试。Isaac 插件现有 `Run E2E Smoke` 按钮仍调用旧云端流程；本地新流程使用上述终端入口，不从该按钮启动。

## 2026-11-09 本机修复

测试脚本默认使用项目 `.venv-lerobot/bin/python`；可用 `LEROBOT_PYTHON` 覆盖。
依赖在独立环境中安装：`uv pip install --python .venv-lerobot/bin/python --torch-backend cpu -r requirements-sim.txt`。
CPU PyTorch 用于数据导出；Isaac Sim 保持其自带 GPU Python 环境。

`scripts/open_simulation.sh` 打开可见窗口，完成 60 帧合成 home 保持录制后继续展示场景。关闭窗口结束。
真实录制使用原 `record` 入口；合成验证不是人类示教。

重复配置会复用灯光/标记变换，并显式设置夹指凸包碰撞近似。该近似不是精密接触几何标定。

## 初始姿态防干涉（2026-11-09）

模型侧初始目标：左 [0.002, 0.005, 0.034, 0, 0, 0, 0.004]；右 [0.002, 0.045, 0.034, 0, 0, 0, 0.004]，单位 m/rad/单指 m。
两侧回撤并前后错开 40 mm；不改变关节限位、CAD 网格或左右侧定义。
CAD 回归检查要求左右臂所有链接的世界 AABB 距离至少 5 mm，左右夹指至少 10 mm；本次夹指下界约 17 mm。
场景构建同时检查运行时夹指包围盒，并保存 home 链接姿态和驱动目标，避免仅改录制时的状态。
此验证覆盖初始姿态，不保证任意外部动作全程无碰撞。
外观随后依据更新后的 PLATFORM_HARDWARE.md 和本地参考照片配置，见下节。

## 型号参考外观（2026-11-09）

最新默认场景：`outputs/hardware_alignment_tip_20261109/micromani_equipped.usda`。
依据 `PLATFORM_HARDWARE.md` 与其中的本地实物/目录图片配置：KXL滑台银色金属、电机黑色；KRW/KGW黑色机构；EPG深灰黑色夹爪；HKVL银色筒身和深色端面。
实现位于 `isaacsim_tools/micromani_appearance.py`。解除导入时父级强制白色材质，19个可见CAD网格分配部件材质，额外使用面子集区分6个电机和12组紧固件。纹理仅调节粗糙度，不修改CAD顶点或碰撞体。
相机整机型号未确认，未按IMX335参考模组替换其外壳；目录后缀、合并STL中的细节和微观粗糙度属于视觉近似。外观配置不修改关节行程或宣称完成实机参数标定。

## EPG颜色与材质细化

默认场景更新为 `outputs/epg_material_20261109/micromani_equipped.usda`，基于硬件对齐后的场景做材质覆盖，保留原相机、传感器、CAD、关节限位与初始姿态。
`micromani_epg_finish.py` 按已有CAD部件区分哑光炭灰机身、深黑端盖、半哑光夹指、银色滑块/接头、橡胶线缆和蓝色指示灯。参数来自产品照片的视觉近似；没有复制产品铭牌或声称测得真实表面反射参数。
查看近景：`scripts/open_simulation.sh --view-camera /World/EPG_Inspection`；该预览相机不加入三路训练图像。

## 其他机构参考图配色细化

当前默认场景为 `outputs/platform_photo_verified_20261109/micromani_equipped.usda`。
`micromani_platform_finish.py` 对现有CAD面细分黑色升降框架、银色电机端板/转接件/支架以及橡胶线缆。依据本地实机与型号照片近似配色，左右镜像分别映射；不修改机构几何与物理参数。EPG面集合与本次修改面集合保持不相交。重建材质子集时删除空集合，避免渲染器把空材质面集误应用到整个网格。
