# MicroMani 仿真传感器与控制接口

2026-11-10 检查及补全。默认场景启动脚本 `scripts/open_simulation.sh` 同时开启本机 HTTP API：`http://127.0.0.1:49012`。服务器只监听 loopback；关闭 Isaac Sim 后接口关闭。可通过 `MICROMANI_API_PORT` 修改端口，0 关闭 API。

## 封装现状

原有封装包括 `micromani_collect_isaac.py` 的关节读写和六维反力、`micromani_grip_sensing.py` 的指尖接触力、`micromani_camera_preview.py` 的三路图像。此次新增 `micromani_api.py` 统一服务和 `micromani_client.py` 无第三方依赖的 Python 客户端，接入默认 GUI/录制运行器。

修复了默认 GUI 在 smoke 验证后进入 keep-open 循环便不再接收控制的问题。现在录制和持续预览两种状态都能读传感器、处理控制。HTTP 工作线程仅访问缓存与命令队列，Isaac 调用全部在仿真线程执行。

## 一个手臂只有一个夹爪控制量

对外共14个控制量：左臂6轴+左夹爪，右臂6轴+右夹爪。左右指的是模型左右，操作员左右与它们相反。

`left_gripper` 与 `right_gripper` 各驱动两根夹指，两个底层指关节接收同一位移目标，因模型轴向相反而对称开合。`joint_finger1_left` 等单指名称被拒绝。四个指尖的接触力仍分别返回，用于观察单侧/双侧接触。

推荐 `client.set_gripper('left', opening_mm=4.5)`：参数为名义总开口，范围0至20 mm；内部将开口除以2000转换为单指位移（米）。原始14维接口中的 gripper 值仍是单指位移，0至0.01 m，以兼容既有遥操作和数据格式。`gripper_opening_mm` 为两个指关节平均位移换算的名义总开口；实际CAD零位存在约0.0006 mm几何偏差，不是精确碰撞间距测量。

## HTTP 端点

| 方法与路径 | 内容 |
|---|---|
| GET `/v1/schema` | 接口版本、控制顺序、关节限制、单位、坐标与信号定义 |
| GET `/v1/state` | 同一次采样的关节状态、目标、名义开口、两路六维力/矩、四指接触力及两侧均值 |
| GET `/v1/cameras` | 三路640×480 RGB JPEG（base64），各自仿真渲染时间戳 |
| POST `/v1/commands` | 全14维 `axes` 或逻辑关节部分更新 `joints`；二选一，可带 `ttl_s` |
| GET `/v1/commands/{id}` | accepted / applied / expired / failed / cancelled |
| POST `/v1/stop` | 清除排队命令，下个输出周期保持手臂实测位置及夹爪当前驱动指令 |

命令被接收返回202，**并不意味着已经执行**。`applied` 表示仿真线程接收连续运动目标，并非物理到位/夹持成功。队列容量32；满时429；保留最近256条结果，旧ID可能404。TTL默认2墙钟秒，可设0.05至10秒；暂停期间不执行，在恢复处理时判断过期。每个30Hz输出周期最多接收一条HTTP命令，部分更新在执行时合并，避免后续命令覆盖此前对其他轴的修改。

数值须有限且在限制内；未知名称、NaN、Inf、越界、非法形状返回400。控制不静默裁剪。不要因请求超时直接重复提交运动指令，应先查询已有ID。既有 UDP 控制保留在49010；同一周期先处理UDP，再处理HTTP；运动忙时拒绝后续目标，不能混用两个控制源。

## Python 示例

```python
import sys
sys.path.insert(0, '/home/nkj/Projects/MicroManiURDF/isaacsim_tools')
from micromani_client import MicroManiClient

sim = MicroManiClient()
print(sim.schema())
state = sim.state()                 # 包含时间戳与 stale
forces = sim.force_torque()         # 2个六维力/力矩传感器
contacts = sim.grip()               # 每侧2指读数、均值与双侧接触标志
images = sim.cameras()              # 三路 JPEG + 各自时间戳
left_image = sim.camera('left_wrist')

# 以下调用会改变仿真；左右指模型左右。
accepted = sim.set_gripper('left', opening_mm=4.5)  # 左手两指同步
result = sim.wait(accepted['id'])
assert result['status'] == 'applied', result
state = sim.wait_motion(command_id=accepted['id'])
print(state['motion'])  # reached 或 contact_or_tracking_error 等

# 单轴、单臂、双臂均可控制；位置为绝对关节坐标。
accepted = sim.set_joints({'joint1_left': 0.002})
# sim.set_arm('right', [0.002, 0.045, 0.034, 0., 0., 0.])
# sim.set_action(完整14维目标列表)
```

相机图片可用 `base64.b64decode(left_image['jpeg'])` 解码成JPEG文件字节。客户端没有强制要求numpy/Pillow；是否保存或解码由调用方决定。读取操作不产生运动。

## 单位、频率和实际能力

- 每臂前三轴为m，后三轴（RX/RZ/RY）为rad；关节目标不是末端笛卡尔位姿。
- FT轴序Fx/Fy/Fz/Mx/My/Mz，单位N、N·m，坐标为各自 joint6 child frame。提供 raw_wrench 以及HKVL-36A范围/分辨率模型后的wrench和饱和标志。含重力、未去皮的关节支撑反力，不能直接当作外加接触力。
- 四指读数是整个指体接触力大小，均值不是标定过的电机夹紧力；数据来自仿真，不是真实硬件协议。
- 默认物理步为960 Hz，相机和传感器发布为30 Hz仿真时间；墙钟速率取决于渲染性能，不保证实时30 FPS。硬件仓库的FT采样链路另有200/1000 Hz，未在此虚构。
- state 内数值是同一仿真步采样。相机与state分开读取，不保证跨HTTP请求同帧；应比较 simulation_time/render_time。两个接口均返回 age_wall_s 和 stale（超过1秒）。暂停时缓存不冒充新采样。
- 新运行器只在初始化时设置一次关节位置；之后以限速、限加速度的五次曲线更新物理驱动目标。每臂仍为6轴+1个耦合夹爪开合量。`applied`表示轨迹已接收，实际完成看`state.motion.status`。运动中提交新轨迹会返回failed/busy。
- `motion.status`为moving/settling/reached/contact_or_tracking_error/stopped/idle。夹紧物体导致不能到达更小开口时会出现contact_or_tracking_error，需结合夹指力与图像判断，不能视为抓取成功。
- `POST /v1/stop`（客户端`stop()`）取消排队命令，并在下个传感器周期保持手臂实测位置，同时保留两侧夹爪已经执行到的驱动指令，以免用实测位置覆盖目标而卸掉夹持预紧；正在闭合时不会继续追踪尚未执行的最终目标。这是软件保持，不是硬件急停，也不主动释放夹爪。此行为需要重启运行器加载新代码；通过 `/v1/schema` 的 `stop_behavior` 字段确认，旧进程不支持该保持语义。运动状态可用`wait_motion()`等待，但必须先确认相应命令applied；仅支持单一控制器。
- 速度上限：平移2mm/s，转动0.1rad/s，每指0.5mm/s；加速度上限：平移10mm/s²，转动0.5rad/s²，每指2mm/s²。接触时实际位置可能滞后，不每步清零速度。
- `outputs/assembly_physics/micromani_assembly.usda`提供动态零件和TPU接触材料；旧table_parts场景仍保留。密度、摩擦、柔顺性未按实物标定，详见ASSEMBLY_PHYSICS.md。

## 兼容说明和验证

`micromani_control_bridge.py` 是旧的独立桥，TCP49011会与图像预览冲突，不应和默认运行器同时启动。其单指命令入口也已改为只允许14个逻辑控制量。`micromani_record_command.py` 服务于另一个UDP录制运行器，并非本HTTP服务的客户端。

旧位置设定版本的历史验证：31项测试通过；在默认GUI smoke结束后的持续运行状态，通过HTTP读取3路图像、2路六维力/矩和4指接触力；分别修改左右夹爪开口与左右X轴小位移，查询到applied并核对状态跟随，最后恢复全部原始目标。测试验证单指控制请求被拒绝。

动态装配运行器联调：37项测试通过，GUI实际验证连续移动、夹爪开合、停止保持、恢复原始目标及三路图像/全部传感器读取。物理夹具与材料限制见ASSEMBLY_PHYSICS.md；不代表自主装配已验收。

## 公开标定辅助

`micromani_robot_calibration.py` 根据机器人URDF与夹指CAD计算正运动学，不读取场景物料。`tcp(action, side)` 返回列向量约定的TCP到base_link变换，单位m；TCP是两指最前端平面的中点，姿态沿link6。设计几何不等于实物标定，当前计算已与机器人网格指尖中点在20组姿态下交叉检查。

`micromani_wrench_compensation.py` 从空载静止时的公共六维力读数和机器人姿态拟合支承重量、质心，拒绝姿态变化不足的拟合。其旋转输入是sensor_to_world；当前模型中左joint6局部传感器旋转为diag(1,-1,-1)，右为单位阵，再左乘link6世界旋转。`compensated_wrench()`只扣除重力模型，不扣惯性、摩擦或传感器噪声。左侧已用独立空载姿态验证，右侧仍需转动空载标定；输出不自动替代API原始读数。不能用接触中的数据重新拟合，否则可能把真实接触力当成偏置消掉。HKVL量化分辨率仍然有效，原始仿真浮点残差小不代表实物具备相同精度。

### Contact phase guard (controller-side public-sensor checks)

`micromani_contact_guard.ContactGuard` consumes public sensor messages and a
wrist residual magnitude calculated using an independently fitted free-air
gravity calibration. It does not query the simulator or estimate object pose.
Five distinct simulation timestamps and sequence numbers with both finger
magnitudes above 3 mN confirm bilateral contact. Repeated network polls do not
count as additional sensor samples. Contact loss resets the count; stale,
invalid, saturated, regressing-clock or excessive-force data latch a fault.
A new guard must be explicitly created after a fault is investigated.

The initial limits (50 mN per finger, 20 mN unilateral, 100 mN residual wrist
force) are exploratory simulation settings, not allowable TPU/part loads or
hardware sensitivity claims. By default the other gripper must remain contact-free. `holding_other=True`
checks that an existing grasp is retained; individual brokers apply additional
force limits. These checks do not by themselves establish assembly success. Bilateral confirmation alone does not prove a grasp:
images and a small lift must establish that the intended part was picked up
without slip. The guard has been used during live pickup/carry trials, including a detected
sleeve drop during rotation. Contact confirmation must not be reported as
stable full assembly.

### 接触受限的有限运动完成判定

`micromani_completion.ContactCompletionGate` 是控制器侧的只读判定，不修改服务器状态、驱动目标或物理参数。调用者必须明确提供命令 ID、完整目标和允许接触的夹爪边界。

服务器的原始 `contact_or_tracking_error` 会保留。只有轨迹时间及参考目标已完成、所有手臂轴在原位置容差内、误差仅来自指定夹爪的有限正向压缩、双侧力均在指定范围内，并获得至少 5 个新鲜且夹爪位置稳定的终态样本，才能单独标记为 `contact_limited_complete`。这不是 `reached`，也不是抓持成功。

未指定的夹爪仍须满足原 5 µm 位置容差。陈旧/回退时钟、命令或目标改变、非有限读数、腕部饱和、单侧失去接触、手臂误差或无法解释的终态错误都会拒绝完成。该判定只解释终态，不能替代运动过程中的力保护与图像检查。

当前验证：14 项离线判定用例，以及一次真实公开 hold 和一次掉落记录回放；控制器工作区的力确认闭合与接触保持搬运分别通过 5 项、9 项离线用例；只读两秒接触保持观察通过 6 项离线用例。尚未以该新判定完成在线高预紧抓取或装配。力边界均为仿真探索参数，不能视为 TPU 或硬件允许载荷。

接触保持搬运可在已审核的轨迹段中指定 `maximum_linear_step_m`，范围 1–250 µm；默认 250 µm。此参数只限制每次有限运动的线性增量，不替代轨迹碰撞检查、夹持力检查及图像复核。夹爪预检压缩边界与完成判定一致（左 10 µm、右 25 µm），机械臂轴容差不变。

在线高预紧试抓在双指 14.829/15.290 mN 时暴露单侧判断与确认目标耦合错误，已停止且未提起，不能计为成功抓取。`ContactLimits.unilateral_contact_min_n` 现在可独立配置接触存在阈值；本试抓使用 1 mN，双侧确认仍为 15 mN，单指硬上限仍为 30 mN。默认未指定时保留旧行为。实测触发边界的 5 项回归已通过；随后 093059 公开试抓记录已验证双侧存在接触、力值跨越 15 mN 目标时不再误触发单侧停止；此项只证明该判定边界已修复，不证明抓取或装配成功。
