# 单臂底座坐标系的法兰 FK、IK 与 EEF 控制

支持 ARX X5（单/双臂）、AgileX Cobot Magic（四臂）、Franka Panda。AgiBot A2D 保留 joint 控制，本次未增加 IK。

## 坐标系与资产入口

| 配置 | arm | 底座 | 法兰 |
|---|---|---|---|
| x5 | arm | base_link | link6 |
| x5_dual | left / right | 各实例 base_link | 各实例 link6 |
| cobot_magic | fl / fr / bl / br | 对应 *_base_link | 对应 *_link6 |
| franka | arm | panda_link0 | panda_link8（link7 沿 Z 偏移 0.107 m） |

所有位姿都是该臂底座到法兰的变换，单位米、弧度，四元数顺序 **wxyz**，必须为单位四元数。不含夹爪中心或额外 TCP。底座移动后仍使用当前底座坐标系。

资产入口：`source/isaaclab_assets/data/Robots/` 下的 `ARX/X5/kinematics.py`、`AgileX/CobotMagic/kinematics.py`、`FrankaEmika/kinematics.py`。它们声明运动链，公共算法在 `scripts/demos/web_control/kinematics.py`。仿真加载资产后从实际 USD 的关节轴、父子局部变换构建运动链，因此不使用另抄的 DH 参数；FK/IK 算法本身使用 NumPy/SciPy，不操作仿真状态。Franka 法兰偏移采用 Panda 标准 link8 定义，不包含 panda_hand 的夹爪旋转。

启动仍使用 `start_simulation.sh --robot ...`，自动加载匹配模块。启动时检查 FK 对照物理姿态，不一致时拒绝启动；日志出现 `KINEMATICS_VERIFIED`。客户端保持纯标准库。

## Python 接口

```python
from robot_client import RobotClient
robot = RobotClient('http://127.0.0.1:8080', timeout=90)
print(robot.get_arms())  # base、flange、joint_names、limits、精度阈值
arm = 'fl'  # Cobot；双 X5 用 left/right，单 X5 和 Franka 用 arm
pose = robot.fk(arm)  # 当前实测关节角对应的 FK
solution = robot.ik(arm, pose['position'], pose['quaternion'])
reconstructed = robot.fk(arm, solution['joint_positions'])

task = robot.execute_eef_chunk(
    arm=arm,
    positions=[pose['position']],
    quaternions=[pose['quaternion']],
    steps_per_action=120,
)
print(robot.wait(task.id, timeout=120).details)
```

`fk(arm, joint_positions)` 可指定角度，顺序见 `get_arms()`。`ik(..., seed=q)` 可指定初值，缺省为当前实际关节角。返回 `joint_positions`、`position_error`（米）、`rotation_error`（弧度）。当前接受阈值为 1e-4 m 和 1e-3 rad，这是上限而不是承诺所有输入均可收敛。

`get_joint_state()['eef_poses']` 是物理引擎测量的各臂法兰位姿，与同次返回的关节状态配对，可用于独立验证 FK。它不同于调用 FK 从关节角计算位姿。

## 执行语义

EEF chunk 最多 128 行，位置与姿态行数必须一致。整个请求先校验，再逐点 IK；当前状态作为第一点初值，前一解作为下一点初值。全部成功后才提交 joint chunk，任意一点失败整段拒绝（错误含行号）。规划期间拒绝其它控制请求；仿真继续推进，但不会下发部分 IK 结果。

不插值、不规划碰撞、不自动等待到位。每行 IK 解按指定物理步数执行；未指定的手臂保持其已施加目标。返回任务包含 `control=eef`、`arm`、`solved_joint_positions`，进度与取消沿用 joint 接口。首版一个 chunk 控制一条手臂；若需多臂同步，可分别调用 IK，然后合并成一个 joint chunk。

数值 IK 可能因初值、奇异点或限位不收敛，失败不能证明目标绝对不可达。七自由度 Franka 同一末端位姿可对应不同角度，因此应验证 FK→IK→FK 的位姿重建误差；原关节角作为 seed 时应保持原解。求解不保证不同分支之间的连续性，调用端应检查返回的关节序列并决定是否执行。

## 精度验证

运行中的测试实例可执行（会移动每条手臂并返回）：

```bash
python scripts/demos/web_control/tests/check_eef_live.py \
  --robot cobot_magic --output /tmp/cobot_eef.json
```

测试连接 8083。每臂在初始姿态附近采样 12 组合法关节角，验证原解初值与扰动初值的 FK→IK→FK；另外在三个实际运动姿态将 FK 与物理引擎测量对照。数值误差与动力学跟踪误差分开记录。局部采样不代表全工作空间验证，求解精度不等于控制到位精度。

## 本机实测结果（2026-09-22）

每臂 12 组局部随机姿态。原解作为初值时，关节重建误差均小于 1e-6 rad；下表为扰动初值后 FK→IK→FK 的最大误差，以及三个实际姿态的物理对照最大位置误差。

| 配置 | 手臂 | 重建位置误差 m | 重建姿态误差 rad | FK/物理位置差 m | EEF 目标保持 1 秒后的跟踪位置误差 m |
|---|---|---:|---:|---:|---:|
| x5 | arm | 5.55e-13 | 6.88e-13 | 5.07e-08 | 9.54e-10 |
| x5_dual | left | 5.55e-13 | 6.88e-13 | 9.93e-08 | 9.54e-10 |
| x5_dual | right | 5.55e-13 | 6.88e-13 | 9.93e-08 | 9.54e-10 |
| cobot_magic | fl | 6.15e-11 | 9.45e-10 | 1.08e-07 | 0.0155 |
| cobot_magic | fr | 5.78e-11 | 8.83e-10 | 1.52e-07 | 0.0155 |
| cobot_magic | bl | 8.76e-11 | 1.32e-09 | 2.96e-07 | 0.00268 |
| cobot_magic | br | 8.86e-11 | 1.32e-09 | 2.59e-07 | 0.00268 |
| franka | arm | 1.77e-12 | 6.69e-12 | 3.98e-07 | 0.0699 |

**精度结论：FK/IK 重建与独立物理姿态对照通过；不能据此声明末端精确到位。** 保留原执行器参数时，Cobot 前臂跟踪误差约 1.55 cm，Franka 约 7 cm。需要精确 EEF 跟踪的任务还需调整执行器/重力补偿等控制策略；本次没有擅自更改这些物理配置。测试脚本将跟踪误差作为测量记录，数值精度与物理对照才使用严格阈值。

Franka 的扰动初值可能返回不同关节解，这是七自由度冗余；重建法兰位姿仍精确。自动 IK 初值来自实际状态，当物理状态短暂越过软限位时，将初值投影到合法范围。显式越界 seed 仍拒绝；目标和返回解不绕过限位检查。

本机原始记录：`output/eef_verification/*.json` 和同目录日志。测试实例均已停止。
