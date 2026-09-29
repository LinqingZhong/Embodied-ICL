# 新机器人接入、关节控制与头部／腕部相机教程

本文说明如何在 Isaac Lab 中导入新的机器人模型，控制关节，并持续采集头部相机和腕部相机图像。适用于无外接显示屏的运行方式。

本文中的“接入机器人”是把机器人模型放入仿真。若要连接实体机器人，还需要其硬件 SDK、ROS 2 或通信驱动；导入模型不会自动建立硬件连接。

相关基础内容见[无显示屏仿真、图像保存与关节控制指南](headless_simulation_and_joint_control_zh.md)。

## 1. 整体结构与准备工作

```text
机器人 URDF / USD + 网格与贴图
    → 机器人关节机构、碰撞和动力学
    → ArticulationCfg 配置执行器和初始状态
    → InteractiveScene 管理机器人和相机
    → 接收控制目标 → 物理步进 → 更新状态
    → 读取头部／腕部图像 → 保存或发送给视觉模型
```

先激活本机已安装的环境：

```bash
source ~/anaconda3/bin/activate env_isaaclab
cd /home/agiuser/workspace/dev_zlq/IsaacLab
```

需要准备的资产和参数：

| 项目 | 要确认的内容 |
| --- | --- |
| 机器人模型 | URDF 或带 articulation 物理结构的 USD，以及网格和贴图 |
| 连杆与关节 | 名称、父子关系、关节类型、轴方向、零位和限位 |
| 动力学 | 每个连杆的质量、质心、惯量和碰撞形状 |
| 执行器 | 位置／速度／力矩控制，输出限制，刚度和阻尼 |
| 基座 | 固定式机械臂，或需要自由移动的机器人 |
| 相机 | 对应连杆、安装位姿、内参、分辨率及采集频率 |

普通 OBJ/STL 主要提供外形，不能完整表达机器人的关节机构。若只有这些网格，需要先在 URDF 或 USD 中补齐连杆、关节和物理属性。

## 2. 把机器人资产转换为 USD

### 2.1 建议的资产目录

下面是目录示例，需要自行放入真实文件：

```text
assets/my_robot/
├── robot.urdf
├── meshes/
├── textures/
└── usd/
```

检查 URDF 中的网格路径是否能被导入器解析。若模型来自 ROS 包，不能默认所有 `package://` 引用都已配置正确；保留包结构并核对解析结果，必要时改为可访问的文件路径。

若输入是 Xacro，应先用 ROS 的 Xacro 工具展开成 URDF。当前仓库的转换命令接收 URDF 文件。

### 2.2 运行 URDF 转换工具

参考 [convert_urdf.py](../scripts/tools/convert_urdf.py)。固定基座机器人的示例命令：

```bash
mkdir -p assets/my_robot/usd

./isaaclab.sh -p scripts/tools/convert_urdf.py \
  assets/my_robot/robot.urdf \
  assets/my_robot/usd/robot.usd \
  --fix-base \
  --joint-target-type position \
  --joint-stiffness 100.0 \
  --joint-damping 5.0 \
  --headless
```

这里的增益仅演示参数用法，应根据具体关节调整。对人形、四足或移动底盘，若需要浮动基座，不要添加 `--fix-base`。固定基座可用于最初的关节调试，但不能验证行走或平衡控制。

工具也支持 `--merge-joints`，用于合并固定关节。初次接入头部／腕部相机时建议先不合并，以便保留和检查安装连杆。转换后的 USD 层级可能与 URDF 不完全相同，仍需要检查实际路径。

如果已经有 USD，可以跳过转换，但要确认它确实包含 articulation、关节、刚体和碰撞属性。只有可视网格的 USD 不能直接当作可控机器人。

转换输出可能引用其他文件。迁移资产时应保留完整输出目录和相对引用。

## 3. 配置机器人执行器和初始状态

可参考仓库的 [Franka 配置](../source/isaaclab_assets/isaaclab_assets/robots/franka.py) 和 [add_new_robot.py](../scripts/tutorials/01_assets/add_new_robot.py)。

下面是放在 `AppLauncher` 启动之后的配置片段。路径、关节名、初始角度和增益均需根据真实机器人修改，不是可直接运行的完整程序。

```python
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

MY_ROBOT_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/agiuser/workspace/dev_zlq/IsaacLab/assets/my_robot/usd/robot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={".*": 0.0},  # 示例；替换为无碰撞且满足限位的初始姿态
        joint_vel={".*": 0.0},
    ),
    actuators={
        "position_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=100.0,
            damping=5.0,
            effort_limit_sim=20.0,
        ),
    },
)
```

该片段为了展示结构，把全部关节放进同一个位置执行器组。实际机器人通常需要拆分手臂、头部、夹爪和轮子等组，并使用互不重叠的关节表达式配置各自控制方式。被动关节不能当成主动关节统一驱动。

旋转关节的位置单位是弧度，速度是弧度／秒，力矩是牛·米；移动关节对应米、米／秒和牛。初始角度不能直接假定全零合法。

## 4. 把头部相机和腕部相机放到对应连杆上

相机的父节点应是实际运动的连杆，这样它会随关节运动：

```text
Robot
├── head_link
│   └── head_camera
└── wrist_link
    └── wrist_camera
```

上图只是示意，真实 USD 可能有更深的层级。`head_link` 和 `wrist_link` 必须替换成实际连杆路径。

参考 [add_sensors_on_robot.py](../scripts/tutorials/04_sensors/add_sensors_on_robot.py)，将机器人与两台相机组合到场景配置中：

```python
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass


@configclass
class MyRobotSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2500.0),
    )
    robot = MY_ROBOT_CFG

    head_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/head_link/head_camera",
        update_period=1.0 / 30.0,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            horizontal_aperture=20.955,
            clipping_range=(0.05, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.05, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            convention="ros",
        ),
    )

    wrist_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/wrist_link/wrist_camera",
        update_period=1.0 / 30.0,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            horizontal_aperture=20.955,
            clipping_range=(0.02, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.05),
            rot=(1.0, 0.0, 0.0, 0.0),
            convention="ros",
        ),
    )
```

安装位姿都是示例：`offset.pos` 是相对于父连杆的平移，`offset.rot` 是按 `w, x, y, z` 排列的四元数。在 `convention="ros"` 下，相机坐标为 **Z 向前、X 向右、Y 向下**。单位四元数并不保证相机朝机器人前方，必须按连杆坐标系设置。

把相机放到机器人表面之外，避免被自身网格遮挡。腕部近距离观察还要检查近裁剪面是否切掉夹爪或物体。

若 USD 已有 Camera prim，可使用 `spawn=None` 引用现有相机；如果它只是相机外壳网格，则仍需创建 Camera prim。仿真相机不会自动继承实体相机的标定，需另外设置内参、外参和所需的畸变模型。

## 5. 关节控制和相机采集如何一起运行

### 5.1 创建场景并确认关节

以下片段承接前面的配置，仍须放在已启动 Isaac Sim 的程序中：

```python
from isaaclab.scene import InteractiveScene

sim_dt = 1.0 / 120.0
sim = sim_utils.SimulationContext(
    sim_utils.SimulationCfg(dt=sim_dt, device="cuda:0")
)
scene = InteractiveScene(MyRobotSceneCfg(num_envs=1, env_spacing=2.0))
sim.reset()

robot = scene["robot"]
print(robot.joint_names)
print(robot.body_names)

# 替换为真实关节名，必须恰好匹配一个关节。
joint_ids, _ = robot.find_joints("arm_joint1")
assert len(joint_ids) == 1
```

### 5.2 示例时序：120 Hz 物理，30 Hz 相机

可以每个物理步更新控制，每 4 步渲染并读取一次图像。下面的循环用 `simulation_app` 表示 `AppLauncher` 创建的应用：

```python
import math

step = 0
while simulation_app.is_running():
    sim_time = step * sim_dt
    targets = robot.data.default_joint_pos.clone()
    targets[:, joint_ids] += 0.1 * math.sin(2.0 * math.pi * 0.5 * sim_time)

    # 示例位置目标；使用前需确认摆动范围满足真实关节限位。
    robot.set_joint_position_target(targets)
    scene.write_data_to_sim()
    sim.step(render=False)
    step += 1

    capture = step % 4 == 0
    if capture:
        sim.render()
    scene.update(sim_dt)

    if capture:
        head_rgb = scene["head_camera"].data.output["rgb"]
        wrist_rgb = scene["wrist_camera"].data.output["rgb"]
        # 在这里处理两台相机数据，或交给保存/通信模块。
        print(step * sim_dt, head_rgb.shape, wrist_rgb.shape)
```

此处显式控制渲染频率，因此关闭了每次 `sim.step()` 的自动渲染。相机 `update_period` 控制更新周期，不能单独替代实际渲染；读取图像要放在渲染及场景更新之后。相机数据通常按需更新，仅调用 `scene.update()` 不代表所有图像都已被读取。

初始化后先运行若干帧，确认图像有效，再开始正式记录。上述“同时采集”指同一仿真步后的两台相机数据，不代表模拟了真实硬件的曝光同步或滚动快门。

对于有限步数的测试脚本，关闭应用前应清理仿真实例的回调，避免停止回调持续等待：

```python
sim.clear_all_callbacks()
sim.clear_instance()
simulation_app.close()
```

### 5.3 控制接口与外部控制器

| 控制方式 | 接口 |
| --- | --- |
| 位置目标 | `robot.set_joint_position_target(...)` |
| 速度目标 | `robot.set_joint_velocity_target(...)` |
| 力／力矩目标 | `robot.set_joint_effort_target(...)` |

切换接口时应同步检查执行器配置。纯速度控制一般不需要位置恢复力；纯力矩控制也应避免未预期的 PD 驱动叠加。`write_joint_state_to_sim()` 直接改物理状态，主要用于初始化和重置，不能代替电机驱动过程。

外部算法、Socket 或 ROS 2 可以作为目标来源：

```text
外部控制消息 → 线程安全的最新指令缓存 → 仿真主循环读取
    → 关节目标映射与限位检查 → 写入仿真 → 步进
    → 返回关节状态、头部图像、腕部图像和时间戳
```

通信回调只更新指令缓存，主循环负责操作仿真对象。需要明确关节顺序、单位、消息时间戳，以及指令超时后的处理；不要阻塞物理循环等待网络数据。

## 6. 保存两台相机的数据

以下片段可放到采集分支。`output_dir` 和 `frame_id` 应在循环外初始化，编号独立递增，避免重置仿真时覆盖旧数据。

```python
from pathlib import Path

import numpy as np
from PIL import Image

# 放在循环外：
output_dir = Path("output/my_robot")
for name in ("head_camera", "wrist_camera"):
    (output_dir / name).mkdir(parents=True, exist_ok=True)
frame_id = 0
```

在每次采集分支中保存：

```python
for name in ("head_camera", "wrist_camera"):
    camera = scene[name]
    rgb = camera.data.output["rgb"][0, ..., :3].cpu().numpy()
    depth = camera.data.output["distance_to_image_plane"][0].cpu().numpy()
    Image.fromarray(rgb.astype(np.uint8)).save(
        output_dir / name / f"rgb_{frame_id:06d}.png"
    )
    np.save(output_dir / name / f"depth_{frame_id:06d}.npy", depth)
frame_id += 1
```

`[0]` 表示第一套并行环境。RGB 通常是 uint8；深度保留原始浮点值，`distance_to_image_plane` 表示到成像平面的深度，单位为米，无效区域可能包含非有限值。

正式采集建议额外记录仿真时间、关节位置/速度、相机位姿和内参，方便重建控制与视觉的对应关系。相机采集频率、模型推理频率和磁盘保存频率可以不同。

同步 `.cpu()` 和 PNG 编码会影响吞吐量。持续采集时可使用有界队列交给后台写盘，并明确队列满时是丢帧还是减速；后台应使用已复制的数据快照，避免读取被下一帧更新的缓冲区。

## 7. 无显示屏运行与“实时”的含义

先可用仓库现成的单相机示例熟悉流程：

```bash
./isaaclab.sh -p scripts/tutorials/04_sensors/add_sensors_on_robot.py \
  --headless --enable_cameras --num_envs 1
```

该示例使用仓库自带的 ANYmal，并非你的新机器人，也不包含本文的双相机保存逻辑。

建议复制该脚本为自己的 `scripts/tutorials/04_sensors/run_my_robot_cameras.py`，保留 `AppLauncher` 启动结构，替换机器人、场景配置和主循环，然后运行：

```bash
./isaaclab.sh -p scripts/tutorials/04_sensors/run_my_robot_cameras.py \
  --headless --enable_cameras --num_envs 1
```

这里的新脚本名称仅为建议，本文未创建该文件。本文代码是组合参考，需替换真实资产和关节/连杆名后再运行；也可把脚本的 `args_cli.num_envs` 传给场景配置。

无显示屏仍能使用 GPU 渲染。30 Hz 的相机周期表示每秒**仿真时间**约 30 帧，不保证每秒**现实时间**也生成 30 帧。实际速度取决于相机数量、分辨率、输出类型、场景、推理和写盘开销。

建议先用一个机器人和两台 640×480 RGB 相机验证，再加入深度和更高分辨率；只需 RGB 时，把配置中的 `data_types` 改为 `["rgb"]` 并去掉深度读取。物理控制频率和物理步进频率也可不同，较慢的控制器可以在多个物理步之间保持目标。

测试前可以检查 GPU 占用：

```bash
nvidia-smi
```

本机安装时曾有其他进程占用大量显存，实际运行前应重新检查，不能沿用当时的剩余显存估计。

## 8. 推荐接入顺序和常见问题

1. 验证模型尺寸、初始姿态、基座和碰撞，无相机运行。
2. 查询关节名，逐个测试小幅位置目标和实际反馈。
3. 添加头部相机，确认连杆路径、朝向和图像内容。
4. 添加腕部相机，运动手臂，确认相机随连杆运动。
5. 同步记录两台相机和关节状态，再接外部控制器。
6. 最后测试帧率、延迟、连续运行和数据保存吞吐量。

| 现象 | 优先检查 |
| --- | --- |
| 找不到机器人或关节 | 资产路径、articulation 属性、关节名和导入结果 |
| 关节不动或抖动 | 控制模式、增益、输出限制、关节限位、碰撞与惯量 |
| 浮动机器人倒下 | 初始姿态、支撑和姿态控制；单关节目标不等于平衡控制 |
| 找不到相机父节点 | 实际 USD 连杆路径，以及固定关节是否已被合并 |
| 相机没有跟随手臂 | 是否挂在实际腕部连杆下面，而非机器人根节点 |
| 图像全黑或只看见机器人内部 | 灯光、相机安装位置/朝向、裁剪范围及初始渲染 |
| 图片重复或更新很慢 | 实际渲染频率、相机周期、数据访问时机和写盘耗时 |

参考源码还包括 [cameras.py](../scripts/demos/sensors/cameras.py) 和 [run_diff_ik.py](../scripts/tutorials/05_controllers/run_diff_ik.py)。后者展示末端目标如何通过逆运动学变成关节角度；接入新机器人时还需要替换末端连杆、关节集合和运动学参数。

## 9. 验证范围

本文按当前仓库的转换工具、机器人配置与相机示例整理。尚未提供真实的新机器人资产，因此未执行该机器人的导入、双相机渲染或帧率验证。示例中的资产路径、关节名称、连杆路径、安装位姿和控制参数均需要按实际模型调整。
