# Isaac Lab 无显示屏仿真、图像保存与关节控制指南

本文基于当前仓库中的示例，说明如何启动无界面仿真、保存相机图像，以及通过 Python 或外部控制信号驱动机器人关节。

## 1. 环境与启动方式

本机已创建 Conda 环境 `env_isaaclab`，使用 Python 3.11、Isaac Sim 5.1 和 PyTorch 2.7（CUDA 12.8）。

每次打开终端后执行：

```bash
source ~/anaconda3/bin/activate env_isaaclab
cd /home/agiuser/workspace/dev_zlq/IsaacLab
```

启动最基本的空场景：

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --headless
```

常用参数：

| 参数 | 含义 |
| --- | --- |
| `-p` | 使用当前 Python 环境运行指定脚本 |
| `--headless` | 不打开 GUI 窗口，适合无显示屏或远程服务器 |
| `--enable_cameras` | 启用相机渲染；需要相机图像时添加 |
| `--num_envs 1` | 对支持该参数的脚本，只创建一个并行环境 |

无外接显示屏也可以渲染，但运行仿真和渲染仍需要兼容的 GPU。`--headless` 不会自动保存图片；需要在场景中配置相机，再调用保存代码。

示例的基本运行流程是：

```text
AppLauncher 启动 Isaac Sim
    → 创建物理世界、机器人、灯光和相机
    → sim.reset() 初始化
    → 循环：发送控制 → 仿真步进 → 更新状态与相机 → 保存数据
```

通常先创建 `AppLauncher`，再导入依赖运行中 Isaac Sim 的模块。参考 [create_empty.py](../scripts/tutorials/00_sim/create_empty.py)。

## 2. 无界面渲染与保存图片

### 2.1 直接运行相机教程

参考脚本：[run_usd_camera.py](../scripts/tutorials/04_sensors/run_usd_camera.py)。

```bash
./isaaclab.sh -p scripts/tutorials/04_sensors/run_usd_camera.py \
  --headless \
  --enable_cameras \
  --save \
  --camera_id 0
```

该脚本创建地面、灯光、随机几何物体和两台相机。`--camera_id 0` 保存第一台相机的数据，也可以改为 `1`。

默认输出目录为：

```text
/home/agiuser/workspace/dev_zlq/IsaacLab/scripts/tutorials/04_sensors/output/camera/
```

相机分辨率为 **640 × 480**，配置了 RGB、深度、法线和分割数据。脚本通过 Replicator 的 `BasicWriter` 保存数据。它持续运行并写入文件，结束时可以按 `Ctrl+C`。

需要调整时，在脚本中查找：

| 配置内容 | 函数或变量 |
| --- | --- |
| 图像分辨率、数据类型、相机内参 | `define_sensor()` 中的 `CameraCfg` |
| 输出目录 | `run_simulator()` 中的 `output_dir` |
| 相机位置和观察目标 | `camera_positions`、`camera_targets` |
| 设置相机位姿 | `camera.set_world_poses_from_view(...)` |
| 保存逻辑 | `if args_cli.save:` 分支 |

`sim.set_camera_view(...)` 设置的是主视图；传感器相机的位姿需要通过相机自身的接口设置。

### 2.2 自己保存 RGB 和原始深度

下面是插入现有仿真脚本的代码片段，不能单独运行。假设 `camera` 已创建，当前帧已经渲染，并调用了 `camera.update(dt)`；如果相机属于 `InteractiveScene`，则通过 `scene.update(dt)` 更新。

```python
from pathlib import Path

import numpy as np
from PIL import Image

output = Path("output")
output.mkdir(parents=True, exist_ok=True)

# 第一台相机；只取 RGB 三个通道。
rgb = camera.data.output["rgb"][0, ..., :3].cpu().numpy()
Image.fromarray(rgb.astype(np.uint8)).save(output / "rgb.png")

# 保留原始浮点深度值，便于后续计算。
depth = camera.data.output["distance_to_image_plane"][0].cpu().numpy()
np.save(output / "depth.npy", depth)
```

说明：

- `output` 是相对于当前工作目录的路径。
- 此示例中的 `rgb` 使用相机输出的 uint8 颜色数据。
- `distance_to_image_plane` 表示到成像平面的深度，单位为米，并不是到相机中心的欧氏距离；背景等无效区域可能包含非有限值。
- 连续保存时应加入帧号，例如 `rgb_000001.png`，避免覆盖同名文件。
- 用于计算的深度建议保存为 `.npy` 等保留浮点数据的格式；可视化 JPG 不能替代原始深度。
- 可以每隔若干步保存一次，减少磁盘写入；相机也可能按 `update_period` 以低于物理步进的频率更新。

### 2.3 带机器人相机的示例

[cameras.py](../scripts/demos/sensors/cameras.py) 演示 ANYmal 机器人上的普通相机、平铺相机和射线投射相机，并保存图像网格：

```bash
./isaaclab.sh -p scripts/demos/sensors/cameras.py \
  --headless --enable_cameras --num_envs 1
```

输出位于 `scripts/demos/sensors/output/` 下的子目录。脚本每隔 10 个仿真步保存可视化图像；其计数器会在重置时归零，长时间运行可能覆盖先前同名文件。需要连续采集数据时，应使用独立递增的文件编号。

机器人资产首次加载可能需要从远程服务器下载。

## 3. 让机器人某个关节运动

不需要先训练强化学习模型，可以直接设置关节的位置、速度或力矩目标。

### 3.1 最适合入门的脚本

[add_new_robot.py](../scripts/tutorials/01_assets/add_new_robot.py) 同时演示：

- Jetbot：设置轮子的目标速度，让小车前进和转弯。
- Dofbot：设置机械臂的目标关节角度，让机械臂摆动。

```bash
./isaaclab.sh -p scripts/tutorials/01_assets/add_new_robot.py \
  --headless --num_envs 1
```

这个脚本没有配置相机图片保存。无界面运行时，可以打印关节角度检查运动，或者把相机教程中的配置与保存逻辑加入同一个场景。

### 3.2 修改为仅第一个关节摆动

脚本中 `run_simulator()` 的 `# wave` 代码块让 Dofbot 前四个关节一起摆动。可以把该代码块替换为：

```python
robot = scene["Dofbot"]

# 保持其他关节的目标值为默认姿态。
# clone() 避免修改 default_joint_pos 本身。
target = robot.data.default_joint_pos.clone()

# 第一个关节围绕默认角度摆动。
# 振幅 0.25 rad，频率 0.5 Hz，即每 2 秒一个周期。
target[:, 0] += 0.25 * np.sin(2 * np.pi * 0.5 * sim_time)

robot.set_joint_position_target(target)
```

保留原脚本后面的步进与更新逻辑：

```python
scene.write_data_to_sim()  # 把控制目标写入物理引擎
sim.step()                # 推进一个物理时间步
sim_time += sim_dt
count += 1
scene.update(sim_dt)      # 更新机器人和传感器状态
```

`target[:, 0]` 的第一维表示并行机器人实例，第二维表示关节索引。此代码会控制所有实例的第一个关节。旋转关节的位置单位是弧度，`0.25 rad` 约为 `14.3°`；移动关节的位置单位是米。

命令目标不等于实际位置，实际运动还受执行器、关节限制、接触和动力学影响。选择目标时应检查关节限位。

### 3.3 查询关节名称与反馈

在 `sim.reset()` 完成初始化后，可以查询：

```python
print(robot.joint_names)
print(robot.data.joint_pos)
print(robot.data.joint_vel)
```

读取仿真后的反馈前，确保已调用 `robot.update(sim_dt)`，或者在使用 `InteractiveScene` 时调用 `scene.update(sim_dt)`。

对于已初始化的 Franka 机器人，也可以按名称查找关节：

```python
joint_ids, joint_names = robot.find_joints("panda_joint1")
print(joint_ids, joint_names)

target = robot.data.default_joint_pos.clone()
target[:, joint_ids] += 0.2
robot.set_joint_position_target(target)
```

这段代码假设 `robot` 是 Franka；不能把 Franka 的关节名称直接用于 Dofbot。目标写入后仍需执行写入物理引擎、步进和状态更新。

### 3.4 三种控制接口

| 控制方式 | API | 常见用途 |
| --- | --- | --- |
| 位置目标 | `set_joint_position_target(...)` | 机械臂转到指定角度 |
| 速度目标 | `set_joint_velocity_target(...)` | 轮子按指定转速旋转 |
| 力／力矩目标 | `set_joint_effort_target(...)` | 自定义动力学或力矩控制器 |

接口需要与机器人的执行器配置匹配。例如位置控制需要合适的刚度和阻尼；纯力矩控制应避免未预期的位置驱动同时作用。执行器配置可参考 [franka.py](../source/isaaclab_assets/isaaclab_assets/robots/franka.py)。

`write_joint_state_to_sim(...)` 直接设置物理状态，主要用于初始化或重置。它与发送电机控制目标不同，不适合用来模拟关节逐渐转到目标角度。

力矩控制入门可参考 [run_articulation.py](../scripts/tutorials/01_assets/run_articulation.py)：

```bash
./isaaclab.sh -p scripts/tutorials/01_assets/run_articulation.py --headless
```

该示例对倒立摆系统施加随机关节力／力矩，展示 `set_joint_effort_target()`、`write_data_to_sim()`、`sim.step()` 和 `robot.update()` 的调用顺序。

## 4. 接入外部控制信号

上述教程直接在 Python 中生成控制目标。要接入外部程序、Socket 或 ROS 2，需要添加通信层，把收到的数据转换为同样的关节目标。

```text
外部控制器发送目标角度
    → 通信模块接收并缓存最新指令
    → 仿真主循环读取指令
    → set_joint_position_target(...)
    → write_data_to_sim()
    → sim.step()
    → update(dt)
    → 读取实际关节位置、速度和相机图像并反馈
```

建议通信回调只更新线程安全的指令缓存，由仿真主循环调用机器人接口。不要让等待网络消息阻塞每个物理时间步。

接入时应明确：

- 指令对应哪些关节，名称和排列顺序如何映射。
- 单位是弧度还是角度，以及位置、速度或力矩的控制模式。
- 指令频率与仿真步进频率不同的时候，如何保持或插值目标。
- 指令超时后使用什么目标，例如保持最近有效目标。
- 接收的数据如何转换成与机器人相同设备、正确形状的 PyTorch 张量。

这套 Python 接口控制的是仿真机器人。要控制实体机器人，还需要对应硬件驱动或机器人提供的通信接口。

## 5. 让机械臂末端移动到指定位置

如果目标是“机械臂末端移动到某个三维位置或姿态”，可以使用逆运动学，不必手动计算每个关节角度。

参考 [run_diff_ik.py](../scripts/tutorials/05_controllers/run_diff_ik.py)：

```bash
./isaaclab.sh -p scripts/tutorials/05_controllers/run_diff_ik.py \
  --robot franka_panda \
  --num_envs 1 \
  --headless
```

该脚本的流程是：

```text
末端目标位姿 → DifferentialIKController → 目标关节角度 → 位置控制接口
```

查找 `ee_goals` 可以修改末端目标。示例的目标位姿定义在机器人根坐标系下，排列为 `[x, y, z, qw, qx, qy, qz]`；四元数顺序是 `w, x, y, z`。

若需要操作空间力／位姿控制，可进一步阅读 [run_osc.py](../scripts/tutorials/05_controllers/run_osc.py)。

## 6. 导入新的物体资产：以杯子为例

普通杯子通常是没有关节的刚体，用 `RigidObject` 加载即可。机器人或带活动关节的机构才需要 `Articulation`。

基本流程：

```text
准备杯子模型和贴图
    → 检查尺寸、朝向和模型原点
    → 转成 USD，并配置刚体、质量和碰撞形状
    → 用 UsdFileCfg + RigidObjectCfg 放入场景
    → 验证掉落、接触、抓取和相机成像
```

### 6.1 准备模型文件

| 当前资产格式 | 导入方式 |
| --- | --- |
| `.usd`、`.usda`、`.usdc` | 使用 `UsdFileCfg` 直接引用；另行确认物理属性是否齐全 |
| `.obj`、`.stl`、`.fbx` | 使用仓库的 `convert_mesh.py` 转为 USD |
| `.blend`、`.glb`、`.gltf` 等 | 先用建模工具导出 USD 或上述转换工具支持的格式 |

建议把自定义资产集中存放。例如下面的目录仅为建议，需要自行放入真实模型：

```text
IsaacLab/
└── assets/
    └── cup/
        ├── cup.obj
        ├── cup.mtl          # 如果 OBJ 使用外部材质
        ├── textures/       # 如果模型使用贴图
        └── usd/            # 转换输出目录
```

保留材质文件、贴图和 USD 外部引用的相对路径。移动转换结果时，应一起移动它依赖的文件，不能默认只复制一个 `.usd` 就足够。

导入前检查：

- **尺寸**：场景通常以米为单位，例如杯高约 `0.1 m`。不要直接把以毫米建模的坐标当作米使用。
- **朝向**：Isaac Lab 常用 Z 轴朝上；确认杯口方向正确。
- **模型原点**：原点在杯底还是杯子中心，会影响初始放置高度。
- **几何和材质**：杯壁应有合理厚度；STL 通常只有几何，不能指望它保留完整材质和贴图。

### 6.2 用仓库工具转换 OBJ/STL/FBX

参考工具：[convert_mesh.py](../scripts/tools/convert_mesh.py)。进入已激活的环境并在仓库根目录运行：

```bash
mkdir -p assets/cup/usd

./isaaclab.sh -p scripts/tools/convert_mesh.py \
  assets/cup/cup.obj \
  assets/cup/usd/cup.usd \
  --collision-approximation convexDecomposition \
  --mass 0.2 \
  --headless
```

把输入路径替换成真实文件；STL 或 FBX 只需更换输入文件路径。这里 `0.2` 表示质量 `0.2 kg`，应按实际杯子调整。

当前脚本中，**传入 `--mass` 才会同时为转换结果添加质量和刚体属性**。不传时不会添加动态刚体属性，不能把“转换成功”直接当成“可以掉落和抓取”。

`--collision-approximation` 控制碰撞近似。转换时不需要 `--enable_cameras`。`--headless` 模式完成转换后退出，不会持续运行物理仿真。

如果要高效复制大量相同资产，可以额外使用 `--make-instanceable`。初次调试可先不加，便于检查和编辑资产。

该命令行脚本没有提供 `--scale` 参数。尺寸或朝向不正确时，可在建模软件中修正后重新导出；自定义转换程序也可以设置 [MeshConverterCfg](../source/isaaclab/isaaclab/sim/converters/mesh_converter_cfg.py) 的 `scale`、`rotation` 和 `translation`。仅在场景中缩放不能代替对质量和碰撞精度的检查。

### 6.3 杯子的碰撞形状为什么重要

渲染网格决定“看起来是什么形状”，碰撞网格决定“物理引擎认为是什么形状”。两者可以不同。

| 碰撞方式 | 对杯子的影响 |
| --- | --- |
| `convexHull` | 单一凸包通常会封住杯口、填平杯柄孔洞，适合较粗略的外形近似 |
| `convexDecomposition` | 用多个凸块近似，较适合作为动态杯子的起点，但不保证自动保留所有空腔 |
| `sdf` | 可用于更复杂的非凸碰撞，需要另外检查网格质量、分辨率和运行开销 |
| `none` | 不生成可用的碰撞响应；不能用于需要接触或抓取的杯子 |

对于“只从外侧夹住杯身”和“把物体放进杯子里”，碰撞精度要求不同。若要保留杯口、内壁和杯柄孔洞，需要检查分解结果；必要时手工制作多个简单碰撞体，或调整凸分解/SDF 参数。多个碰撞体可以属于同一个刚体。

不要直接假设普通三角网格碰撞适合动态杯子；动态非凸碰撞需要采用 PhysX 支持的表示方式。

### 6.4 在场景中生成杯子

以下是插入现有场景构建函数的片段，**不是独立启动脚本**。前提是 Isaac Sim 已通过 `AppLauncher` 启动，且 `cup.usd` 已具有刚体和碰撞属性，例如使用上面的转换命令生成。

```python
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg

cup_cfg = RigidObjectCfg(
    prim_path="/World/Cup",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/agiuser/workspace/dev_zlq/IsaacLab/assets/cup/usd/cup.usd",
        scale=(1.0, 1.0, 1.0),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            kinematic_enabled=False,
            disable_gravity=False,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.3),
        rot=(1.0, 0.0, 0.0, 0.0),  # 四元数顺序为 w, x, y, z
    ),
)

cup = RigidObject(cfg=cup_cfg)
```

该配置将杯子放在 `z=0.3 m` 的位置，开启重力，允许其自由运动。实际生成位置取决于模型原点；应避免杯子初始就与地面或桌面穿插。场景需要有带碰撞的地面或桌面，才能接住杯子。

在创建场景时、`sim.reset()` 之前执行上面的代码。需要读取杯子状态时，在步进后更新：

```python
sim.step()
cup.update(sim.get_physics_dt())
print(cup.data.root_pos_w)
```

如果杯子由 `InteractiveScene` 管理，则使用 `scene.update(sim_dt)` 统一更新；批量环境中的路径一般写为 `{ENV_REGEX_NS}/Cup`，避免所有环境共用 `/World/Cup`。

**已有 USD 的注意点：** `UsdFileCfg` 的这些物理配置主要修改 USD 中已存在的相应物理 schema。一个只有可视网格的 USD，不能保证仅填上 `rigid_props` 和 `collision_props` 就自动成为完整的可交互刚体。应先在资产制作阶段，使用 Isaac Sim 的物理编辑工具或 schema 定义接口，补齐刚体和碰撞属性。单个 `RigidObject` 对应的资产应包含一个物理刚体，其下可有多个可视网格和碰撞网格。

### 6.5 复用已有教程，观察杯子并保存图片

最快的组合方式是复制 [run_usd_camera.py](../scripts/tutorials/04_sensors/run_usd_camera.py) 为自己的脚本，例如 `scripts/tutorials/04_sensors/run_cup_camera.py`，然后：

1. 在 `design_scene()` 中、`return scene_entities` 之前加入上一节的杯子创建代码。
2. 加入 `scene_entities["cup"] = cup`，保留杯子对象的引用。
3. 可删除原来的随机几何物体生成循环，保留地面、灯光和相机。
4. 根据杯子的位置调整 `camera_positions` 和 `camera_targets`，让相机朝向杯子。
5. 如果需要打印杯子位置，在仿真循环 `sim.step()` 后调用 `scene_entities["cup"].update(sim.get_physics_dt())` 再读取数据。

完成复制和修改后运行：

```bash
./isaaclab.sh -p scripts/tutorials/04_sensors/run_cup_camera.py \
  --headless --enable_cameras --save --camera_id 0
```

这里的 `run_cup_camera.py` 是建议自行创建的文件，本文没有替你生成该脚本或杯子模型。若保留原相机脚本的输出逻辑，并把副本放在同一目录，图片仍保存到 `scripts/tutorials/04_sensors/output/camera/`；建议为不同实验设置独立输出目录。

也可参考 [run_rigid_object.py](../scripts/tutorials/01_assets/run_rigid_object.py)，它演示刚体的生成、重置、状态读取和步进。把其中的圆锥配置换成杯子的 `UsdFileCfg` 时，需要同时调整资产路径、初始位置以及相关变量和字典键名。

### 6.6 验证杯子资产

建议先做简单掉落实验，再尝试机器人抓取：

1. **外观**：尺寸和朝向正确，材质与贴图能找到，相机能看见杯子。
2. **重力**：动态杯子会掉落；若悬空不动，检查刚体、重力和运动学设置。
3. **接触**：落到地面后不会穿透；检查杯子和地面是否都有碰撞属性。
4. **空腔**：用小物体测试杯口和内部是否可进入；“看起来是空的”不代表碰撞也是空的。
5. **抓取**：检查质量、摩擦、碰撞形状和夹爪力。导入杯子不会自动生成抓取策略。

## 7. 推荐学习顺序与验证范围

1. 运行空场景，理解启动和仿真步进。
2. 运行 `run_usd_camera.py`，确认无界面图像保存。
3. 运行 `add_new_robot.py`，理解位置和速度控制。
4. 修改 Dofbot 示例，让单个关节运动并打印实际位置。
5. 把机器人和相机放入同一个场景，同时控制关节和保存图像。
6. 按需要接入外部控制器，或学习逆运动学。
7. 导入自己的杯子等物体，先验证掉落和碰撞，再与相机、机械臂组合。

本机环境安装阶段已经验证 GPU 张量运算和 10 步无界面空场景仿真。本文的相机、机器人控制和资产导入命令根据当前仓库源码整理，尚未逐项实机运行验证；杯子示例路径需要替换为实际资产。

本机 `env_isaaclab` 已配置 Vulkan 驱动描述路径，以避免系统中重复 NVIDIA ICD 注册；激活该环境即可应用。环境还有一项已知依赖元数据差异：为兼容仓库要求的 Starlette，安装了新版 FastAPI，与 Isaac Sim 固定要求的旧版本不同。基础仿真验证已通过，HTTP 服务未验证。详细安装记录位于：

```text
/home/agiuser/anaconda3/envs/env_isaaclab/isaaclab-install-notes.txt
```
