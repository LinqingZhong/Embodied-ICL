# 仿真可视化与机器人控制

在同一个 Isaac Lab 仿真实例中运行机器人、物理与多相机渲染，通过浏览器查看画面，或通过 Python 控制关节、求解 FK/IK、执行抓取并采集 HDF5。本目录控制仿真模型，不连接实体机器人。

目录：[启动与网页](#启动与网页) · [机器人与资产](#机器人与资产) · [Python-关节控制](#python-关节控制) · [法兰-FKIK](#法兰-fkik) · [场景与物体](#场景与物体) · [接入机器人与相机](#接入机器人与相机) · [采集与回放](#采集与回放) · [抓取规划](#抓取规划) · [验证与排错](#验证与排错) · [代码导航](#代码导航)

## 启动与网页

以下命令均在仓库根目录执行。服务器需要已安装 Isaac Lab 的环境和兼容 GPU；`--headless` 省去 GUI 窗口，相机渲染仍使用 GPU。

```bash
cd /home/agiuser/workspace/dev_zlq/IsaacLab

# 查看仿真 + 外部 Python 控制（observe 模式）
bash scripts/demos/web_control/start_simulation.sh

# 或：网页手动控制 + 外部 Python 控制（web 模式）
bash scripts/demos/web_control/start_web_control.sh
```

两种方式选择一种，等待日志中的 `WEB_CONTROL_READY` 后操作。启动脚本自动定位仓库、激活 `~/anaconda3` 下的 `env_isaaclab`，并开启 `--headless --enable_cameras`。可通过 `ISAACLAB_CONDA_ROOT`、`ISAACLAB_CONDA_ENV` 更换 Conda 位置和环境。首次加载云资产或编译着色器可能较慢；按 `Ctrl+C` 停止服务。

默认配置可修改启动脚本顶部，命令行参数优先：

```bash
bash scripts/demos/web_control/start_simulation.sh --robot x5_dual --port 8081
bash scripts/demos/web_control/start_web_control.sh --robot a2d --width 320 --height 240
```

| 参数 | 启动脚本默认值 | 含义 |
|---|---|---|
| `--robot` | `cobot_magic` | 机器人配置，见下表 |
| `--port` | `8080` | 本地 HTTP 端口 |
| `--width` / `--height` | `480` / `360` | 每路 RGB 图片尺寸，最大 1920×1080 |
| `--camera-fps` | `3` | 捕获频率，最大 30；不保证达到相同墙钟 FPS |
| `--capture-clock` | `wall` | 按墙钟或 `simulation` 仿真时间采样 |
| `--scene-config` | 机器人默认场景 | 场景 JSON；修改后重启生效 |
| `--output-dir` | `output/web_control` | 快照 ZIP 保存目录 |
| `--recording-dir` | `output/datasets` | HDF5 保存目录 |
| `--control-quality` | `compensated` | 重力补偿控制；`legacy` 使用原配置 |
| `--visual-quality` | `studio` | 柔光场景；`legacy` 使用原光照 |
| `--jpeg-quality` | `95` | JPEG 质量，1～100 |
| `--hdri` | 未设置 | 可选本地或远程环境纹理 |
| `--duration` | `0` | 就绪后运行的墙钟秒数；0 表示持续运行 |

直接运行 [run.py](run.py) 时默认是 `franka`、`web`、10 FPS；启动脚本会覆盖这些值。手动启动示例：

```bash
source ~/anaconda3/bin/activate env_isaaclab
./isaaclab.sh -p scripts/demos/web_control/run.py \
  --headless --enable_cameras --robot franka --camera-fps 3
```

### 远程连接与网页操作

在自己的电脑建立 SSH 隧道，端口与服务一致，然后访问 `http://127.0.0.1:8080`：

```bash
ssh -N -L 8080:127.0.0.1:8080 agiuser@服务器地址
```

Python 客户端也连接此地址。服务只监听回环地址；关闭浏览器或断开客户端不会停止仿真，机器人保持最后有效目标。多个浏览器共享同一个实例。

| 功能 | 行为 |
|---|---|
| 相机与状态 | 显示各视角、实际关节位置、已施加目标与画面新鲜度；断线时禁用手动控件并自动重连 |
| 关节、夹爪 | 仅 web 模式允许手动操作；目标按 profile 的 `speed` 渐进变化；程序执行 chunk 期间拒绝手动运动 |
| 暂停、单步 | 暂停物理步进；单步推进 1 个物理步，当前为 1/120 秒 |
| 重置场景 | 恢复机器人与动态物体初态，取消活动 chunk；录制或 IK 求解期间拒绝重置 |
| 保存并下载 | 保存同次采集的全部 `<相机名>.jpg` 与 `metadata.json` 到独立 ZIP |

快照包含捕获序号、时间、关节名称、位置、速度和目标。当前网页与录制器输出 RGB JPEG；深度采集需按[相机教程](../../tutorials/04_sensors/run_usd_camera.py)扩展，原始浮点深度应另存为 `.npy` 等格式。

### 控制与渲染质量

默认 `compensated` 对 X5、Cobot Magic 和 Franka 启用真实重力与 PhysX 重力补偿前馈，保留隐式 PD 和原力矩上限，并为 PD 预留补偿后剩余的力矩预算。Franka 使用 Kp/Kd=400/40；A2D 沿用原控制。覆盖仅发生在本 demo 中，详见 [simulation_quality.py](simulation_quality.py)。接触、载荷、惯量和限幅仍会产生跟踪误差。

默认渲染使用柔光、JPEG 95、4:4:4 和 DLAA。提高尺寸或设置 `--hdri` 后需重启；多相机增加显存与编码开销，建议先用默认分辨率。对照原配置可传 `--control-quality legacy --visual-quality legacy --jpeg-quality 80`。仿真时间可能慢于墙钟时间，该接口不提供硬实时控制。

## 机器人与资产

| `--robot` / profile | 直接控制关节与夹爪 | 相机 | FK/IK 的 `arm` |
|---|---|---|---|
| `franka` / [franka.py](robot/franka.py) | 7 轴；双指夹爪 | `scene`、`wrist` | `arm` |
| `a2d` / [a2d.py](robot/a2d.py) | 18 轴：升降、躯干、头部和双臂；左右夹爪 | `scene`、`head`、`left_wrist`、`right_wrist` | 不支持，使用 joint 控制 |
| `x5` / [x5.py](robot/x5.py) | 6 轴；双指夹爪 | `scene`、`wrist` | `arm` |
| `x5_dual` / [x5_dual.py](robot/x5_dual.py) | 两套 6 轴臂；两个独立夹爪 | `scene`、`left_wrist`、`right_wrist` | `left`、`right` |
| `cobot_magic` / [cobot_magic.py](robot/cobot_magic.py) | 四套 6 轴臂、支架 2 轴；四个夹爪 | `scene`、`head`、`fl_wrist`、`fr_wrist` | `fl`、`fr`、`bl`、`br` |

模型通过 `ISAACLAB_ASSETS_DATA_DIR` 定位。X5 和 Cobot 的 USD、引用层与网格随 Git LFS 管理，克隆后执行 `git lfs pull`；迁移资产时保留完整目录。Franka、A2D 和部分桌子/瓶子使用 NVIDIA 云资产，首次运行需要访问对应资源。

### ARX X5 单臂与双臂

资产位于 [ARX/X5](../../../source/isaaclab_assets/data/Robots/ARX/X5/)，来源为 [RoboDojo 数据集](https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/tree/main/Assets/Robots/x5)，附下载清单和 `ROBODOJO_LICENSE`。通用机器人配置是 [ARX_X5_CFG](../../../source/isaaclab_assets/isaaclab_assets/robots/arx.py)；可用环境变量 `ARX_X5_USD` 指定另一套完整资产的入口 USD。

单臂控制 `joint1`～`joint6`；夹爪 `joint7`、`joint8` 同向张开至 0.044 m，闭合为 0，不使用外部配置中的负开度。双臂 API 关节名带实例前缀，如 `left/joint1`、`right/joint1`，夹爪名为 `left`、`right`。

双臂由两个固定基座实例组成，默认位置为 `[-0.3,-0.45,0.765]` 和 `[0.3,-0.45,0.765]`，均绕 Z 旋转 90°。场景 `robot_position` 是共享平移偏移，叠加到实例位置。默认桌面布局见 [x5_dual_table.json](scenes/x5_dual_table.json)。独立控制双臂不包含协同避碰规划。

### AgileX Cobot Magic

资产来自 `mobile_aloha_sim-2.0.0` 的 `aloha_tracer2_d435_dark.urdf`，目录为 [AgileX/CobotMagic](../../../source/isaaclab_assets/data/Robots/AgileX/CobotMagic/)。通用配置是 [COBOT_MAGIC_CFG](../../../source/isaaclab_assets/isaaclab_assets/robots/agilex.py)，默认场景为 [cobot_magic.json](scenes/cobot_magic.json)。底盘固定，轮子不作为位置控制关节。

`fl/fr/bl/br` 分别代表前左、前右、后左、后右。各夹爪的 `joint7` 张开至 +0.04 m，`joint8` 至 -0.04 m，闭合均为 0。初始各臂 joint2=1 rad、joint3=-1.2 rad。源模型自碰撞关闭，保留与外部物体的接触；驱动参数为仿真调试值，不能视为厂商标定。

需要重新生成 USD 时，在激活的环境中执行：

```bash
./isaaclab.sh -p scripts/demos/web_control/robot/convert_cobot_magic.py
```

专用转换脚本使用本机 Isaac Sim 5.1 已有的 URDF 导入器，保留固定连杆，并调用 [cobot_materials.py](robot/cobot_materials.py)，让机械臂网格的黑、红、灰材质优先于统一浅色覆盖。输出 `CONVERTED` 后重启仿真才能加载新模型。部分源相机外壳可能仍有引用或遮挡问题。

### AgiBot A2D 与 Franka

A2D 使用仓库 `AGIBOT_A2D_CFG`，基座固定用于上半身调试。升降单位为米，其余直接关节为弧度；头部连续偏航关节在面板中限制为 ±π/2。左右夹爪各联动三个驱动/支撑关节：张开 0.994 rad，左闭合 0、右闭合 0.2 rad；不要将其当作毫米开度或遗漏 Support 关节。默认场景为 [a2d_table_bottle.json](scenes/a2d_table_bottle.json)，与 Franka 桌面高度不同。

Franka 使用仓库 `FRANKA_PANDA_CFG`；手指单侧张开目标为 0.04 m、闭合为 0。Franka 没有头部，`scene` 是固定世界视角，`wrist` 安装于 `panda_hand`。各机器人相机均为新增仿真 RGB 传感器，安装参数不代表实体相机标定。

## Python 关节控制

服务器启动后，在另一个终端激活环境并设置模块路径；远程客户端只需复制 [robot_client.py](robot_client.py)，它仅依赖 Python 标准库。

```bash
source ~/anaconda3/bin/activate env_isaaclab
cd /home/agiuser/workspace/dev_zlq/IsaacLab
export PYTHONPATH="$PWD/scripts/demos/web_control:$PYTHONPATH"

# 只读取观测、保存图片；指定 --joint 后才移动
python scripts/demos/web_control/examples/joint_chunk.py
python scripts/demos/web_control/examples/joint_chunk.py --joint fl_joint1 --delta 0.05
```

示例默认连接 8080，可用 `--url` 修改；图片保存在调用端的 `output/robot_observation/`。不同机器人必须使用各自的关节名。

```python
from robot_client import RobotClient

robot = RobotClient("http://127.0.0.1:8080", timeout=90)
state = robot.get_joint_state()
print(state["controllable_joint_names"])

obs = robot.get_observation()
print(obs.sequence, obs.sim_time, obs.captured_at)
print(obs.images.keys())  # 每路为 JPEG bytes

# 保持第一个可控关节的当前角度；可替换为调用端生成的轨迹
name = state["controllable_joint_names"][0]
i = state["joint_names"].index(name)
q = state["positions"][i]
task = robot.execute_joint_chunk([name], [[q], [q], [q]], steps_per_action=12)
result = robot.wait(task.id, timeout=120)
print(result.status, result.details)
```

`get_observation()` 返回最近一帧的全部相机与同次采集的关节状态；不会等待新帧，可用 `sequence` 去重、`captured_at` 判断新鲜度。`get_joint_state()` 返回最新物理步状态，通常更新得更快。`positions` 是实测值，`targets` 是已施加的位置目标；`limits` 与 `joint_names` 对齐，被动连续关节的无限边界可能为 `null`。

### Chunk 的执行规则

- `positions` 形状为 `[T,N]`，列顺序由 `joint_names` 指定；旋转关节用弧度，移动关节用米。仅允许可控关节，整段先校验名称、维度、有限数值和限位。
- 每行直接下发绝对位置目标，保持 `steps_per_action` 个物理步；服务端不插值、不平滑、不自动限速或裁剪。需要平滑运动时由调用端生成中间点。
- `physics_dt` 当前为 1/120 秒，12 步对应 0.1 仿真秒。时间到就切换下一行，不等待到位；未指定的关节保持此前已施加目标。
- 一次只运行一段，不排队；最多 2048 行，每行 1～120000 步，请求上限 2 MiB。暂停时可以提交，但进度冻结；单步推进一个物理步。
- `completed` 仅表示最后一行已执行足够步数。终态的 `actual_positions`、`targets`、`errors`（目标减实际）来自最终物理步，顺序与任务关节名一致。继续保持后的反馈需重新读取状态。
- `robot.cancel(task.id)` 取消后保持最后目标，不会瞬间停止真实运动；重置取消任务。服务器保留最近 64 个任务，重启后不保留。
- `wait()` 超时或客户端断开不取消动作。提交网络超时时先查询 `get_joint_state()["active_task"]`，避免重复下发；状态可由 `get_task()` 查询。

### HTTP 接口

Python 客户端封装以下路由。直接调用时，POST 使用 JSON；FK 的 `joint_positions` 和 IK 的 `seed` 可传 `null`，字段仍需存在。

| 方法 | 路径 | 内容 |
|---|---|---|
| GET | `/api/robot/joints` | 关节、物理步长、活动任务、实测 EEF 位姿 |
| GET | `/api/robot/observation` | 同帧关节状态与 Base64 JPEG |
| POST | `/api/robot/chunks` | `joint_names`、`positions`、`steps_per_action` |
| GET | `/api/robot/chunks/{id}` | 任务状态、步数与终态误差 |
| POST | `/api/robot/chunks/{id}/cancel` | 空对象 `{}` |
| GET | `/api/robot/arms` | 支持的手臂与运动学元数据 |
| POST | `/api/robot/fk` | `arm`、`joint_positions` |
| POST | `/api/robot/ik` | `arm`、`position`、`quaternion`、`seed` |
| POST | `/api/robot/eef/chunks` | `arm`、`positions`、`quaternions`、`steps_per_action` |
| POST | `/api/robot/recording/start` | `instruction` |
| POST | `/api/robot/recording/stop` | `episode_id`、`success`、`reason` |
| GET | `/api/robot/recording` | 当前录制状态 |

常见状态码：400 参数错误、404 未知或过期任务、409 忙或当前状态不允许、413 请求过大、503 观测尚未就绪。客户端通过 `RobotError.status` 获取状态码，通过 `str(error)` 获取错误说明。

## 法兰 FK/IK

位姿定义为**单臂底座到末端法兰**的变换，位置单位米、角度单位弧度，四元数为单位四元数 **wxyz**；不包含夹爪中心或额外 TCP。

| 配置 | 底座 | 法兰 |
|---|---|---|
| X5 单/双臂 | 对应实例 `base_link` | `link6` |
| Cobot Magic | 对应 `fl/fr/bl/br_base_link` | 对应 `*_link6` |
| Franka | `panda_link0` | `panda_link8`，即 link7 沿 Z 偏移 0.107 m，位于手掌旋转之前 |

[kinematics.py](kinematics.py) 从已加载 USD 的关节轴和局部变换构建运动链，使用 NumPy/SciPy 求解。资产目录内的 `kinematics.py` 声明对应链。启动时与物理姿态对照，成功后打印 `KINEMATICS_VERIFIED`；不一致则拒绝启动。

以下示例连接 Cobot；双 X5 用 `left/right`，单 X5 和 Franka 用 `arm`：

```python
from robot_client import RobotClient

robot = RobotClient("http://127.0.0.1:8080", timeout=90)
print(robot.get_arms())  # base、flange、joint_names、limits、容差
arm = "fl"
pose = robot.fk(arm)  # 默认使用当前实测关节角
solution = robot.ik(arm, pose["position"], pose["quaternion"])
reconstructed = robot.fk(arm, solution["joint_positions"])
task = robot.execute_eef_chunk(
    arm, [pose["position"]], [pose["quaternion"]], steps_per_action=120,
)
print(robot.wait(task.id, timeout=120).details)
```

`fk(arm, joint_positions)` 按 `get_arms()` 的关节顺序计算；`ik(..., seed=q)` 可指定初值，默认使用当前实测关节角并处理短暂软限位越界，显式越界 seed 仍拒绝。IK 返回关节解、位置误差和姿态误差，接受阈值为 1e-4 m、1e-3 rad。

EEF chunk 最多 128 行，位置与姿态行数一致。首点以当前状态为初值，后续以前一解为初值；全部求解成功后才下发 joint chunk，任一点失败整段拒绝并返回行号。求解期间保留控制权，仿真继续推进但不下发部分结果。任务包含 `control=eef`、`arm` 和 `solved_joint_positions`。

求解不做插值、碰撞规划或到位等待；一次 chunk 控制一只手臂。需要多臂同步时，可分别求 IK，再合并成一个 joint chunk。数值 IK 可能因初值、奇异点或限位失败，也可能切换分支；调用端应检查关节序列。Franka 有冗余自由度，应比较 FK→IK→FK 的位姿误差，而非要求关节角唯一。

`get_joint_state()["eef_poses"]` 是物理引擎独立测量的法兰位姿。数值求解容差、FK/物理一致性和实际目标跟踪误差是三项不同指标，IK 收敛不保证物理到位。

## 场景与物体

复制 [scenes/](scenes/) 中适合当前机器人的 JSON，再通过 `--scene-config /绝对路径/my_scene.json` 加载。JSON 显式值覆盖 profile 的 `scene_defaults`；机器人基座、桌面和相机位置应配套调整。

下面展示一个自定义物体场景；需把路径和坐标替换为真实资产配置：

```json
{
  "ground_z": 0,
  "robot_position": [0, 0, 0],
  "scene_eye": [1.6, 1.4, 1.2],
  "scene_target": [0.4, 0, 0.2],
  "wrist_target": [0.4, 0, 0.2],
  "assets": [
    {
      "name": "Cup",
      "usd_path": "assets/cup.usd",
      "mode": "dynamic",
      "position": [0.4, 0, 0.2],
      "orientation": [1, 0, 0, 0],
      "scale": [1, 1, 1]
    }
  ]
}
```

本地 `usd_path` 相对于 **JSON 所在目录**；也支持 HTTP(S)、Omniverse 和 `isaac://Props/...`（映射到 `ISAAC_NUCLEUS_DIR`）。资产名称需唯一，使用字母、数字、下划线且不能以数字开头。位置以米计，朝向为 wxyz 并自动归一化，缩放必须为正数。未知字段、错误数值或缺失本地文件会在仿真启动前报错。

| `mode` | 要求与效果 |
|---|---|
| `visual`（默认） | 普通装饰物；禁用已有刚体与碰撞 |
| `static` | 固定桌子等；USD 必须有碰撞体，已有刚体设为运动学刚体 |
| `dynamic` | 受重力和接触影响；USD 必须有且仅有一个刚体及碰撞体 |

带 articulation 的机器人不能放入此数组。加载器不会为纯视觉网格自动补齐物理结构。`position` 指模型原点，不是底面；应检查尺寸、朝向与初始穿透。现有瓶子示例使用 YCB 芥末瓶，替换其它瓶子时要同时替换 USD 与放置参数。

OBJ/STL/FBX 可先转换，`--mass` 才会添加动态刚体与质量属性：

```bash
./isaaclab.sh -p scripts/tools/convert_mesh.py \
  /path/to/cup.obj /path/to/cup.usd \
  --collision-approximation convexDecomposition --mass 0.2 --headless
```

杯口和把手孔洞可能被单一 `convexHull` 填平；凸分解也需检查结果，必要时制作专用碰撞体。GLB/Blend 等先通过建模工具导出支持的格式。保留纹理和外部引用；转换工具没有 `--scale` 参数，模型单位错误应在制作阶段修正。先验证外观、掉落和碰撞，再尝试抓取。

## 接入机器人与相机

1. 准备带关节、惯量、碰撞和限位的 URDF/USD；Xacro 先展开为 URDF，核对 `package://` 网格路径。
2. 使用 [convert_urdf.py](../../tools/convert_urdf.py) 转换，再定义 `ArticulationCfg` 的执行器与初始姿态。固定基座示例：

   ```bash
   ./isaaclab.sh -p scripts/tools/convert_urdf.py \
     /path/to/robot.urdf /path/to/robot.usd \
     --fix-base --joint-stiffness 100 --joint-damping 10 \
     --joint-target-type position --headless
   ```

3. 复制 `robot/franka.py` 或 `robot/a2d.py`，定义 `PROFILE`，在 [robot/__init__.py](robot/__init__.py) 的 `PROFILES` 注册。
4. 配置合适的场景，先验证关节小幅运动，再检查场景、头部与腕部图像。需要 FK/IK 时，还要扩展 [kinematics.py](kinematics.py) 的机器人到资产描述文件映射，并添加运动链描述。

转换示例中的增益需按机器人调整；浮动基座不要使用 `--fix-base`。合并固定关节可能移除相机安装连杆，初次接入建议保留。执行器应区分手臂、头部、夹爪和轮子，被动关节不应作为主动关节驱动。直接写关节状态用于初始化/重置，日常运动通过控制目标和物理步进实现。

| `PROFILE` 字段 | 用途 |
|---|---|
| `asset_module` / `asset_config` | `ArticulationCfg` 模块和变量名，仿真启动后导入 |
| `label` / `fix_root_link` | 显示名称与固定基座设置 |
| `default_scene` / `scene_defaults` | 默认场景路径（相对本目录）与缺省场景值 |
| `instances` | 多实例名称、位置和朝向；参考双 X5 |
| `joints` | 主动关节的精确名称、分组、`unit`、手动控制 `speed` 与按钮 `step` |
| `grippers` | 命名夹爪的 `open/close` 目标映射，关节不与 `joints` 重复 |
| `cameras` | 相机列表，含名为 `scene` 的固定观察视角 |

普通关节范围读取实际软限位；连续关节必须指定有限 `limits`。位置单位为 rad/m，网页 `speed` 为每仿真秒目标变化量。RobotServer 的 joint chunk 直接执行调用端目标，不应用网页的渐进限速。

相机 `body` 指实际运动连杆；A2D 头部为 `link_pitch_head`，腕部为 `left_base_link/right_base_link`。`eye_offset` 是初始世界坐标偏移，`target` 是初始世界注视点；启动时根据物理连杆位姿求局部安装变换，之后随连杆运动，不持续自动瞄准物体。

精确安装时同时填写局部 `mount_pos` 和 `mount_rot`；四元数 wxyz，USD/OpenGL 相机约定为 **-Z 向前、+Y 向上**。以下仅说明字段格式，需替换真实安装值：

```python
camera = {
    "name": "head", "label": "头部相机", "body": "link_pitch_head",
    "eye_offset": [0.15, 0, 0.05],
    "mount_pos": [0.05, 0, 0.02],
    "mount_rot": [1, 0, 0, 0],
}
```

本 demo 的控制协议是位置目标，新增速度/力矩控制、底盘运动或灵巧手需要扩展协议。自行编写仿真循环时，应先创建 `AppLauncher`，再导入仿真模块；通信线程只更新指令缓存，由主线程写入控制目标、`sim.step()` 并更新机器人/相机状态。

## 采集与回放

### 启动采集与记录一段动作

```bash
bash scripts/demos/web_control/start_collection.sh --robot cobot_magic
```

该脚本使用 observe 模式，默认 **30 FPS 仿真时间**采集：120 Hz 物理每 4 步采集一次完整多相机图像，每个物理步记录关节状态，暂停时不重复生成图像。实际墙钟速度取决于渲染性能；`--camera-fps` 可覆盖，非 120 的整数约数会受物理步量化。

等待就绪后，在另一个终端按 [Python 关节控制](#python-关节控制)设置环境。以下例子选择一个 joint1，小幅移动后返回，轨迹插值在调用端完成：

```python
from robot_client import RobotClient

robot = RobotClient("http://127.0.0.1:8080", timeout=90)
state = robot.get_joint_state()
name = next(n for n in state["controllable_joint_names"] if n.endswith("joint1"))
i = state["joint_names"].index(name)
q = state["positions"][i]
low, high = state["limits"][i]
delta = 0.04 if q + 0.04 <= high else -0.04
assert low <= q <= high and low <= q + delta <= high
rows = [[q + delta * k / 30] for k in range(31)]
rows += rows[-2::-1]

episode = robot.start_recording("机械臂小幅运动后返回初始位置")
reason = "execution_error"
try:
    task = robot.execute_joint_chunk([name], rows, steps_per_action=12)
    result = robot.wait(task.id, timeout=300)
    reason = "actions_completed" if result.status == "completed" else result.status
finally:
    saved = robot.stop_recording(episode["id"], success=False, reason=reason)
    print(saved)
    if saved["error"]:
        raise RuntimeError(saved["error"])
```

支持全部现有机器人；录制中可执行 joint 或 EEF 动作，也可以只读取观测。`instruction` 是任务标签，不会自动生成动作；`success` 由调用端判定，动作执行完成不等于任务语义成功。返回 `path` 是服务器文件路径，默认 `output/datasets/episode_<id>.hdf5`；文件不覆盖，状态通过 `recording_status()` 查询。

录制期间禁止 reset。正常退出会关闭文件并标记 `simulation_stopped`，强杀可能留下不完整文件；后台队列超过 256 项时明确报错并标记 `complete=False`。停止录制不会自动取消仍在执行的动作，异常/超时后应查询任务再决定是否取消。

### 动作清单

保存 JSON 后交给 [collect_episode.py](examples/collect_episode.py)，它依次执行并自动录制、结束和检查写入错误。以下绝对角度只适用于 Cobot，执行前确认初始状态：

```json
{
  "instruction": "前左臂转动后返回",
  "actions": [
    {"type": "joint", "joint_names": ["fl_joint1"], "positions": [[0.0], [0.02], [0.04], [0.02], [0.0]], "steps_per_action": 120}
  ]
}
```

```bash
python scripts/demos/web_control/examples/collect_episode.py /path/to/actions.json
```

`type=eef` 时填写 `arm`、`positions`、`quaternions`、`steps_per_action`。该脚本默认 `success=False`，分别记录动作完成或执行异常。

### HDF5 结构

| 内容 | 含义 |
|---|---|
| attributes | `schema=isaaclab_robot_episode_v1`、`instruction`、`robot`、`metadata_json`、`complete`、`success`、`end_reason` |
| `metadata_json` | 机器人 profile、场景、启动参数、相机顺序、物理步长等 |
| `joint_names` | 关节数组的列顺序 |
| `state/*` | `sim_time`、`wall_time`、`step`、`positions`、`velocities`、`targets`，以及 `object_states_json` |
| `frames/*` | 图像的 `sim_time`、`wall_time`、`step` |
| `images/camera_0` 等 | 变长 uint8 JPEG bytes，相机名称按 metadata 中顺序对应 |
| `events/json` | 动作请求、EEF 请求、求解后的关节轨迹及任务终态 |

关节与图片使用独立时间轴，以 `frames.step` 对齐 `state.step`；step 在服务生命周期内递增，不要求每个文件从零开始。普通墙钟模式在暂停时可能记录同一步的状态与新图像，采集模式则按仿真步采样。

```python
import io
import json
import h5py
from PIL import Image

with h5py.File("episode.hdf5") as f:
    print(f.attrs["complete"], f.attrs["success"])
    metadata = json.loads(f.attrs["metadata_json"])
    q = f["state/positions"][:]
    timestamps = f["frames/sim_time"][:]
    image = Image.open(io.BytesIO(f["images/camera_0"][0].tobytes()))
```

这是自定义格式，不是 LeRobot 数据集。虽记录物体观测，仍不包含可完整恢复物理场景的快照、求解器内部状态和随机状态；重新下发相同动作不能保证关节状态或图片逐位一致。

### 导出 MP4 与 HTML

无需启动仿真。MP4 依赖 `ffmpeg`、h5py、NumPy 和 Pillow，拒绝覆盖现有文件；HTML 导出会覆盖同名文件，应选择新的输出名称：

两种导出都要求 `complete=True`；MP4 还要求至少两帧且仿真时间戳严格递增。普通墙钟模式在暂停时采集的重复时间戳会导致 MP4 导出被拒绝，视频采集建议使用 `start_collection.sh`。

```bash
mkdir -p output/videos
# 所有相机拼接；默认按仿真时间导出 30 FPS
python scripts/demos/web_control/examples/export_hdf5_mp4.py \
  output/datasets/episode_ID.hdf5 --output output/videos/episode.mp4
# 单一相机
python scripts/demos/web_control/examples/export_hdf5_mp4.py \
  output/datasets/episode_ID.hdf5 --camera scene --output output/videos/scene.mp4
# 独立 HTML，保留原 JPEG，支持多视角、播放暂停和拖动进度
python scripts/demos/web_control/examples/replay_hdf5.py \
  output/datasets/episode_ID.hdf5 --output output/videos/episode.html
```

将文件下载到本机观看。HTML 内嵌全部图片，适合短 episode；MP4 使用有损 H.264，旁边生成元数据 JSON。固定帧率导出可能保持最近真实帧，不能把视频 FPS 当作实际采集吞吐。图片回放复用已保存图像，不重新执行物理仿真。

## 抓取规划

抓取示例使用 cuRobo 在已知桌子/方块场景中离线规划，再通过 RobotClient 执行“接近、下降、闭爪、抬升、保持、放回、开爪、撤离”并录制。先运行规划进程，待其退出释放显存后再启动仿真；每次执行使用新启动的专用场景，规划参数与仿真机器人/手臂必须一致。

### 环境与 Franka

参考环境为 Python 3.11、Isaac Sim 5.1、PyTorch 2.7 cu128、CUDA Toolkit 12.8；cuRobo 使用提交 `ebb71702f3f70e767f40fd8e050674af0288abe8`，需提前安装兼容版本并设置 `CUDA_HOME`。示例使用 PRIMITIVE 碰撞检查器；无需依赖旧 `warp.torch` 网格接口。

```bash
source ~/anaconda3/bin/activate env_isaaclab
export CUDA_HOME="$CONDA_PREFIX"
mkdir -p output/franka_planned_grasp
python scripts/demos/web_control/planning/plan_franka.py \
  --request scripts/demos/web_control/planning/franka_grasp_request.json \
  --output output/franka_planned_grasp/plan.json
bash scripts/demos/web_control/planning/start_franka_grasp.sh
```

另一个已激活环境的终端中，等待就绪后执行：

```bash
python scripts/demos/web_control/planning/execute_franka_grasp.py \
  --plan output/franka_planned_grasp/plan.json \
  --output output/franka_planned_grasp/result.json
```

专用服务与执行器默认端口均为 **8083**，执行脚本可通过 `--url` 改地址。Franka 使用 0.2 m 高桌面、4 cm/50 g 方块，法兰目标由 [franka_grasp_request.json](planning/franka_grasp_request.json) 给出，规划检查桌面和自碰撞；专用夹爪增益为 800/40、力上限 20，闭爪通过接触夹持方块。

### ARX 与 AgileX

```bash
export CUDA_HOME="$CONDA_PREFIX"
python scripts/demos/web_control/planning/plan_grasp.py \
  --robot cobot_magic --arm fl --output output/my_cobot_grasp/plan.json
bash scripts/demos/web_control/planning/start_grasp.sh cobot_magic --grasp-arm fl
```

另一个终端执行：

```bash
python scripts/demos/web_control/planning/execute_grasp.py \
  --plan output/my_cobot_grasp/plan.json \
  --output output/my_cobot_grasp/result.json
```

| 机器人 | 规划 `--arm` / 仿真 `--grasp-arm` | 已有物理抓取验证范围 |
|---|---|---|
| `x5` | `arm`（默认） | 单臂 |
| `x5_dual` | `left`（默认）、`right` | 左臂；右臂配置存在，未做物理采集验证 |
| `cobot_magic` | `fl`（默认）、`fr` | 前左臂；前右配置存在，未做物理采集验证 |

一次只驱动选定手臂，其余保持原位；Cobot 后方两臂可用通用 joint/EEF 及录制接口，未加入此抓取场景。[grasp_profiles.py](planning/grasp_profiles.py) 定义底座、桌子、物体和法兰目标；X5 接触偏移沿法兰 +X 约 14 cm，Cobot 沿 +Z 约 11 cm。修改模型或场景后需要重新规划。

### 成功判定与范围

执行前检查机器人和物体初态，ARX/AgileX 还校验规划/仿真 FK 一致性。成功基于真实物体升高超过 10 cm、四次保持采样波动小于 2 cm，并回到初始高度附近；不保证最终位置/姿态完全相同。物体通过真实夹爪接触运动，不绑定或传送。

结果 JSON 包含检查结果与 `recording.path`，可按[导出说明](#导出-mp4-与-html)生成视频；失败录制标记 `success=False`。输出是本地生成文件，不随仓库分发。

这是固定场景示例，没有视觉识别、随机任务或通用抓取姿态搜索。ARX/AgileX 规划采用选中手臂的近似碰撞球与桌面，尚未完整包含其它手臂、机身、手指和被抓物体；Franka 也未建立载物避碰模型。FK 精度不等于接触稳定性或抓取成功率，实际采集前应检查腕部遮挡和多视角覆盖。

## 验证与排错

不启动 GPU 的单元与 HTTP 测试：

```bash
python -m pytest -q scripts/demos/web_control/tests
```

材质测试依赖 `pxr`，普通 Python 环境缺失时会跳过。真实仿真检查会移动关节或重置场景，应使用单独测试实例，参数与机器人一致：

```bash
# 终端一
bash scripts/demos/web_control/start_simulation.sh --robot cobot_magic --port 8083
# 终端二：相机、joint chunk、保持与返回、observe 模式限制
python scripts/demos/web_control/tests/check_robot_profiles_live.py \
  --robot cobot_magic --url http://127.0.0.1:8083 --output /tmp/cobot_check.json
# FK/IK、独立物理位姿、局部 EEF 跟踪；此脚本固定连接 8083
python scripts/demos/web_control/tests/check_eef_live.py \
  --robot cobot_magic --output /tmp/cobot_eef.json --tracking-tolerance 0.001
```

EEF 检查在每臂初态附近采样 12 组姿态，并在三个实际运动姿态对照 FK；`--tracking-tolerance 0.001` 额外要求局部跟踪小于 1 mm，不代表全工作空间或高速/接触任务保证。A2D 不运行该 EEF 检查。

其它检查入口见 [tests/](tests/)：`check_x5_live.py`、`check_x5_dual_live.py`、`check_cobot_live.py`、`check_a2d_live.py` 检查各机器人网页操作；`check_recording_profiles_live.py` 检查录制与目标重放；`check_scene_assets.py` 通过 `isaaclab.sh -p ... --headless` 启动独立物理检查。

浏览器检查固定针对 Franka 的 7 个关节和两路相机，需要 Playwright/Chromium，以及 `start_web_control.sh --robot franka` 启动的 web 模式服务：

```bash
node scripts/demos/web_control/tests/check_browser.cjs
```

可设置 `WEB_CONTROL_URL`、`WEB_CONTROL_ARTIFACTS` 和 `BROWSER_EXECUTABLE`，Node.js 需能解析 `playwright` 模块；使用独立安装目录时可设置 `NODE_PATH`。此检查也会操作关节和重置场景。

| 现象 | 检查位置 |
|---|---|
| 模型不存在或只得到 LFS 文本 | `git lfs pull`、完整 USD 引用目录、云资产访问 |
| 页面一直加载或没有图像 | `WEB_CONTROL_READY`、traceback、`--enable_cameras`、GPU 显存；无显示屏 GLFW 警告本身不等于失败 |
| 画面黑、相机不跟随或被外壳遮挡 | 父连杆、局部安装方向、裁剪范围、灯光、固定关节是否被合并 |
| 关节不动、抖动或任务完成但不到位 | 暂停状态、实际反馈、限位、力矩预算、增益、惯量与接触；勿只检查 `completed` |
| 渲染慢或显存不足 | 相机数量、尺寸、GPU 占用；先降至 320×240，FPS 仍受仿真开销限制 |
| Cobot 手臂颜色变浅 | 重跑转换/材质修复并重启服务，刷新浏览器不会重载 USD |
| HDF5 不完整 | `saved.error`、文件 `complete`、写盘队列与是否强杀；`success=False` 不等于文件损坏 |

## 代码导航

| 文件/目录 | 职责 |
|---|---|
| [run.py](run.py) | 主线程加载场景、推进物理、施加目标并发布观测 |
| [state.py](state.py)、[robot_server.py](robot_server.py) | 线程安全状态、网页指令、chunk 执行与任务结果 |
| [server.py](server.py)、[robot_api.py](robot_api.py)、[static/](static/) | HTTP、图片、快照和浏览器界面 |
| [robot_client.py](robot_client.py)、[examples/](examples/) | 外部客户端、采集、回放与导出示例 |
| [kinematics.py](kinematics.py)、[eef_control.py](eef_control.py) | 公共 FK/IK 与 EEF 控制 |
| [recording.py](recording.py) | HDF5 写盘与录制生命周期 |
| [robot/](robot/)、[scenes/](scenes/) | 机器人控制/相机配置与场景参数 |
| [scene_config.py](scene_config.py)、[scene_assets.py](scene_assets.py) | 场景校验与普通物体加载 |
| [planning/](planning/)、[grasp_scene.py](grasp_scene.py) | 离线抓取规划、专用场景和执行 |

需要从底层搭建自己的仿真时，可直接参考仓库教程：[空场景](../../tutorials/00_sim/create_empty.py)、[关节控制](../../tutorials/01_assets/add_new_robot.py)、[刚体](../../tutorials/01_assets/run_rigid_object.py)、[力矩控制](../../tutorials/01_assets/run_articulation.py)、[相机与原始深度保存](../../tutorials/04_sensors/run_usd_camera.py)、[机器人传感器](../../tutorials/04_sensors/add_sensors_on_robot.py)、[多种相机示例](../sensors/cameras.py)、[差分 IK](../../tutorials/05_controllers/run_diff_ik.py)、[操作空间控制](../../tutorials/05_controllers/run_osc.py)。
