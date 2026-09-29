# 仿真可视化与机器人控制

本目录提供 Isaac Lab 无显示屏仿真、多相机网页可视化，以及通过 Python 下发关节动作序列的接口。网页和外部控制程序使用同一个仿真实例，不需要分别启动两套仿真。

## 1. 启动仿真服务

在仓库根目录执行，以下两种方式选择一种。

### 仿真可视化 + 外部程序控制

```bash
cd /home/agiuser/workspace/dev_zlq/IsaacLab
bash scripts/demos/web_control/start_simulation.sh
```

加载机器人、场景、物理和相机，同时启动 RobotServer。网页保留画面、状态、暂停、单步、重置和快照；关节与夹爪手动按钮禁用。关闭浏览器不影响仿真或程序控制。

机器人、场景、端口、图片尺寸和刷新率可以直接修改 [start_simulation.sh](start_simulation.sh) 顶部配置。默认使用 Cobot Magic、8080 端口、480×360 图片、3 FPS。场景配置留空时使用机器人默认场景。

命令行可以覆盖配置：

```bash
bash scripts/demos/web_control/start_simulation.sh --robot x5_dual --port 8081
```

### 网页手动控制 + 外部程序控制

```bash
bash scripts/demos/web_control/start_web_control.sh
```

网页允许操作关节与夹爪，也提供相同的 Python 控制接口。执行 action chunk 期间会禁用网页关节操作，避免两种控制同时修改目标。

默认配置在 [start_web_control.sh](start_web_control.sh) 顶部修改。两个启动脚本都会自动激活 Conda 环境 `env_isaaclab`，并启用无显示屏运行与相机渲染。按 `Ctrl+C` 停止服务；切换启动模式时先停止旧服务。

### 远程连接

在自己的电脑上建立 SSH 隧道，端口与服务一致：

```bash
ssh -N -L 8080:127.0.0.1:8080 agiuser@服务器地址
```

随后打开 `http://127.0.0.1:8080`。运行在本机的 Python 客户端也连接此地址。服务仅监听服务器 localhost，不通过服务器 IP 直接访问。

## 2. 启动外部控制程序

先启动上述任意一个仿真服务，等待就绪，再打开另一个终端：

```bash
source ~/anaconda3/bin/activate env_isaaclab
cd /home/agiuser/workspace/dev_zlq/IsaacLab

# 只读取观测、保存图片，不移动机器人
python scripts/demos/web_control/examples/joint_chunk.py

# Cobot Magic：前左臂 joint1 在当前位置基础上移动 0.05 rad
python scripts/demos/web_control/examples/joint_chunk.py \
  --joint fl_joint1 --delta 0.05
```

示例默认连接 `http://127.0.0.1:8080`，可用 `--url` 修改地址。默认图片保存到当前工作目录下的 `output/robot_observation/`。不指定 `--joint` 时只读取观测；指定后才执行运动。其它机器人需使用其自身的关节名称。

编写控制程序时参考 [examples/joint_chunk.py](examples/joint_chunk.py)，通过 [robot_client.py](robot_client.py) 中的 `RobotClient` 调用即可。客户端只依赖 Python 标准库，不需要安装 Isaac Sim；也可以将该文件复制到自己的项目。

### 最小调用示例

在仓库根目录设置模块搜索路径：

```bash
export PYTHONPATH="$PWD/scripts/demos/web_control:$PYTHONPATH"
```

然后在 Python 程序中调用：

```python
from robot_client import RobotClient

robot = RobotClient("http://127.0.0.1:8080")

# 最近一帧：多视角 JPEG bytes，以及同次采集的关节状态
obs = robot.get_observation()
print(obs.sim_time, obs.joint_names, obs.positions)
print(obs.images.keys())

# 最新物理步关节状态，不受图片刷新频率限制
state = robot.get_joint_state()
print(state["controllable_joint_names"])

# 保持第一个可控关节的当前实际位置；真实轨迹由调用端生成
name = state["controllable_joint_names"][0]
index = state["joint_names"].index(name)
q = state["positions"][index]
task = robot.execute_joint_chunk(
    joint_names=[name],
    positions=[[q], [q], [q]],  # [T, N]：T 个时间点、N 个关节
    steps_per_action=12,
)
result = robot.wait(task.id, timeout=120)
print(result.status, result.details)
```

### 动作执行规则

- 每行是绝对关节位置目标，顺序与 `joint_names` 一致。旋转关节使用弧度，移动关节使用米。
- 服务端不插值、不平滑、不自动限速、不裁剪目标。插值策略由调用端决定；自带运动示例中的插值也在调用端完成。
- 每行保持 `steps_per_action` 个物理步。当前步长为 `1/120` 秒，12 步对应 0.1 仿真秒；墙钟时间可能因渲染速度更长。
- 到时间就切换下一行，不等待关节到位；结束后保持最后目标。未指定的关节保持已有的已施加目标。
- `completed` 表示整段目标已完成指定物理步数，不保证实际位置精确到位。驱动力、惯量、负载和接触都可能造成跟踪误差，即使目标没有超限。
- 终态结果包含 `actual_positions`、`targets` 和 `errors`（目标减实际），调用端可以据此决定是否继续保持或调整动作。
- 同时只执行一个 chunk，忙时拒绝新任务。暂停冻结进度，重置取消任务。`robot.cancel(task.id)` 取消后保持最后目标，不会瞬间停止真实运动。
- `wait()` 超时或客户端断开不会取消执行。提交请求网络超时时先检查活动任务，避免盲目重复提交。

完整接口、限制、异常处理和验证说明见 [RobotServer 中文文档](../../../docs/robot_server_zh.md)。

多机器人相机读取与关节 chunk 控制的复测方法见 [RobotServer 文档](../../../docs/robot_server_zh.md#多机器人复测)。

## 3. Python 文件职责

| 文件 | 职责 | 使用方式 |
|---|---|---|
| [run.py](run.py) | 仿真主程序：加载机器人和场景、创建相机、推进物理、下发目标、发布观测 | 由启动脚本运行 |
| [robot_server.py](robot_server.py) | `RobotServer`：校验和执行 chunk、记录进度、结果和误差 | 主程序内部使用 |
| [robot_client.py](robot_client.py) | `RobotClient`：外部程序读取观测、提交动作、查询和取消任务 | 在自己的控制代码中导入 |
| [robot_api.py](robot_api.py) | 将 RobotServer 功能暴露为 HTTP 接口 | HTTP 服务内部使用 |
| [server.py](server.py) | HTTP 服务：网页、图片流、快照、控制接口和 Robot API 注册 | 主程序内部使用 |
| [state.py](state.py) | 线程安全状态管理：关节状态、相机快照、网页指令、暂停与重置；RobotServer 的基类 | 服务内部使用 |
| [scene_config.py](scene_config.py) | 读取并校验场景 JSON | 主程序内部使用 |
| [scene_assets.py](scene_assets.py) | 按场景配置创建桌子、瓶子等物体资产 | 主程序内部使用 |

## 4. 子目录职责

| 目录或文件 | 用途 |
|---|---|
| [robot/](robot/) | 机器人配置：Franka、A2D、单/双 X5、Cobot Magic；定义关节分组、夹爪映射、相机安装等 |
| [robot/convert_cobot_magic.py](robot/convert_cobot_magic.py) | Cobot Magic 资产转换工具，重新生成 USD 时使用，日常启动不需要执行 |
| [robot/cobot_materials.py](robot/cobot_materials.py) | Cobot Magic 材质修复工具，解决机械臂颜色被统一浅色覆盖的问题 |
| [examples/](examples/) | 外部程序调用示例，目前提供 `joint_chunk.py` |
| [tests/](tests/) | 单元测试及真实仿真验证；`check_*live.py` 可能会移动机器人 |
| [scenes/](scenes/) | 桌子、瓶子、机器人位置等场景 JSON |
| [static/](static/) | 网页 HTML、CSS 和 JavaScript |

机器人 USD 等资产放在仓库原生资产目录 `source/isaaclab_assets/data/Robots/`。本目录的 `robot/` 主要管理网页与程序控制所需的机器人配置。

## 5. 调用关系

```text
start_simulation.sh / start_web_control.sh
                    ↓
                  run.py
          ┌─────────┴─────────┐
     Isaac Lab 仿真      RobotServer
                             ↑
                       HTTP 服务/API
                       ↑           ↑
                    网页      RobotClient
                                  ↑
                          你的控制程序
```

日常使用主要关注启动脚本、`robot_client.py` 和 `examples/joint_chunk.py`。

## 6. 法兰 EEF 控制与 FK/IK

ARX 单/双 X5、AgileX Cobot Magic 和 Franka 支持 `get_arms()`、`fk()`、`ik()`、`execute_eef_chunk()`。法兰位姿相对各自单臂底座，四元数 wxyz，不包含夹爪 TCP。启动时自动加载资产目录中的 `kinematics.py`。公共运动学代码在 `kinematics.py`，控制集成在 `eef_control.py`。

调用示例、精度定义及限制见 [EEF 运动学文档](../../../docs/robot_eef_kinematics_zh.md)。

## 7. 控制与渲染质量

默认采用重力补偿控制、柔光场景、480×360、JPEG 95 和 DLAA。可用 `--control-quality legacy`、`--visual-quality legacy` 做对照；显存充足时可提高尺寸或用 `--hdri` 指定环境纹理。实现与实测限制见 [控制与渲染质量记录](../../../docs/control_render_quality_zh.md)。

## 8. HDF5 自动采集

`start_collection.sh` 启动按仿真时间 30 FPS 的采集模式。调用 `RobotClient.start_recording(instruction)` / `stop_recording(...)` 保存关节状态、动作记录和多相机 JPEG 序列；`examples/collect_episode.py` 可以自动执行动作清单并录制。说明及 HDF5 结构见 [采集文档](../../../docs/hdf5_collection_zh.md)。实际墙钟 FPS 受渲染性能影响。

HDF5 图像回放：`python scripts/demos/web_control/examples/replay_hdf5.py episode.hdf5 --output replay.html`。浏览器直接显示原始 JPEG；这与重新执行物理仿真不同，后者当前不保证严格一致，详见采集文档。

### 8.1 从零开始采集

在仓库根目录打开终端一，选择一个本体启动：

```bash
cd /home/agiuser/workspace/dev_zlq/IsaacLab
bash scripts/demos/web_control/start_collection.sh --robot cobot_magic
```

`--robot` 可选 `x5`（ARX 单臂）、`x5_dual`（ARX 双臂）、`cobot_magic`（AgileX 四臂）、`franka`、`a2d`（AgiBot）。每次只启动一个服务，等待 `WEB_CONTROL_READY`。默认端口 8080，输出在服务器 `output/datasets/`；使用 `--recording-dir /path/to/datasets` 可修改。

终端二激活 Python 环境：

```bash
source ~/anaconda3/bin/activate env_isaaclab
cd /home/agiuser/workspace/dev_zlq/IsaacLab
export PYTHONPATH="$PWD/scripts/demos/web_control:$PYTHONPATH"
```

以下代码适用于上述本体：先查询可控关节，选择非夹爪的一个关节，小幅移动并返回；轨迹插值在调用端完成。

```python
from robot_client import RobotClient

robot = RobotClient('http://127.0.0.1:8080', timeout=90)
state = robot.get_joint_state()
name = next(n for n in state['controllable_joint_names']
            if n.endswith('joint1') and 'finger' not in n and 'hand' not in n)
index = state['joint_names'].index(name)
q = state['positions'][index]
low, high = state['limits'][index]
delta = 0.04 if q + 0.04 < high else -0.04
assert low <= q + delta <= high
rows = [[q + delta * i / 30] for i in range(31)]
rows += rows[-2::-1]  # 返回起点

episode = robot.start_recording('机械臂小幅运动后返回初始位置')
reason = 'execution_error'
try:
    task = robot.execute_joint_chunk([name], rows, steps_per_action=12)
    result = robot.wait(task.id, timeout=300)
    reason = 'actions_completed' if result.status == 'completed' else result.status
finally:
    saved = robot.stop_recording(episode['id'], success=False, reason=reason)
    print(saved)
    if saved['error']:
        raise RuntimeError(saved['error'])
```

返回的 `path` 是服务器上的 HDF5 路径。`success=False` 表示未标注任务语义成功，不等于文件损坏；检查 `error is None` 和文件属性 `complete=True`。录制期间可执行 joint / EEF 控制；A2D 当前只支持 joint 控制。按 `Ctrl+C` 停止仿真前先结束录制。

### 8.2 动作清单自动采集

将以下内容保存为 `/tmp/collect.json`（例子只适用于 Cobot）：

```json
{
  "instruction": "前左臂转动后返回",
  "actions": [
    {"type": "joint", "joint_names": ["fl_joint1"], "positions": [[0.0], [0.02], [0.04], [0.02], [0.0]], "steps_per_action": 120}
  ]
}
```

```bash
python scripts/demos/web_control/examples/collect_episode.py /tmp/collect.json
```

该脚本自动录制整个动作过程，异常也会关闭文件。此例位置为绝对角度；其它机器人请先查询关节名称与限位再填写。`instruction` 不会自动转换成动作。

### 8.3 导出 MP4 和交互式回放

无需启动仿真，只需已采集的 HDF5：

```bash
# 所有相机拼接成一段 MP4，默认按仿真时间导出 30 FPS
python scripts/demos/web_control/examples/export_hdf5_mp4.py \
  output/datasets/episode_ID.hdf5 --output /tmp/episode_multiview.mp4

# 只导出场景相机
python scripts/demos/web_control/examples/export_hdf5_mp4.py \
  output/datasets/episode_ID.hdf5 --camera scene --output /tmp/episode_scene.mp4

# HTML：多视角、播放暂停、拖动进度，使用原始 JPEG 字节
python scripts/demos/web_control/examples/replay_hdf5.py \
  output/datasets/episode_ID.hdf5 --output /tmp/episode.html
```

MP4 导出依赖 `ffmpeg`、`h5py`、NumPy 和 Pillow，本机环境已具备。H.264 使用有损编码，因此 MP4 像素不保证与原 JPEG 解码逐位一致；原始图片以 HDF5 为准。非规则输入时间轴导出固定帧率视频时会保持最近真实帧，不能将输出 FPS 视为采集 FPS；30 FPS 仿真采集的连续序列正常逐帧对应。不会覆盖已有输出文件。

将 MP4/HTML 下载到本机播放器或浏览器观看。拼接视频每格显示相机名称与仿真时间。当前导出用于图像可视化，关节数值仍保存在 HDF5 中；动作重新仿真复现与视频回放的区别见上方采集文档。

## 9. Franka 自动规划抓取（cuRobo）

已完成独立场景的真实物理抓取：接近、下降、闭合、抬升约18 cm、保持、放回、张开、撤离。运动路径由 cuRobo 规划，检查桌面与自碰撞；夹持通过物理接触，无固定约束绑定物体。参考 RoboTwin 的“规划成功 + 物理成功判定 + 保存轨迹/录制”流程。

- [环境安装与复现说明](../../../docs/franka_planned_grasp_zh.md)

cuRobo 已安装在 env_isaaclab。为节省显存，先离线规划，规划进程退出后再启动仿真：

```bash
source ~/anaconda3/bin/activate env_isaaclab
cd /home/agiuser/workspace/dev_zlq/IsaacLab
export CUDA_HOME="$CONDA_PREFIX"
mkdir -p output/franka_planned_grasp
python scripts/demos/web_control/planning/plan_franka.py \
  --request scripts/demos/web_control/planning/franka_grasp_request.json \
  --output output/franka_planned_grasp/plan.json
bash scripts/demos/web_control/planning/start_franka_grasp.sh
```

另一个终端激活同一环境，在仓库根目录执行：

```bash
python scripts/demos/web_control/planning/execute_franka_grasp.py \
  --plan output/franka_planned_grasp/plan.json \
  --output output/franka_planned_grasp/new_result.json
```

要求专用场景处于初始状态，脚本检查机器人及方块初始位置。完成后输出记录文件路径，再用第8节的导出工具生成新视频。本示例是已知桌面/方块场景，不是开放场景视觉抓取系统；没有通用载物避碰模型或随机物体抓取适配。

## 10. ARX / AgileX 自动规划抓取采集

现在可用 cuRobo 离线规划，再通过 RobotClient 执行抓取并采集 HDF5。已实测成功：X5 单臂、双 X5 左臂、Cobot Magic 前左臂。三条多视角 MP4 分别保存在 `output/x5_planned_grasp/`、`output/x5_dual_planned_grasp/`、`output/cobot_planned_grasp/`；对应 `result.json` 给出 HDF5 路径。

完整命令、结果和适用范围见 [ARX / AgileX 自动规划抓取教程](../../../docs/arx_agilex_planned_grasp_zh.md)。最小使用流程（仓库根目录、env_isaaclab 环境）：

```bash
export CUDA_HOME="$CONDA_PREFIX"
python scripts/demos/web_control/planning/plan_grasp.py --robot x5 --output output/my_grasp/plan.json
bash scripts/demos/web_control/planning/start_grasp.sh x5
# 另一终端，等待 WEB_CONTROL_READY 后：
python scripts/demos/web_control/planning/execute_grasp.py --plan output/my_grasp/plan.json --output output/my_grasp/result.json
```

更换本体时，规划和启动参数必须一致：`x5_dual --arm left` 对应启动 `x5_dual --grasp-arm left`；`cobot_magic --arm fl` 对应启动 `cobot_magic --grasp-arm fl`。每次采集需重启专用场景。先规划，待进程退出释放显存后启动仿真。默认端口 8083。

成功按真实物体抬升、保持和放回高度判定；不是仅检查轨迹是否下发完成。图像采用仿真时钟 30 FPS。这是选定单臂在固定场景的示例，尚不支持多臂协同规划，也未完成全机身/其它手臂/被抓物体的完整避碰。
