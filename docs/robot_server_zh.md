# 仿真可视化与 RobotServer 程序控制

仿真只运行一份：加载机器人、场景、物理和多路相机。`RobotServer` 管理状态和动作执行，HTTP 服务负责传输；外部程序通过不依赖 Isaac Sim 的 `RobotClient` 使用它。浏览器关闭不影响执行。原 README 未修改。

## 启动

先停止旧服务，避免同端口冲突或显存不足，然后在仓库根目录运行：

```bash
bash scripts/demos/web_control/start_simulation.sh
```

该脚本顶部可修改机器人、端口、场景和图片参数。默认 Cobot Magic、8080、480×360、3 FPS。场景留空使用机器人配置中的默认场景。命令行可覆盖，例如：

```bash
bash scripts/demos/web_control/start_simulation.sh --robot x5_dual --port 8081
```

`observe` 模式禁用网页关节/夹爪手动操作，但保留画面、状态、暂停、单步、重置和快照。原 `start_web_control.sh` 默认仍是 `web` 手动模式，也提供相同 Robot API；执行 chunk 时服务端拒绝手动关节操作，网页同时禁用相关按钮。重置会取消 chunk。

服务只监听 localhost。远程调用与浏览器使用相同 SSH 隧道，端口需一致：

```bash
ssh -N -L 8080:127.0.0.1:8080 agiuser@服务器地址
```

打开 `http://127.0.0.1:8080`。Python 客户端也连接此地址。

## Python 使用

调用端只需要 `scripts/demos/web_control/robot_client.py`，可复制到自己的项目；无 Isaac Sim、torch 或 Pillow 依赖。仓库内使用可设置：

```bash
export PYTHONPATH="$PWD/scripts/demos/web_control:$PYTHONPATH"
```

```python
from robot_client import RobotClient

robot = RobotClient("http://127.0.0.1:8080")
state = robot.get_joint_state()
print(state["joint_names"], state["positions"], state["velocities"])
print(state["controllable_joint_names"])

obs = robot.get_observation()
print(obs.sequence, obs.sim_time, obs.joint_names, obs.positions)
for name, jpeg in obs.images.items():
    print(name, len(jpeg))  # JPEG bytes；可以交给 PIL.Image.open(io.BytesIO(jpeg))
```

观测图片为 JPEG RGB，所有相机和观测内关节状态来自同一次仿真循环采集。它返回最近一帧，不强制等待新帧；用 `sequence` 去重，用 `captured_at`（Unix 秒）判断新鲜度。图片刷新率与仿真步频独立。`get_joint_state()` 返回最新物理步关节状态，因此通常比观测中的关节状态更新。`targets` 是位置目标，`positions` 是实际物理位置。

## 下发 joint action chunk

先从 `controllable_joint_names` 查询合法名称。关节位置顺序完全由请求中的 `joint_names` 决定；旋转关节使用弧度，移动关节使用米。多实例 X5 名称带实例前缀，不能直接套用 Cobot 名称。

```python
state = robot.get_joint_state()
name = state["controllable_joint_names"][0]
i = state["joint_names"].index(name)
q = state["positions"][i]

# 示例保持当前角度；真实轨迹由调用端生成。
task = robot.execute_joint_chunk(
    joint_names=[name],
    positions=[[q], [q], [q]],  # [T, N]，也接受 numpy 数组
    steps_per_action=12,
)
result = robot.wait(task.id, timeout=120)
print(result.status, result.details)
```

- 每行直接作为绝对位置目标，下发给仿真关节执行器，无插值、平滑、自动限速或裁剪。
- 每行保持 `steps_per_action` 个物理步；物理步长见 `physics_dt`，当前为 1/120 秒。12 步对应 0.1 仿真秒，渲染较慢时墙钟时间会更长。
- 时间到就执行下一行，不等待到位。最后一行保持指定步数后标记 `completed`，之后继续保持最后目标。
- 未指定的关节保持此前已施加的目标；不会继续网页尚未结束的渐进移动。
- 一次只执行一个 chunk，不预排后续 chunk；忙时返回 HTTP 409。暂停时允许提交，但序列不会推进，单步会执行一个物理步。
- 每段最多 2048 行，每行保持 1～120000 步，请求不超过 2 MiB。所有行先检查关节名、维度、有限数值及限位，错误整段拒绝。被动关节不允许控制。
- 服务器保留最近 64 个任务的状态；未知或过期任务返回 404。

### 为什么完成不等于到位

位置控制是给驱动器设定目标，不是修改关节真实位置。驱动力上限、惯量、重力、接触和控制增益都会影响跟踪。即使目标处于关节限位内，给定时间不足时也可能没有到位；遇到阻挡时甚至可能持续不到位。

`completed` 表示最后一行已经完成指定物理步数。完成结果包括 `actual_positions`、`targets` 和 `errors`（目标减实际），顺序与任务关节名称一致，取自最终物理步之后。调用端可判断误差，再决定继续保持、规划新序列或结束。任务的完成结果是固定记录；继续保持后的最新位置请读取 `get_joint_state()`。

```python
result = robot.get_task(task.id)
print(result.details["executed_steps"], result.details["total_steps"])
# 取消仍保留最后已下发的目标，不会瞬间停止真实运动。
robot.cancel(task.id)
```

`wait()` 的超时是墙钟时间，超时不会取消服务器执行；断开客户端也不取消。提交请求网络超时时不要盲目重试，因为服务端可能已经接受；先检查 `get_joint_state()["active_task"]`。仿真正常退出/报错时活动任务标为 `failed`，进程退出后 HTTP 自然不可再查询，任务不落盘。

## 可运行示例

读取观测并保存图片，不移动机器人：

```bash
python scripts/demos/web_control/examples/joint_chunk.py
```

明确指定关节后才移动。以下示例的插值发生在调用端：

```bash
python scripts/demos/web_control/examples/joint_chunk.py --joint fl_joint1 --delta 0.05
```

## HTTP 接口

| 方法 | 路径 | 用途 |
|---|---|---|
| GET | `/api/robot/joints` | 最新关节状态、物理步长、可控关节和活动任务 |
| GET | `/api/robot/observation` | 同帧多路 JPEG（Base64）与关节状态 |
| POST | `/api/robot/chunks` | JSON：joint_names、positions、steps_per_action |
| GET | `/api/robot/chunks/{id}` | 执行状态、步数、终态误差 |
| POST | `/api/robot/chunks/{id}/cancel` | 空 JSON 对象 `{}`，取消并保持目标 |

验证命令：`python -m pytest -q scripts/demos/web_control/tests`。单元和 HTTP 集成测试不需要 GPU；实际模型跟踪误差仍取决于资产、执行器配置及场景。

本次验证：55 项 CPU 单元/HTTP 测试通过，Ruff、Shell/JavaScript 语法检查通过。在独立 8083 测试实例中读取 Cobot Magic 四路 480×360 图片，并对前左、前右 joint1 执行 11 行、每行 12 步的 chunk。执行计数 132 步，仿真时间差 1.1 秒，终态误差约 ±0.00228 rad，随后下发返回原始位置的目标；观测模式拒绝网页手动运动检查通过。测试实例已停止。未验证抓取或所有关节的极限运动。

关节状态里的 `limits` 与 `joint_names` 对齐；无限位的被动连续关节使用 `null` 表示无有限边界，不代表允许控制该关节。

## 多机器人复测

启动所需配置后，可检查相机图像、关节动作序列、目标保持和返回，以及 observe 模式的手动控制限制。脚本会移动指定仿真实例的部分关节；该检查不覆盖全部关节、碰撞或抓取。

```bash
bash scripts/demos/web_control/start_simulation.sh --robot a2d --port 8083
# 在另一个已激活 env_isaaclab 的终端执行：
python scripts/demos/web_control/tests/check_robot_profiles_live.py \
  --robot a2d --url http://127.0.0.1:8083 --output output/a2d_robot_server_check.json
```

`--robot` 支持 `x5`、`x5_dual`、`cobot_magic`、`franka`、`a2d`，必须与服务配置一致。
