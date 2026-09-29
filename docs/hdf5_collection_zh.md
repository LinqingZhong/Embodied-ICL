# HDF5 自动采集

先启动采集模式，再运行自己的客户端或动作清单。规划器暂未接入，instruction 只是任务标签，不自动生成运动。

```bash
bash scripts/demos/web_control/start_collection.sh --robot cobot_magic
```

默认请求 30 FPS **仿真时间**采样，120 Hz 物理步每 4 步生成一组真实多相机图片，不复制帧；关节状态每个物理步记录。暂停时不重复生成帧。实际墙钟速度取决于 GPU，文件保存两种时间。普通 start_simulation.sh 仍默认 3 FPS 墙钟采样；start_collection.sh 才启用采集时序。可传 --camera-fps 覆盖；非 120 的整数约数会受物理步量化。

## Python 录制

```python
from robot_client import RobotClient
r = RobotClient('http://127.0.0.1:8080', timeout=90)
episode = r.start_recording('将左臂移动到目标位置')
try:
    # 可下发已有 joint / EEF chunk，也可以只采集观测
    state = r.get_joint_state()
    print(r.recording_status())
finally:
    result = r.stop_recording(episode['id'], success=False, reason='user_stop')
    print(result)  # path 是服务器文件路径，不是客户端下载路径
```

文件默认在服务器仓库根目录的 `output/datasets/episode_<id>.hdf5`，可通过启动参数 `--recording-dir` 修改。每个文件一段 episode，不覆盖旧文件。自然语言任务成功必须由调用者判断，不应将动作执行完成等同于任务成功。

## 动作清单自动执行

创建 JSON：

```json
{
  "instruction": "前左臂 joint1 小幅移动",
  "actions": [
    {"type": "joint", "joint_names": ["fl_joint1"], "positions": [[0.0], [0.02]], "steps_per_action": 120}
  ]
}
```

```bash
python scripts/demos/web_control/examples/collect_episode.py /path/to/actions.json
```

`type=eef` 时填写 arm、positions、quaternions、steps_per_action，与 RobotClient 相同。客户端负责生成轨迹，录制器不插值、不规划。脚本自动开始录制、依次执行、结束并检查写入错误。执行超时/异常时会结束文件，但不会自动取消仍在服务端执行的任务；按既有控制接口查询或取消。文件 success 默认 false，原因区分动作完成和执行异常。

## 文件结构

- attributes：schema、instruction、robot、metadata_json（机器人 profile、场景、启动参数、相机名称、physics_dt）、complete、success、end_reason。
- joint_names：数组列的关节顺序。
- state：sim_time、wall_time（Unix 秒）、step、positions、velocities、targets（实际施加的关节目标）。
- frames：sim_time、wall_time、step，与每路图片逐行对应。
- images/camera_0 等：变长 uint8 JPEG bytes，相机顺序见 metadata_json.cameras。
- events/json：动作请求、EEF 原始请求、IK 后关节序列及任务终态。

关节与图片采用独立时间轴；frames.step 与 state.step 对齐。普通墙钟模式暂停时可能有同物理步的重复状态和新图片，采集模式不会。step 是服务器生命周期内递增编号，不要求每个文件从 0 开始。

```python
import h5py, json, io
from PIL import Image
with h5py.File('episode.hdf5') as f:
    metadata = json.loads(f.attrs['metadata_json'])
    q = f['state/positions'][:]
    timestamps = f['frames/sim_time'][:]
    image = Image.open(io.BytesIO(f['images/camera_0'][0].tobytes()))
```

后台写盘队列最多 256 个条目，溢出明确报告 error 并将 complete 标为 false，不静默丢帧冒充完整数据。录制期间禁止 reset，以免时间倒退。正常服务退出会关闭文件并标记 simulation_stopped；强杀/断电可能留下 incomplete 文件。它是自定义 HDF5 格式，不是 LeRobot 数据集，后续可另做转换。

## 实测

Cobot Magic，4 路 480×360 JPEG：一段记录含 124 个状态样本、31 组图片，图片间隔为 1/30 仿真秒。实际墙钟吞吐约 14 FPS，因此当前资源下不能声称实时 30 FPS。文件重新打开、四路 JPEG 解码及时间间隔检查通过。

## 回放与复现的区别

HDF5 采集支持所有现有 RobotServer 机器人配置。已补测 ARX 单 X5、双 X5、Franka 和 AgileX Cobot Magic 的真实录制、全部 JPEG 解码、时间对齐及保存动作重放。

原始图像回放不启动仿真，直接显示保存的 JPEG，图像字节不会重新编码：

```bash
python scripts/demos/web_control/examples/replay_hdf5.py \
  output/datasets/episode_ID.hdf5 --output /tmp/episode_replay.html
```

将 HTML 下载到本机后用浏览器打开，支持播放/暂停、拖动进度和多视角同步显示。按保存的仿真时间间隔播放；浏览器调度和显示设备不保证精确显示时刻，但帧内容与保存 JPEG 相同。独立 HTML 会包含全部图片，适合短 episode；长数据集建议按段导出。

重新跑仿真是另一回事。本次测试从 HDF5 events 读取实际 joint chunk，重置后重新下发，并按首个变化的目标对齐比较 120 个物理步。目标序列完全一致，但关节状态并不普遍逐位相同。网络提交时刻、起始速度、物体接触和 GPU 求解都会影响复现。

**当前文件不包含可完整恢复物理场景的快照**（物体根状态/速度、驱动与求解器内部状态、随机状态等），因此不能保证重新仿真得到相同图片或任意任务轨迹。要看到完全相同的已保存画面，请使用图片回放；状态复现需要后续扩展完整场景状态与确定性调度。

测试脚本 `tests/check_recording_profiles_live.py` 会移动机器人并重置场景，只用于独立测试实例。结果及原始/重放 HDF5 路径见 `output/recording_replay_verification/*.json`。
