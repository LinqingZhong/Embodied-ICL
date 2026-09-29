# ARX / AgileX 自动规划抓取与采集

已完成真实接触抓取验证：ARX X5 单臂、双 X5 的左臂、AgileX Cobot Magic 的前左臂（fl）。工作流参考 RoboTwin 的“规划、物理成功判定、保存数据”，这里使用 Isaac Lab + cuRobo，未使用 SAPIEN 接口。没有将物体绑定到夹爪或直接修改物体位姿。

## 已验证结果（2026-09-22）

| 本体 | 执行臂 | 抬升高度 | HDF5 图像帧数 | 视频时长 |
|---|---|---:|---:|---:|
| X5 | arm | 约 16 cm | 718 | 23.93 s |
| 双 X5 | left | 约 16 cm | 716 | 23.87 s |
| Cobot Magic | fl | 约 15 cm | 737 | 24.57 s |

每条包含接近、下降、闭爪、抬升、保持、放回、开爪、撤离。成功要求物体中心升高超过 10 cm、四次保持采样高度波动小于 2 cm，最后回到初始高度附近。此判据不保证回到完全相同的位置/姿态，也不代表任意资产的通用抓取成功率。

视频位于仓库根目录下：

- `output/x5_planned_grasp/x5_multiview.mp4`
- `output/x5_dual_planned_grasp/x5_dual_multiview.mp4`
- `output/cobot_planned_grasp/cobot_multiview.mp4`

各目录已放置便于查看的 `episode.hdf5` 副本。各目录的 `result.json` 包含真实 HDF5 路径与物体状态检查结果，`plan.json` 保存轨迹和场景配置。失败尝试也保留在 datasets 中，标记 success=false；Cobot 首次高物体尝试的报告为 `failed_tall_object.json`。

## 使用方法

在 IsaacLab 根目录操作。先关闭占用显存的仿真服务；规划进程退出后再启动仿真。这台机器剩余显存不足以同时运行两者。

```bash
source ~/anaconda3/bin/activate env_isaaclab
export CUDA_HOME="$CONDA_PREFIX"

# 1. 离线生成轨迹：x5 / x5_dual / cobot_magic
python scripts/demos/web_control/planning/plan_grasp.py \
  --robot cobot_magic --arm fl --output output/my_cobot_grasp/plan.json

# 2. 启动独立采集仿真，等待 WEB_CONTROL_READY
bash scripts/demos/web_control/planning/start_grasp.sh cobot_magic --grasp-arm fl
```

另一个终端执行（激活同一 Conda 环境）：

```bash
# 3. 执行轨迹、检查真实抬升并自动保存 HDF5
python scripts/demos/web_control/planning/execute_grasp.py \
  --plan output/my_cobot_grasp/plan.json \
  --output output/my_cobot_grasp/result.json

# 4. 从 result.json 的 recording.path 获取本次 HDF5 路径
python scripts/demos/web_control/examples/export_hdf5_mp4.py \
  output/datasets/episode_实际ID.hdf5 --output output/my_cobot_grasp/video.mp4
```

每次执行使用新启动的场景，避免机器人初态、物体位置与规划不符。失败时停止录制并标记失败，检查报告后重启场景。视频导出不会覆盖已有文件。

机器人/手臂参数对应关系：

| robot | planner --arm / simulation --grasp-arm | 实际抓取验证 |
|---|---|---|
| x5 | arm（默认） | 通过 |
| x5_dual | left（默认）、right | left 通过；right 配置存在，未做物理采集验证 |
| cobot_magic | fl（默认）、fr | fl 通过；fr 配置存在，未做物理采集验证 |

这不是双臂协同抓取；一次只驱动选定的一只手臂，其余手臂保持原位。Cobot 后方 bl/br 仍可用通用 joint/EEF 接口控制和 HDF5 采集，但未加入本自动抓取场景。

## 坐标与配置

- `planning/grasp_profiles.py`：手臂底座世界位姿、桌子和方块、法兰目标、接触偏移。
- `planning/plan_grasp.py`：原生 URDF 的 cuRobo 模型与离线规划。X5 使用资产目录已有碰撞球；Cobot 使用沿关节连杆的近似球体。
- `planning/execute_grasp.py`：校验规划与 RobotServer FK 的位置/姿态一致性，执行 joint chunk，调用夹爪映射，保存真实物体状态和成功标记。
- `grasp_scene.py`：生成静态桌面与有质量的动态方块。

EEF 始终为末端法兰，目标位姿相对选中手臂的底座。X5 接触偏移沿法兰 +X 约 14 cm；Cobot 沿 +Z 约 11 cm。修改资产、夹爪或桌面配置后应重新规划与验证。

采集包含 instruction、robot_profile、全部关节观测/目标、物体状态和各相机 JPEG；相机为仿真时钟 30 FPS，状态为 120 Hz。实际墙钟速度取决于渲染性能。Cobot 保存 scene/head/fl_wrist/fr_wrist 四路；双 X5 保存 scene/left_wrist/right_wrist 三路。

## 当前限制

这是固定场景、已知物体位置的抓取示例，尚无视觉识别、随机任务生成或抓取姿态搜索。规划检查选中机械臂近似碰撞球与桌面；完整机身、其它手臂、手指和被抓物体的碰撞包络没有全部纳入规划。Cobot 球体不是经认证的网格包络。因此不能据此宣称支持复杂场景安全避障或真机直接执行。物理仿真中仍有实际碰撞接触。

执行器保持已有配置，未为了得到成功数据而绑定物体、传送物体或无限增加力矩。PD 跟踪误差和接触滑移仍可能存在；必须通过实际物体状态判定抓取结果。

已逐帧验证三条 HDF5 JPEG 和导出 MP4 可解码，时间间隔满足 30 Hz 图像/120 Hz 状态；校验摘要为各输出目录的 `verification.json`。规划与 USD FK 最大位置差约 0.006 mm，这不是物理跟踪误差。当前腕部相机在部分阶段被夹爪遮挡，多视角训练数据使用前需要优化安装位置；场景相机可查看完整动作。
