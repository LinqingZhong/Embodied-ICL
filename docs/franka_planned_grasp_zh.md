# Franka 自动规划抓取教程

参考 RoboTwin 的两阶段采集：先规划并验证物理成功，再保存轨迹与采集结果；规划成功不是抓取成功。RoboTwin 采用 SAPIEN，本实现使用 Isaac Lab，不直接复制其仿真接口。

cuRobo 固定到 Isaac Lab 文档推荐的 ebb71702f3f70e767f40fd8e050674af0288abe8。源码位于 `/home/agiuser/workspace/dev_zlq/third_party/curobo`，环境为 env_isaaclab、PyTorch 2.7 cu128，安装 CUDA Toolkit 12.8。

场景：桌面高度 0.2 m，4 cm 方块、质量 50 g、摩擦接触。目标为抓起、抬高并保持，不通过固定约束把物体绑到机器人。规划器考虑桌面和机器人自碰撞；目标方块属于允许接触物，抓取接近时不能将全部接触当作碰撞拒绝。

规划在独立进程中运行，退出释放显存，再启动渲染执行，避免 cuRobo 与 Isaac Sim 同时占用有限显存。成功标记由执行脚本的物理抬升与放回检查决定。

## RoboTwin 参考点

- `RoboTwin/scripts/collect_data.py`：先按 seed 试运行，联合 plan_success、check_success、planned_joints_legal 筛选成功轨迹，再采集。
- `RoboTwin/envs/robot/planner.py`：cuRobo MotionGen、机器人基座变换、桌面障碍、轨迹采样；不把 IK 直接称为规划。
- 本示例是单场景确定目标的规划抓取，还不是 RoboTwin 的多任务随机化采集框架。采用对应思路，未复制 SAPIEN 接口。

## 安装记录

CUDA Toolkit 12.8 安装完成，cuRobo 编译架构为 8.9（4090D），编译成功。恢复 packaging=23.0、setuptools_scm=8.1.0，避免破坏 Isaac Sim 依赖；pip check 仍报告原环境 FastAPI 版本与 isaacsim-kernel 的固定版本不同，本任务未调整现有网页服务依赖。

当前 Warp 版本缺少旧版 `warp.torch` 接口，因此本长方体场景使用 cuRobo PRIMITIVE 碰撞检查器，不使用网格碰撞检查器。不修改全局 Warp。

## 手动执行流程

规划与仿真分开运行，避免同时占显存。先停止网页仿真，激活环境，在仓库根目录执行：

```bash
source ~/anaconda3/bin/activate env_isaaclab
export CUDA_HOME="$CONDA_PREFIX"
mkdir -p output/franka_planned_grasp
python scripts/demos/web_control/planning/plan_franka.py \
  --request scripts/demos/web_control/planning/franka_grasp_request.json \
  --output output/franka_planned_grasp/plan.json
```

然后终端一启动场景：

```bash
bash scripts/demos/web_control/planning/start_franka_grasp.sh
```

终端二执行：

```bash
python scripts/demos/web_control/planning/execute_franka_grasp.py \
  --plan output/franka_planned_grasp/plan.json \
  --output output/franka_planned_grasp/result.json
```

必须是刚启动的专用场景，不用于未知状态的机器人。规划请求中的目标是法兰坐标，夹爪接触中心沿法兰 Z 有偏移。计划考虑桌面和机器人自碰撞；方块为允许接触目标，当前没有载物碰撞几何附着，也没有额外杂物。因此不是任意场景的通用避碰保证。

夹爪针对本小方块使用 Kp=800、Kd=40、力上限20；目标张开0.04 m、闭合0 m，通过真实碰撞阻止完全闭合并夹持。没有创建物体固定约束。

## 历史验证结果（本地生成文件）

以下文件是本地运行产物，不随仓库分发；可按上述流程重新生成。

最终成功记录：`output/franka_planned_grasp/result_final.json`。方块中心初始约0.22 m，抬起保持的四个采样高度为 0.39944106、0.39944106、0.39944106、0.39944106 m；随后放回、张开并撤离，最终高度检查通过。

HDF5：`output/franka_planned_grasp/franka_grasp.hdf5`，2407条状态、602组两相机图片、约20.07仿真秒，含每步物体状态；success=True、complete=True。视频为同目录 `franka_grasp_multiview.mp4`。

cuRobo 轨迹终点经当前 USD FK 验证，位置误差约 1.8e-6～1.5e-5 m。第一次执行在释放夹爪时因 float32 上限被拒绝，失败记录保留且标记 success=False；第二次使用实际限位完成全流程。

包含相机调整在内已有三次完整成功执行（同一场景、同一计划），不是多 seed 成功率统计。物理夹持仍有接触速度噪声，未声明真实机器人可直接执行。目标方块不作为禁止接触障碍，载物几何尚未纳入后续避碰；当前无杂物场景验证通过。

最终腕部相机已移至手掌侧方并朝向夹爪接触区域。602组双相机 JPEG 全部解码、30 FPS 仿真时间轴、HDF5 success/complete、MP4全片解码均通过；63项回归测试通过。测试服务已停止。
