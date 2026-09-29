# 控制与渲染质量优化

默认启动脚本已启用 compensated 控制与 studio 光照，图片 480×360、JPEG 95、无色度降采样、DLAA 抗锯齿，仍使用实时渲染。重启服务生效。

## 控制

实现文件：`scripts/demos/web_control/simulation_quality.py`。仅在本 demo 创建机器人时覆盖配置，不修改仓库通用机器人配置。

- ARX X5 单/双臂、Cobot Magic、Franka 开启真实重力，并使用 PhysX 重力补偿力矩作为前馈。
- 使用稳定的隐式 PD。Franka Kp/Kd 从 80/4 改为 400/40，关节等效惯量至少按配置添加；Cobot 与 ARX 保留原 PD 增益。
- 保留原力矩上限。补偿力矩先限幅，再为隐式驱动预留 `原力矩上限 - |补偿力矩|` 的保守预算，使两者总量不会超过原上限。预算不足时无法保证跟踪。
- 位置/速度求解迭代为 16/4。保留物理碰撞设置，保留 120 Hz 物理时序。
- 不插值、不瞬移、不延长 chunk，不自动放宽 IK 容差。IK/FK 与物理姿态仍分别验证。
- A2D 未增加补偿配置，沿用原控制。

显式 PD 曾做对照但未采用：当前步长下出现较大偏差。最终使用隐式驱动。重力前馈依赖仿真模型质量/惯量，不等于实体系统已标定；挂载未建模载荷或接触外力后仍可能有误差。

## 渲染

用低强度环境光和两个不同色温的大面积光源替换单一高强度均匀照明；网格地面改为粗糙灰色材质，机器人原有材质保留。图像提高到 JPEG 95、4:4:4，DLAA 使用原生分辨率减少低分辨率 DLSS 模糊。未加入景深/运动模糊，以免影响机器人视觉输入。

```bash
bash scripts/demos/web_control/start_simulation.sh
# 显存充足时提高分辨率
bash scripts/demos/web_control/start_simulation.sh --width 640 --height 480
# 可选：使用自己的 HDRI 环境纹理（默认无需额外下载）
bash scripts/demos/web_control/start_simulation.sh --hdri /absolute/path/studio.hdr
# 对照旧控制与光照
bash scripts/demos/web_control/start_simulation.sh --control-quality legacy --visual-quality legacy --jpeg-quality 80
```

本机另一个 GPU 进程占约 18 GB；Cobot 四相机 640×480 实测出现显存分配错误，因此默认采用 480×360。不要在这种资源条件下直接开启更昂贵的路径追踪。提高分辨率和照明不能代替高质量纹理；模型仍是 CAD 仿真资产，未声称达到照片级真实。

## 验证

测试原始记录及图片位于 `output/quality_verification/`。运行 `check_eef_live.py --tracking-tolerance 0.001` 会在 FK/IK 精度之外检查末端位置跟踪小于 1 mm、额外实际姿态的方向误差小于 0.01 rad。每臂 12 组局部随机 FK/IK 回代，3 个实际姿态的 FK/物理对照，并下发小幅 EEF 动作。这个范围不代表整个工作空间、高速轨迹或抓取接触稳定性验证。

## 最终实测汇总

以下为默认补偿配置的小幅测试结果，单位 mm：

| 机器人 | EEF 保持 1 秒后最大位置误差 | 额外 3 个姿态最大位置误差 |
|---|---:|---:|
| franka | 0.002876 | 0.011552 |
| cobot_magic | 0.003181 | 0.072899 |
| x5 | 0.000002 | 0.000051 |
| x5_dual | 0.000002 | 0.000100 |

61 项 CPU/HTTP 单元测试通过。所有机器人重新通过 FK→IK→FK 与 FK/物理位姿对照。最终 Franka/Cobot 480×360 日志没有显存分配错误；ARX 先前 640×480 验证无显存错误。实际检查了 Franka 和 Cobot 场景截图，保留其它相机图片供复查。测试实例已停止。
