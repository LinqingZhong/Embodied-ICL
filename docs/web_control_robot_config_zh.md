# 网页控制台：机器人配置与智元 A2D

网页控制台通过 `--robot` 选择机器人配置。配置在 `scripts/demos/web_control/robot/`，目前注册 `franka`、`a2d` 、`x5` 和 `x5_dual`。直接运行 Python 入口时默认 Franka；启动 SH 脚本现在默认双 X5，可修改脚本顶部配置区。

## 启动 A2D

在仓库根目录执行：

```bash
scripts/demos/web_control/start_web_control.sh --robot a2d \
  --width 320 --height 240 --camera-fps 3
```

脚本自动激活 `env_isaaclab`、开启无显示屏渲染。四路相机建议先使用上述分辨率验证；显存足够时可以调高。

远程电脑执行（将服务器地址替换为实际地址）：

```bash
ssh -N -L 8080:127.0.0.1:8080 agiuser@服务器地址
```

浏览器打开 `http://127.0.0.1:8080`。换端口时，启动命令和 SSH 隧道两端端口需一致。

A2D 自动使用 `scenes/a2d_table_bottle.json`：地面 z=0、机器人基座 `[-0.6,0,0]`、桌面约 z=0.6。该桌子沿 Z 缩放至 0.6，瓶子初始高度 0.72。也可显式覆盖：

```bash
scripts/demos/web_control/start_web_control.sh --robot a2d \
  --scene-config /绝对路径/my_scene.json
```

机器人配置决定关节、执行器和相机；场景 JSON 决定桌子、瓶子、机器人摆放位置及观察目标。JSON 中的显式值优先于机器人 `scene_defaults`。旧 Franka 桌面示例的地面/桌面高度不同，不能假设适合 A2D。

## A2D 支持范围

- 18 个直接关节滑块：升降、躯干俯仰、头部两轴、左右臂各七轴。
- 左右夹爪独立开合：每侧驱动 `hand_joint1`、`Right_Support_Joint`、`Left_Support_Joint`；不向被动/模仿关节提供滑块。
- 四路相机：场景、头部、左腕、右腕。快照 ZIP 包含四张 JPEG 和同帧关节状态。
- 暂停、单步、重置场景保持可用。
- A2D 基座固定，用于上半身关节调试，不提供底盘导航或行走控制。

源资产为仓库 `AGIBOT_A2D_CFG`，首次启动可能下载 NVIDIA 云资产。保留源配置的执行器分组与增益；面板另限制目标变化速率。升降单位为米，其他直接控制关节为弧度。

A2D 的头部偏航在 USD 中是连续关节，面板将操作范围限制为 ±π/2。这是网页操作范围，不是新增的机械限位。

夹爪映射来自仓库 A2D 放置任务：张开为 0.994 rad；左侧闭合为 0，右侧闭合为 0.2 rad。页面显示的是驱动关节反馈，不将其误标成夹爪毫米开度。接触物体时实际位置可能无法到达目标；没有碰撞规避或自动抓取规划。

## 配置文件结构

```text
scripts/demos/web_control/robot/
├── __init__.py       # PROFILES 注册表及配置副本读取
├── franka.py         # Franka Panda
└── a2d.py            # Agibot A2D
```

每个文件定义一个普通 Python 字典 `PROFILE`，导入时不启动 Isaac Sim。

| 字段 | 含义 |
| --- | --- |
| `label` | 网页显示名称 |
| `asset_module` / `asset_config` | 机器人 ArticulationCfg 所在模块与变量名；仅在仿真启动后导入 |
| `fix_root_link` | 为 True 时固定根连杆 |
| `default_scene` | 默认场景 JSON，相对于 web_control 目录 |
| `scene_defaults` | 场景 JSON 未指定时使用的位置等默认值 |
| `joints` | 网页直接控制的主动关节，按列表顺序显示 |
| `grippers` | 每个夹爪的开合目标映射 |
| `cameras` | 相机列表，必须包含 `scene` 场景相机 |

关节示例：

```python
{
    "name": "joint_lift_body",
    "group": "躯干",
    "unit": "m",
    "speed": 0.03,     # 每仿真秒允许目标变化 0.03 m
    "step": 0.01,      # 网页 +/- 按钮每次变化 0.01 m
}
```

普通关节范围从实际 USD 的软限位读取。连续关节必须提供有限 `limits: [下界, 上界]`，也可用它缩小普通关节的网页操作范围。关节名必须精确匹配，不能用正则表达式。启动时检查缺失关节、重复驱动、非有限范围、初始值及夹爪目标越界。

夹爪示例：

```python
{
    "name": "left",
    "label": "左夹爪",
    "unit": "rad",
    "speed": 0.5,
    "open": {"left_hand_joint1": 0.994},
    "close": {"left_hand_joint1": 0.0},
}
```

此片段只说明格式。A2D 实际配置还联动两个 Support 关节，应保留完整 `a2d.py` 的映射。夹爪关节不要重复出现在直接控制 `joints` 列表中。

## 相机安装

A2D 已核对的安装连杆：

| 相机名称 | `/World/Robot/` 下的父连杆 |
| --- | --- |
| `head` | `link_pitch_head` |
| `left_wrist` | `left_base_link` |
| `right_wrist` | `right_base_link` |

配置中的 `eye_offset` 是**初始世界坐标系**下相对于连杆位置的偏移；`target` 是可选的世界注视点，未设置时采用场景 `wrist_target`。启动时先应用初始关节姿态，再根据物理连杆位姿求相机局部安装变换；随后相机随连杆运动，不会一直自动瞄准瓶子。

需要精确标定时，在相机配置中同时给出 `mount_pos` 和 `mount_rot` 覆盖推导结果：

```python
{
    "name": "head", "label": "头部相机", "body": "link_pitch_head",
    "eye_offset": [0.15, 0, 0.05],
    "mount_pos": [0.05, 0, 0.02],
    "mount_rot": [1, 0, 0, 0],
}
```

这里的局部位姿仅为格式示例；`mount_rot` 顺序为 **wxyz**，采用 USD/OpenGL 相机约定：**-Z 向前、+Y 向上**。请按实际安装替换；单位四元数不一定朝前。相机是在仿真中新增的 RGB 传感器，不代表 A2D 实体相机的原厂标定或真实视频流。

## 接入第三种机器人

1. 准备有效 URDF/USD，并建立机器人专用 `ArticulationCfg`，见 [机器人导入教程](new_robot_and_multi_camera_control_zh.md)。
2. 复制 `robot/franka.py` 或 `robot/a2d.py` 为 `robot/my_robot.py`，修改模型模块、配置名称、主动关节、夹爪和相机连杆。
3. 在 `robot/__init__.py` 导入该 `PROFILE`，并在 `PROFILES` 中添加 `"my_robot"`。命令行选项自动来自注册表。
4. 准备合适的初始姿态和场景，运行 `--robot my_robot`，逐个小幅验证关节和图像。

这些 Python 配置属于本机受信任代码；网页不接受上传或执行配置。当前直接控制通道是位置目标，速度/力矩控制、移动底盘及灵巧手连续动作还需要扩展控制协议。

HTTP 调用可指定夹爪名称：

```json
{"action": "gripper", "gripper": "left", "open": true}
```

关节仍使用 `/api/status` 返回的 `index`，不能假设同一编号在不同机器人中表示相同关节。

## 本机验证

A2D 已验证四路实际渲染、18 个分组滑块、升降/头部/左右臂反馈、左右夹爪开合、重置和四图快照。网页在桌面和手机宽度下均验证，无 JavaScript 页面错误。


回归单元测试：

```bash
python -m pytest -q scripts/demos/web_control/tests
```

另有运行中的仿真接口检查（会操作指定服务上的仿真机器人）：

```bash
python scripts/demos/web_control/tests/check_a2d_live.py --url http://127.0.0.1:8083
```

本次验证不包含真实 A2D 硬件接入、原厂相机标定或自动抓取成功率。

方舟单臂配置、资产路径和验证说明见 [ARX X5 接入教程](web_control_arx_x5_zh.md)。

AgileX Cobot Magic 四臂模型见 [Cobot Magic 接入教程](web_control_cobot_magic_zh.md)。
