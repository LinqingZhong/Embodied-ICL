# 方舟 ARX X5 网页控制

已将 RoboDojo 的单台 ARX X5 接入当前 Isaac Lab 网页控制台。单臂通过 `--robot x5` 选择：

```bash
bash scripts/demos/web_control/start_web_control.sh --robot x5
```

也可显式选择：

```bash
bash scripts/demos/web_control/start_web_control.sh --robot x5
```

默认端口 8080、480×360、3 FPS。通过原有 SSH 隧道访问。界面显示六个机械臂关节滑块、夹爪开合、场景/腕部两路相机，支持暂停、单步、重置、快照。默认场景包含桌子和芥末瓶。

## 下载的资产

来源为 RoboDojo 官方脚本指定的 Hugging Face 数据集：

https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/tree/main/Assets/Robots/x5

本次下载完整 `Assets/Robots/x5/` 子目录，共 23 个文件，109,641,469 字节，保存在：

```text
/home/agiuser/workspace/dev_zlq/IsaacLab/source/isaaclab_assets/data/Robots/ARX/X5/
├── ARX.usd
├── X5A.urdf
├── robot_config.yml
├── curobo_tmp.yml
├── configuration/
└── meshes/
```

另保存 `DOWNLOAD_SOURCE.json` 记录下载清单和 `ROBODOJO_LICENSE`。X5 完整资产已复制到 Isaac Lab 资产包 data 目录，默认通过 `ISAACLAB_ASSETS_DATA_DIR` 定位，不再依赖相邻 RoboDojo 目录。

迁移机器时需保留整个 x5 目录，或指定：

```bash
ARX_X5_USD=/绝对路径/x5/ARX.usd \
  bash scripts/demos/web_control/start_web_control.sh --robot x5
```

## 配置位置

- `scripts/demos/web_control/robot/x5.py`：网页关节、夹爪、相机配置。
- `source/isaaclab_assets/isaaclab_assets/robots/arx.py`：Isaac Lab 机器人与执行器配置。
- `scripts/demos/web_control/robot/__init__.py`：`x5` 注册入口。

执行器配置参考 RoboDojo `env/robot_manager/robot_config/x5.py`，代码许可保存在模型目录的 `ROBODOJO_LICENSE`。沿用其固定基座、机器人关闭重力、自碰撞开启及 PD 参数；此设置不代表已经校准真实 X5 动力学。

直接控制 `joint1` 到 `joint6`，目标变化速率 0.3 rad/s。夹爪联动 `joint7`、`joint8`，张开 0.044 m，闭合 0 m，变化速率 0.03 m/s。RoboDojo 外部 robot_config.yml 含 -0.01 的夹爪映射，但实际 URDF/USD 下限是 0，所以本适配不发送越界目标。页面显示单指关节位移，不将其误当作真实夹持宽度。

腕部相机挂在 `link6`，通过初始世界观察方向求局部安装变换，随后随手腕运动。它是新增仿真 RGB 相机，不是原厂相机标定。

源 USD 存在部分跨引用范围的材质绑定警告，Isaac Sim 会忽略这些绑定。实际机器人和两路图像能显示，但部分外观材质可能与源场景不同。

## 验证与边界

已验证真实仿真启动、关节位置跟随、夹爪开合、暂停/单步/重置、两图快照；浏览器检查白色主题、两路图片和手机宽度，无脚本错误。复验命令（会操作指定的仿真服务）：

```bash
python scripts/demos/web_control/tests/check_x5_live.py --url http://127.0.0.1:8083
```

上述章节描述单臂；双臂支持见文末。没有接入实体机器人的通信驱动，也未验证自动抓取规划或抓取成功率。

## 双 X5（新增）

现在同时支持 `--robot x5` 单臂和 `--robot x5_dual` 双臂。启动脚本默认配置见顶部 `WEB_ROBOT`；通过命令行选择单臂或双臂：

```bash
bash scripts/demos/web_control/start_web_control.sh --robot x5_dual
bash scripts/demos/web_control/start_web_control.sh --robot x5
```

双臂配置在 `robot/x5_dual.py`，默认场景为 `scenes/x5_dual_table.json`。采用 RoboDojo `env_cfg/robot/dual_x5.yml` 的两台独立、固定基座 X5 布局：

| 实例 | 基座位置（米） | 朝向 wxyz |
| --- | --- | --- |
| left | [-0.3, -0.45, 0.765] | [0.70710678, 0, 0, 0.70710678] |
| right | [0.3, -0.45, 0.765] | [0.70710678, 0, 0, 0.70710678] |

两臂相距 0.6 m，同向绕 Z 旋转 90°。我们配套使用缩放后的 Seattle 桌子，桌面约 z=0.765；这不是 RoboDojo 全部场景和规划系统的移植。

`instances` 指定实例名称、位置和朝向。双臂时场景 JSON 中的 `robot_position` 是两台机器人共享的平移偏移，默认 [0,0,0]；实例位置在此基础上叠加。

网页提供 12 个关节滑块、两个独立夹爪、场景/左腕/右腕三路相机。接口关节名用 `left/joint1`、`right/joint1` 区分两个实例；实际 USD 内的关节名没有修改。三路图像和两台机器人的状态在同一次物理步进后采集，快照记录 16 个关节（含夹爪）。

控制左右夹爪分别发送：

```json
{"action":"gripper","gripper":"left","open":true}
{"action":"gripper","gripper":"right","open":false}
```

重置恢复两台机器人的关节状态及动态物体；关节目标分别写入各自 Articulation，再统一步进一次物理世界。没有新增双臂避碰规划或协同抓取算法。

双臂实测脚本（会移动指定服务里的机器人）：

```bash
python scripts/demos/web_control/tests/check_x5_dual_live.py --url http://127.0.0.1:8083
```

通用机器人配置可使用 `from isaaclab_assets import ARX_X5_CFG`；网页单臂与双臂共用该配置。X5 的 USD 与网格资产通过 Git LFS 随仓库管理；克隆后执行 `git lfs pull` 获取实际模型文件。
