# AgileX Cobot Magic / Mobile ALOHA 接入

本次使用用户提供的 `mobile_aloha_sim-2.0.0`，选择 `aloha_tracer2_d435_dark.urdf`。该模型实际包含 Tracer2 底盘、前后四套六轴臂、四个双指夹爪、升降/俯仰相机架及 D435 外壳。

## 启动

```bash
bash scripts/demos/web_control/start_web_control.sh --robot cobot_magic
```

也可把启动脚本顶部 `WEB_ROBOT` 改为 `cobot_magic` 后直接启动。默认 480×360、3 FPS；四路渲染显存占用较高，启动前需停止其他网页仿真服务。

支持 26 个关节滑块（四臂共 24 个，加支架 2 个）、四个独立夹爪、场景/顶部/前左腕/前右腕四路 RGB。`fl/fr/bl/br` 按源模型命名，分别表示前左/前右/后左/后右；没有自动主从遥操作映射。

底盘固定，轮子不暴露为位置滑块。本适配不包含底盘行驶、导航、硬件通信或多臂协同规划。

## 资产管理

完整源目录已复制到：

```text
source/isaaclab_assets/data/Robots/AgileX/CobotMagic/
├── source/                  # 原始 ROS 描述包与网格
├── cobot_magic.urdf          # package:// 改为包内相对路径的转换输入
└── usd/                     # 转换输出，需完整保留子目录
```

模型使用 `ISAACLAB_ASSETS_DATA_DIR` 定位，不依赖原来的外部目录。通用配置位于 `source/isaaclab_assets/isaaclab_assets/robots/agilex.py`，可导入 `COBOT_MAGIC_CFG`。网页配置为 `scripts/demos/web_control/robot/cobot_magic.py`，场景为 `scenes/cobot_magic.json`。

仓库通用 URDF 转换器在 Isaac Sim 5.1 上会下载 2.4.31 导入插件，本机下载卡住，因此提供专用转换脚本，使用本机已有 2.4.30，保留固定连杆：

```bash
source ~/anaconda3/bin/activate env_isaaclab
./isaaclab.sh -p scripts/demos/web_control/robot/convert_cobot_magic.py
```

转换输出应出现 `CONVERTED`，并存在 `usd/cobot_magic.usd`。USD 及其 configuration 子目录必须一起迁移。这些 USD 与网格通过 Git LFS 管理；克隆后执行 `git lfs pull` 获取实际模型文件。

## 控制与物理配置

夹爪 joint7 正向张开至 +0.04 m，joint8 反向张开至 -0.04 m；闭合均为 0。四个夹爪分别通过 `fl/fr/bl/br` 控制。例如：

```json
{"action":"gripper","gripper":"fl","open":true}
```

初始各臂 joint2=1 rad、joint3=-1.2 rad。执行器增益为仿真调试值，并非厂商标定；增加关节等效惯量和求解迭代用于抑制小惯量关节不稳定。支架驱动力高于源 URDF 的 10 N 标注，用于在本仿真配置下承受支架重量，不应视为实体电机能力。

源模型自碰撞关闭，保留与外部物体的碰撞。关节滑块不进行避碰规划。相机由仿真新增，安装于运动连杆，不代表原厂内外参；图像可能包含自身外壳。

导入器对若干空视觉节点发出 unresolved reference 警告；主体网格可渲染，部分相机外壳可能不完整。原始文件保留以便后续修正，不修改原始资产。

## 验证

### 机械臂颜色

源 URDF 对四臂连杆存在不一致的统一浅色覆盖（前左臂大部分连杆没有覆盖）。导入器会将这些颜色设为强绑定，遮住 DAE 网格中的黑、红、灰材质，导致其它臂看起来发白。

`robot/convert_cobot_magic.py` 现在会调用 `robot/cobot_materials.py`，将具有内嵌材质的机械臂视觉节点改为子网格材质优先。统一颜色仍作为未指定材质区域的备用颜色；车体、碰撞、关节及源资产保持原样。涉及的视觉实例会解除实例化以允许覆盖。

当前本地 USD 已应用修复，并通过四臂材质解析检查。正在运行的服务必须停止后重新运行 `scripts/demos/web_control/start_web_control.sh` 才会加载新资产，单纯刷新浏览器无效；实际渲染效果需在重启后确认。

单元测试：

```bash
python -m pytest -q scripts/demos/web_control/tests
```

运行中的仿真验证（会移动指定服务里的机器人）：

```bash
python scripts/demos/web_control/tests/check_cobot_live.py --url http://127.0.0.1:8083
```

本机已通过支架两关节、四臂各一个关节的小幅跟随测试、四个夹爪开合、暂停/重置及四图快照检查。共 40 项单元测试通过，浏览器四路图片、白色主题和手机宽度检查通过。尚未逐关节覆盖全部运动范围，也未验证抓取成功率。
