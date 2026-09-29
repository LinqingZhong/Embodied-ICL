# 在浏览器中查看和控制 Isaac Lab 仿真

这份示例提供一个浏览器控制台：查看 Franka Panda 的场景相机和腕部相机，调整关节目标，操作夹爪，暂停、单步、重置，并保存双相机快照。运行在服务器的真实 Isaac Sim 仿真上，无需服务器外接显示屏。

入口脚本：[run.py](../scripts/demos/web_control/run.py)。它只控制仿真，不连接实体机器人。


## 1. 在服务器启动

推荐使用[一键启动脚本](../scripts/demos/web_control/start_web_control.sh)，它会自动向上查找仓库根目录、激活 Conda 环境并启用无界面相机渲染：

```bash
cd /home/agiuser/workspace/dev_zlq/IsaacLab
./scripts/demos/web_control/start_web_control.sh
```

可直接传入端口、分辨率等参数：

```bash
./scripts/demos/web_control/start_web_control.sh --port 8081 --width 320 --height 240 --camera-fps 5
./scripts/demos/web_control/start_web_control.sh --help
```

也可以在任意目录使用脚本的绝对路径启动，或进入 `scripts/demos/web_control` 后运行 `./start_web_control.sh`。脚本可放在仓库根目录或子目录，会自动定位 `isaaclab.sh`。默认激活 `~/anaconda3` 下的 `env_isaaclab`；如需更换安装位置或环境，可设置 `ISAACLAB_CONDA_ROOT`、`ISAACLAB_CONDA_ENV`。脚本以前台方式运行，保留 NVIDIA EULA 的正常确认流程，也会继承已设置的 `OMNI_KIT_ACCEPT_EULA`。

以下是等价的手动启动方式：

```bash
source ~/anaconda3/bin/activate env_isaaclab
cd /home/agiuser/workspace/dev_zlq/IsaacLab

./isaaclab.sh -p scripts/demos/web_control/run.py \
  --headless --enable_cameras --port 8080
```

Isaac Sim 启动可能要求确认 NVIDIA EULA。若已阅读并同意该许可，需要在无人值守的终端中运行，可以在启动前设置：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
```

网页服务监听 `127.0.0.1:8080`。终端会打印 `WEB_CONTROL_URL`，完成仿真初始化后打印 `WEB_CONTROL_READY`。首次使用可能需要下载机器人资产、编译 RTX 着色器；初始化期间页面会显示加载状态，控件不可用。

保持启动终端运行；结束时按 `Ctrl+C`，脚本会关闭网页服务并清理仿真。若需要长期在 SSH 断开后保留进程，可以在自己管理的 `tmux` 会话中启动。

当前环境已配置单份 Vulkan 驱动描述，使用上面的 Conda 激活方式即可应用。若使用其他环境，需要重新核对 Isaac Sim、PyTorch、FastAPI、Uvicorn 和 Pillow 依赖。

## 2. 在远程电脑连接

在你的电脑上打开终端，而不是在服务器上，执行：

```bash
ssh -N -L 8080:127.0.0.1:8080 用户名@服务器地址
```

例如把用户名替换为 `agiuser`，服务器地址替换成实际 IP 或 SSH 主机别名。保持这条 SSH 命令运行，然后在电脑浏览器打开：

```text
http://127.0.0.1:8080
```

如果 8080 已占用，可以在服务器启动命令中改用 `--port 8081`，并把 SSH 命令中的两处 8080 和浏览器地址全部改成 8081。

首版默认只监听服务器回环地址，通过 SSH 完成访问认证，不需要开放公网控制端口。不要把这个单用户控制面板当作带账号和权限管理的公网服务部署。

## 3. 网页操作

| 控件或信息 | 行为 |
| --- | --- |
| 场景相机 | 固定在世界中的观察视角，看到机械臂和参照物 |
| 腕部相机 | 挂在 `panda_hand` 连杆下，随手臂运动 |
| 关节滑块 | 设置七个手臂关节的目标角度，单位 rad |
| `−` / `+` | 每次点击减小或增加 0.05 rad；不会持续自动发送点动指令 |
| 实际 / 已施加 | 实际反馈角度，以及经过变化速率限制后的控制目标 |
| 张开 / 闭合 | 同时设置两个夹爪手指的目标位置 |
| 暂停 / 继续 | 停止或继续物理步进；暂停时画面仍会刷新 |
| 单步 | 暂停时前进一个物理步，默认 1/120 秒仿真时间 |
| 重置姿态 | 恢复默认关节状态和目标，清零速度与仿真时间，保持原暂停状态 |
| 保存并下载 | 保存同一次捕获的两路 JPEG 和关节状态，并下载 ZIP |
| 画面距今 | 服务器最新捕获距今的时间；浏览器接收失败也会显示画面过期 |

目标值与实际角度不一定完全相同。示例保留重力并采用 Franka 的默认执行器，关节会受动力学和 PD 增益影响。为避免滑块跳变直接施加大幅目标变化，手臂目标变化限制为 0.5 rad/s，手指为 0.04 m/s，均按仿真时间计算。

暂停期间可以修改目标，继续或单步时开始执行。关节限位来自加载的机器人配置；不代表在限位内的所有姿态都不会发生自碰撞或碰到地面。

多个浏览器页面共享同一个仿真，最后被服务器接受的目标生效。断线时页面禁用控件、清理尚未发送的目标并自动重连；仿真保持最后有效位置目标，连接断开本身不会自动暂停物理世界。

Franka 没有头部，所以左侧观察视角是场景相机，不是头部相机。

## 4. 图片保存位置与数据内容

默认输出目录是启动工作目录下的：

```text
output/web_control/
```

每次保存创建一个独立的 `snapshot_<随机标识>.zip`：

```text
scene.jpg
wrist.jpg
metadata.json
```

元数据记录捕获序号、服务器时间、仿真时间、关节名称、实际位置、速度和目标位置。两张图片与元数据来自同一份共享快照，不会在保存过程中混入下一帧的数据。

可以通过 `--output-dir` 设置其他目录。当前版本不做连续视频录制或深度保存；两路画面均为 RGB。

## 5. 分辨率、性能和命令参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--port` | 8080 | 本地 HTTP 端口 |
| `--width` | 480 | 每台相机的宽度 |
| `--height` | 360 | 每台相机的高度 |
| `--camera-fps` | 10 | 按现实时间限制捕获频率的上限，不是保底帧率 |
| `--output-dir` | `output/web_control` | 快照输出目录 |
| `--duration` | 0 | 就绪后运行的现实秒数；0 表示持续运行 |

显存较紧张时先使用：

```bash
./isaaclab.sh -p scripts/demos/web_control/run.py \
  --headless --enable_cameras \
  --width 320 --height 240 --camera-fps 5
```

本机 RTX 4090 D 的其他进程在验证时占用了约 18 GB 显存。480×360 双相机验证能运行，但剩余显存仅约 0.2 GB；这不是充裕的运行余量。正式长时间使用前，请用 `nvidia-smi` 重新检查并安排其他 GPU 工作负载。

降低分辨率可以减轻部分渲染和带宽开销，但 Isaac Sim 自身也占用显存，不能保证只降低分辨率就能解决所有显存不足。

渲染、JPEG 编码、网络传输和浏览器解码都会影响帧率。物理步长为 1/120 秒，主循环不会为追赶现实时间而积累大量补步，因此繁忙时仿真时间可能比现实时间走得慢。该面板用于交互调试，不提供硬实时控制保证。

## 6. 实现结构与后续替换机器人

| 文件 | 职责 |
| --- | --- |
| [run.py](../scripts/demos/web_control/run.py) | 仿真主线程、机器人、相机、控制步进与退出清理 |
| [state.py](../scripts/demos/web_control/state.py) | 线程安全的目标、动作、状态和成对图像快照 |
| [server.py](../scripts/demos/web_control/server.py) | HTTP API、静态页面、图片响应和快照下载 |
| [app.js](../scripts/demos/web_control/static/app.js) | 控件、状态同步、JPEG 逐帧获取和断线恢复 |

服务线程不会直接操作 Isaac Sim 对象。前端逐帧获取 JPEG，分别跟踪两路图像是否真正接收成功。后端还保留 `/api/stream/scene` 和 `/api/stream/wrist` 的 MJPEG 接口，便于其他客户端使用。

首版固定使用 Franka。若换成你的机器人，需要替换 `run.py` 中的机器人配置、手臂和夹爪关节集合，以及相机父连杆和安装位姿；并检查控制模式、限位和变化速率。不是只给任意 URDF 路径就能自动配置所有控制器。

机器人和头部／腕部相机的配置方法见[新机器人接入教程](new_robot_and_multi_camera_control_zh.md)。

## 7. 验证方式

无 GPU 的控制状态和 HTTP 测试：

```bash
python -m pytest -q scripts/demos/web_control/tests
```

先启动真实演示，再运行集成检查。该检查会实际移动仿真机械臂、操作夹爪并保存一次快照：

```bash
python scripts/demos/web_control/tests/check_live.py
```

浏览器测试需要另行安装 Playwright 和 Chromium，在已有测试环境中运行：

```bash
node scripts/demos/web_control/tests/check_browser.cjs
```

脚本需要 Node.js 能解析 `playwright` 模块，可通过 `NODE_PATH` 指向独立安装目录。可选环境变量为 `WEB_CONTROL_URL`、`WEB_CONTROL_ARTIFACTS` 和 `BROWSER_EXECUTABLE`。浏览器检查同样会修改仿真目标，输出页面截图和下载的 ZIP。

2026-09-18 已验证：控制输入检查、暂停/单步/重置、重置后新目标的顺序、HTTP 生命周期、活动图像流关闭、真实双相机图像、关节运动、夹爪、快照下载、浏览器按钮、跨页面目标同步、相机失败恢复、连接失败恢复、断线前排队指令失效和移动端布局。控制模块与 HTTP 测试共 16 项通过。

本次浏览器测试通过本机 Chromium 执行；远程 SSH 隧道需要在你的客户端使用真实服务器地址建立，未代替你验证外部网络链路。

环境仍有安装阶段记录的 FastAPI 固定版本元数据差异；本面板实际使用的 FastAPI 路由和浏览器交互已经验证。启动日志中无显示屏相关 GLFW 警告不一定代表失败，应结合网页状态、实际图像和 traceback 判断。

## 自定义桌子、瓶子等物体

通过 `--scene-config` 指定 USD 资产场景，示例：

```bash
scripts/demos/web_control/start_web_control.sh \
  --scene-config scripts/demos/web_control/scenes/table_bottle.json
```

详细参数、可乐瓶替换、物理模式及桌面坐标见 [网页控制台资产导入教程](web_control_asset_import_zh.md)。此模式下“重置场景”同时恢复机器人关节和动态物体。

## 切换机器人

现在可通过 `--robot franka` 或 `--robot a2d` 选择机器人。A2D 支持分组关节控制、左右夹爪及四路图像；详细配置和新机器人扩展步骤见 [机器人配置教程](web_control_robot_config_zh.md)。
