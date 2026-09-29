# 网页控制台：导入桌子、瓶子和自定义 USD 资产

控制台支持启动时通过 `--scene-config` 加载 JSON 场景文件。指定后，文件中的资产替代默认三个彩色方块，场景相机和腕部相机都能拍到它们，原有关节控制继续使用。

## 运行桌子＋瓶子示例

在仓库根目录执行：

```bash
scripts/demos/web_control/start_web_control.sh \
  --scene-config scripts/demos/web_control/scenes/table_bottle.json
```

随后照原来的方式建立 SSH 隧道，打开 `http://127.0.0.1:8080`。首次加载需要下载 NVIDIA 资产。控制台日志中的 `WEB_CONTROL_ASSET` 显示每个资产的名称、物理模式和实际地址。

示例使用 SeattleLabTable 和 YCB `006_mustard_bottle.usd`（芥末瓶）。**这不是可乐瓶；仓库未附带可乐瓶模型。** 要显示可乐瓶，需准备对应 USD 及其纹理、引用文件，然后替换示例中 Bottle 的 `usd_path`。

实际无显示屏渲染画面：


## 配置自己的资产

复制示例 JSON，再编辑 `assets` 数组。每一项代表一个独立物体，可以添加多个瓶子，名称需不同。例如用下面这一项替换 Bottle：

```json
{
  "name": "ColaBottle",
  "usd_path": "/home/agiuser/assets/cola/cola.usd",
  "mode": "dynamic",
  "position": [0.55, 0.0, 0.2],
  "orientation": [1, 0, 0, 0],
  "scale": [1, 1, 1]
}
```

上述路径是用户模型的示例路径，必须替换为真实文件。启动时建议传配置的绝对路径：

```bash
scripts/demos/web_control/start_web_control.sh \
  --scene-config /home/agiuser/assets/my_scene.json
```

| 字段 | 含义 |
| --- | --- |
| `name` | 唯一名称，仅英文字母、数字、下划线，不能以数字开头 |
| `usd_path` | 本地 USD 路径或 HTTP(S)/Omniverse 地址；本地相对路径相对于 **JSON 所在目录** |
| `mode` | `visual`、`static` 或 `dynamic`，默认 `visual` |
| `position` | 世界坐标 `[x,y,z]`，单位米，默认 `[0,0,0]` |
| `orientation` | 四元数 **`[w,x,y,z]`**，默认 `[1,0,0,0]`，自动归一化 |
| `scale` | XYZ 缩放，必须为正数，默认 `[1,1,1]` |

`isaac://Props/...` 是这个加载器的简写，会替换为当前 Isaac Sim 配置中的 `ISAAC_NUCLEUS_DIR`。本地文件及纹理、引用文件需保留目录结构。JSON 不支持注释；未知字段、重复名称、错误数值及不存在的本地文件会在启动仿真前报错。

## 能看到与能碰撞、抓取的区别

- `visual`：显示模型，禁用其已有的刚体和碰撞属性，适合装饰。
- `static`：用于桌子等固定支撑物。USD 必须已有碰撞几何；若含刚体，则设为运动学刚体保持固定。
- `dynamic`：用于瓶子、杯子等。USD 必须已有单个刚体及碰撞几何；物体受重力、接触和机器人碰撞影响。多个独立刚体或带关节的机构不适用此接口。

加载器不会凭空给纯视觉 USD 生成可靠的碰撞体。带机器人关节的本体仍使用机器人导入教程，不能当作普通物体塞进此数组。视觉模式也仅针对普通物体，不能用于任意复杂场景/机器人 USD。

OBJ/STL/FBX 需先转换。例如：

```bash
source ~/anaconda3/bin/activate env_isaaclab
./isaaclab.sh -p scripts/tools/convert_mesh.py \
  /home/agiuser/assets/cola.obj /home/agiuser/assets/cola.usd \
  --collision-approximation convexDecomposition --mass 0.35 --headless
```

`--mass` 让转换器添加刚体和质量属性。转换后仍应检查尺寸、贴图和碰撞形状。具备动力学并不保证当前夹爪能抓稳，抓取还取决于形状、摩擦、质量和控制姿态。

网页的“重置场景”恢复机器人关节和所有动态物体的初始姿态、速度；暂停与单步也作用于整个物理世界。

## 桌面高度和相机

示例沿用仓库 lift 任务的桌子坐标：桌子放在 `[0.5,0,0]` 并绕 Z 旋转 90°，桌面在世界 `z=0` 附近，地面为 `ground_z=-1.05`，机械臂基座在 `[0,0,0]`。瓶子从 `[0.55,0,0.12]` 开始受重力下落到桌面。该 YCB 模型长轴沿 Y，示例绕 X 旋转 -90°，使瓶子直立。

**`position` 指模型原点，不是模型底部；不自动推断桌面高度或对齐底面。** 替换模型时根据其尺寸和原点调整高度，避免一开始就穿透桌子。物体朝向也取决于源模型的坐标轴。

顶层可设置：

| 字段 | 默认值 / 作用 |
| --- | --- |
| `ground_z` | `0`，地面高度 |
| `robot_position` | `[0,0,0]`，机器人基座位置 |
| `scene_eye` | `[1.6,1.4,1.2]`，场景相机位置 |
| `scene_target` | `[0.25,0,0.35]`，场景相机注视点 |
| `wrist_target` | `[0.55,0,0.04]`，腕部相机初始注视点；安装后随手腕运动 |

配置在启动时读取，修改后需要重启。当前没有网页上传、运行中增删资产或自动摆放功能。

## 验证记录

本机 Isaac Sim 5.1 / RTX 4090 D 上已验证桌子和直立瓶子的实际加载、场景/腕部两路图像、关节控制、暂停、单步及快照。另有独立物理检查，验证瓶子落桌后稳定、动态物体重置，以及实例化桌子在 visual 模式下禁用碰撞。

```bash
python -m pytest -q scripts/demos/web_control/tests
./isaaclab.sh -p scripts/demos/web_control/tests/check_scene_assets.py --headless
```

第二个命令需要已激活 Isaac Lab 环境及可访问 NVIDIA 云资产。它会启动独立仿真，不连接当前网页服务。
