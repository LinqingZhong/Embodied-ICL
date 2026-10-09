# 新增物体 USD

入口均为 `<name>/<name>.usd`，纹理使用资产目录内的相对路径。
单位为米，Z 轴朝上，原点为底部中心。`scale: [1,1,1]` 即下表尺寸。

| name | 物体 | 宽 × 深 × 高（cm） | 场景 mode |
|---|---|---|---|
| wine_bottle | 酒瓶 | 9.41 × 9.18 × 34.77 | dynamic |
| beer_bottle | 啤酒瓶 | 6.72 × 6.72 × 25 | dynamic |
| orange | 橘子 | 7.79 × 7.97 × 8 | dynamic |
| pear | 梨 | 8.64 × 9.07 × 12 | dynamic |
| storage_box | 收纳箱 | 30 × 30 × 30 | static |
| wood_plate | 木盘 | 36.88 × 36.32 × 3.37 | static |
| metal_plate | 金属盘 | 43.45 × 42.79 × 3.97 | static |

橘子、梨和啤酒瓶尺寸为仿真用途的人工假设，其余保留 FBX 导入尺寸。
动态物体带一个刚体及凸包碰撞，质量分别估计为 0.5、0.35、0.18、0.18 kg。
收纳箱和盘子使用原网格三角面静态碰撞，保留开口与凹面；不能直接改为 dynamic，若需抓取搬运，须另做凸分解/复合碰撞和刚体。

每个目录的 conversion.json 记录来源、缩放、质量和检查结果。
已检查 USD 可读、纹理依赖完整、几何索引、尺寸、原点与现有加载器要求，并用 Blender 离线渲染检查外观。
尚未在 Isaac Sim 运行掉落、接触或抓取测试。玻璃外观会随渲染器变化。
收纳箱源文件没有材质，使用浅灰色材质；啤酒瓶保留原材质并接入附带的水滴法线贴图。
预览见 new_objects_preview.png（各物体独立缩放展示，不代表相对尺寸；啤酒瓶预览为补接水滴法线前的外观）。

现有场景 JSON 可引用，例如：

```json
{
  "name": "Orange",
  "usd_path": "../../../../assets/objects/usd/orange/orange.usd",
  "mode": "dynamic",
  "position": [0.55, 0, 0.004],
  "orientation": [1, 0, 0, 0],
  "scale": [1, 1, 1]
}
```

示例假定桌面高度为 0；实际 position.z 应按桌面高度调整。
