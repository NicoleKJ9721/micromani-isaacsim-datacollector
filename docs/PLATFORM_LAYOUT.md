# 双臂间距与面包板

2026-11-09：左右底座相对原 CAD 各向外移动 30 mm，固定关节 X 坐标由 ±201 mm 改为 ±231 mm，中心距 462 mm。已同步 URDF；原 URDF 备份在 `outputs/spaced_breadboard_final_20261109/original_robot.urdf`。默认启动入口为 `scripts/open_simulation.sh`。

`config/platform_layout.json` 使用米为单位：`arm_outward_offset_m` 是相对原 CAD 的单侧外移量，重复运行不会叠加。`breadboard` 可调整长、宽、厚、孔距、孔径、孔深、孔中心到边缘的最小间距。当前按用户修正为 1200(X)×800(Y)×10 mm，保留中心位置，采用 47×31 个孔，孔距 25 mm，直径 6 mm，深度 4 mm。这些孔参数是待实物确认的可调外观假设，不包含螺纹或隔震动力学。USD 碰撞体同步生成整板包围盒，URDF 碰撞体已同步。固定底座的质量参数暂保留，未按未知实际结构推算质量。groundPlane 已移除。

重建到一个不存在的新目录：

```bash
/home/nkj/isaacsim-5.1.0/python.sh isaacsim_tools/configure_platform_layout.py \
  --stage outputs/breadboard_1200x800_20261109/micromani_equipped.usda \
  --out outputs/layout_revision
```

脚本生成 USD、STL、全景、面包板近景及 `layout_report.json`，运行 60 个仿真步检查固定底座不回弹。通过后可使用 `MICROMANI_STAGE` 指定新场景启动。此脚本只生成新输出，不自动修改 URDF；需要重导入 URDF 时，将新 STL 复制到 `meshes/base_breadboard.STL`，同步 `joint0_left/right` 的 X 坐标为 ±(0.201 + 单侧外移量)。

当前面包板已按 800×1200 mm 扩展，替代原来的 groundPlane 平板。双臂原有安装高度和各部件产品配色保留。
