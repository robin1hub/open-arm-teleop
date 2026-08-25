# YANGHAO — OpenArm 1.0 当前控制版本

> 搜索关键词：`YANGHAO`、`YANGHAO_OPENARM`、`YANGHAO_CONTROL`
>
> 更新日期：2026-07-28。本文是杨浩当前使用的 OpenArm 1.0 单窗口
> MuJoCo/实体控制快速入口。

## 当前版本

- 实机型号：OpenArm 1.0；
- 右臂：`can0`；左臂：`can1`；
- 界面：单个 MuJoCo 原生 `viewer` 窗口；
- 右侧原生 **Control**：左右臂 J1–J7 执行器滑条；
- 绿色大球：当前活动臂末端目标；蓝色小球：另一臂目标；
- 实体跟随速度上限：约 `20°/s`（40 Hz，每周期 0.0087 rad）；
- 实体夹爪保持当前角度，暂不跟随仿真夹爪滑条。

主程序：

```text
interactive_mujoco_ee_drag.py
```

完整说明：[INTERACTIVE_CONTROL.md](INTERACTIVE_CONTROL.md)

## 启动

纯仿真：

```bash
cd /home/robin/open_arm
.venv/bin/python interactive_mujoco_ee_drag.py --model-version v1
```

实体控制：

```bash
cd /home/robin/open_arm
./run_interactive_real.sh --confirm-hardware
```

实体模式启动时只读取双臂位置并初始化仿真，不会自动使能电机。

## 操作

- `1`：选择左臂；
- `2`：选择右臂；
- `Tab`：切换活动臂，并失能此前活动臂；
- `E`：使能/失能当前活动臂；
- `F`：开启/关闭实体连续跟随；
- `D`：实体臂单次慢速移动到当前仿真姿态；
- `P`：失能并将仿真同步到双臂实体姿态；
- `R`：把活动目标复位到当前末端位置；
- `Space`：暂停/恢复 IK；
- `Esc`：退出并失能实体臂。

末端目标使用 MuJoCo 原生物体操作：

1. 双击绿色或蓝色目标球将其选中；
2. `Ctrl + 右键拖动`平移；
3. `Ctrl + 左键拖动`旋转；
4. 普通鼠标操作用于旋转、平移和缩放视角。

## F 跟随规则

`F` 开启后，实体臂按仿真 J1–J7 目标逐步跟随。

- IK 目标不可达：自动执行等效 `R`，目标球回到当前可达末端位置，`F` 保持开启；
- 预测碰撞、实机关节越限或通信异常：停止跟随；
- 切换机械臂或按 `E` 失能：停止跟随；
- 每条实体指令仍经过限速、关节限位和下一步碰撞检查。

## 无滑条旧版

此前的自定义 GLFW 拖动窗口已冻结归档：

```bash
./run_interactive_legacy_no_sliders.sh
./run_interactive_legacy_no_sliders.sh --real
```

归档源文件：

```text
archives/interactive_mujoco_ee_drag_no_sliders_2026-07-28.py
```

## 安全

实体控制前清空工作空间、固定底座并确保急停触手可及。首次操作先选择单臂，
用很小的仿真动作验证方向。发生方向错误、碰撞趋势、振动、异常声音、红灯或
通信错误时立即按急停。
