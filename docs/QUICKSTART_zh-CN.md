# 中文快速开始

本教程适用于 Ubuntu 24.04、Python 3.12 和 OpenArm 1.0。建议先完成纯仿真，
再接入 VIVE，最后才连接实体机械臂。SteamVR 请使用“Ubuntu on Xorg”会话。

## 1. 安装依赖

```bash
sudo apt update
sudo apt install python3.12 python3.12-venv python3-pip git \
  can-utils iproute2 usbutils libgl1 libglfw3 libusb-1.0-0
```

```bash
git clone https://github.com/robin1hub/open-arm-teleop.git
cd open-arm-teleop
./bootstrap_portable.sh
```

脚本会创建 `.venv`、安装锁定版本的依赖、以 editable 模式安装本地节点，
并执行基础安全回归测试。

## 2. 先验证纯仿真

```bash
.venv/bin/python -m unittest -q \
  tests/test_vive_real_soft_limits.py \
  tests/test_safe_ik_limits.py \
  tests/test_ee_to_joint_mujoco.py
./scripts/launch/run_action_test_mujoco.sh
```

此阶段不需要 VIVE、CAN 或实体机械臂。仿真中的左右臂应平滑地从 home 位
移动到镜像目标再返回。若测试或动作异常，不要继续接入实机。

## 3. VIVE + MuJoCo

通过 Steam 安装 SteamVR，确认头显、基站和两个控制器在 SteamVR 窗口中均为
绿色，然后执行：

```bash
./scripts/launch/run_vive_shared_mujoco_sim.sh
```

- `K`：校准 VIVE 到机械臂的映射
- 扳机或 `C`：采集当前参考位姿
- `Esc`：退出

先用小幅、缓慢动作验证方向、比例和左右臂对应关系。

## 4. 接入实体机械臂

先完整阅读[硬件安全清单](HARDWARE_SAFETY.md)。保持电机输出关闭，检查两个
CAN-FD 总线：

```bash
sudo openarm-can-cli can_configure
ip -details link show can0
ip -details link show can1
openarm-can-cli -i can0 discover
openarm-can-cli -i can1 discover
```

两个接口都应为 `UP`、CAN-FD 速率正确、状态为 `ERROR-ACTIVE`，并分别发现
预期的 8 个电机 ID。本机通常是右臂=`can0`、左臂=`can1`，但重新插拔 USB
后编号可能互换，必须以实际发现的电机身份为准。

检查全部通过后启动：

```bash
./scripts/launch/run_vive_shared_mujoco_real.sh --confirm-hardware
```

程序启动时实体输出仍为关闭状态。先按 `K` 校准，再按 `C` 或扳机采集参考，
确认仿真和实机位姿一致、工作区无人且急停可触及后，才按 `E` 使能输出。

- `E`：使能/关闭实体输出
- `P`：关闭输出并重新同步目标
- `H`：执行受速度限制的回 home 动作
- `Esc`：安全退出

若跟踪、CAN 反馈或动作异常，应立即关闭输出或按实体急停。不要通过放宽保护
限制来绕过故障。常见问题见[故障排查](TROUBLESHOOTING.md)。
