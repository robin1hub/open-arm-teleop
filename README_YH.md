# OpenArm 用户侧文档索引

## YANGHAO 当前控制入口

搜索关键词：`YANGHAO`、`YANGHAO_OPENARM`、`YANGHAO_CONTROL`

- 当前单窗口 MuJoCo + 实体跟随快速说明：
  [`YANGHAO_OPENARM_CONTROL.md`](YANGHAO_OPENARM_CONTROL.md)
- 完整控制与实现记录：
  [`INTERACTIVE_CONTROL.md`](INTERACTIVE_CONTROL.md)

> 当前实机已于 2026-07-28 确认为 **OpenArm 1.0**。旧调试记录中出现的
> “OpenArm 2.0”是此前的错误型号判断；实体交互控制以
> `INTERACTIVE_CONTROL.md` 和 v1 模型为准。

## MuJoCo 拖动与实体控制

交互式仿真、实体模式启动方法、按键和首次慢速操作流程见：

- [`INTERACTIVE_CONTROL.md`](INTERACTIVE_CONTROL.md)

更新日期：2026-07-28

## 最终 CAN 调试文档

- 文档名称：`CAN_AND_ARM_DEBUGGING.md`
- 相对位置：`./CAN_AND_ARM_DEBUGGING.md`
- 绝对位置：`/home/robin/open_arm/CAN_AND_ARM_DEBUGGING.md`
- 状态：最终实测版；早期失败状态已移除

### 内容摘要

该文档记录早期双臂实机 CAN-FD 配置与完整排查经验。文档产生时型号曾被误判为
OpenArm 2.0，因此其中涉及机器人模型、零位 offset 和实体控制的结论已被
`INTERACTIVE_CONTROL.md` 取代；CAN-FD 和达妙电机排查方法仍可参考。

- 当前最终映射：右臂 `can0`，左臂 `can1`；
- 两臂 ID `1–8`、Master ID `0x11–0x18`；
- MIT 模式和 `can_br=9`；
- DM-USB2FDCAN 不支持 `restart-ms` 时的手动配置；
- 经典 CAN 电机迁移到 CAN-FD 1M/5M；
- 电机 ID 和 Master ID 修改流程；
- 重复 ID 必须物理隔离的原因与操作；
- 左臂夹爪 ID 8 的定位和恢复过程；
- `gs_usb failed to xmit URB -ENOENT` 发送通道卡死；
- sudo/pkexec 中断导致接口残留错误速率；
- “没有 ACK”和“ID 不匹配”的区别；
- OpenArm 官方 `openarm-can-cli show_param` 验收命令；
- 真机使能、零位校准和 VR 遥操作前的安全清单。

### 当前验收结论（2026-07-28）

- 右臂 `can0`：ID `1–8` 全部被 OpenArm 官方工具识别；
- 左臂 `can1`：ID `1–8` 全部被 OpenArm 官方工具识别；
- 16 台电机均为 MIT、CAN-FD 1 Mbps/5 Mbps；
- CAN 配置阶段完成；
- 实机型号已确认为 OpenArm 1.0；
- 零位已经写入，当前本机使用零 offset 坐标；
- v1 J1–J7 实机/MuJoCo 只读映射已验证；
- v1 交互式实体控制入口已实现，但夹爪实体控制和 VR 实体遥操作仍待迁移。

## 相关文档

| 文档 | 相对位置 | 用途 |
|---|---|---|
| 本机快速说明 | `./LOCAL_SETUP.md` | 仿真和真机启动入口 |
| Windows 达妙配置 | `./WINDOWS_DAMIAO_MOTOR_ID_SETUP.md` | UART/Windows 电机 ID 配置 |
| VIVE Pro 2 桥接 | `./WINDOWS_VIVE_BRIDGE_GUIDE.md` | SteamVR 遥操作数据桥接 |
| VIVE Linux 纯仿真 | `./run_vive_mujoco_sim.sh` | `YANGHAO_VIVE_BIMANUAL`：SteamVR 双手柄同时控制 OpenArm 1.0 双臂位置、方向和夹爪；IK 失败自动复位并继续跟随 |
| VIVE 双臂实体控制 | `./run_vive_mujoco_real.sh --confirm-hardware` | `YANGHAO_VIVE_REAL`：显式 E 使能、P 失能同步；独立实机限速、关节限位、下一步碰撞预测和异常双臂失能 |
| VIVE 绝对位姿纯仿真 | `./run_vive_absolute_mujoco_sim.sh` | `YANGHAO_VIVE_ABSOLUTE`：按 C 固定人体坐标标定，双手在绝对空间中的位置与方向映射到双臂；旧相对方案保留 |
| VIVE 共享绝对空间仿真 | `./run_vive_shared_mujoco_sim.sh` | `YANGHAO_VIVE_SHARED`：使用手柄 tip 位姿和统一空间变换；双手柄重合时原始双臂目标重合 |
| VIVE 绝对位姿实体控制 | `./run_vive_absolute_mujoco_real.sh --confirm-hardware` | `YANGHAO_VIVE_ABSOLUTE_REAL`：默认失能，C 标定、E 使能；连续投影、关节限位、躯干/双臂碰撞检查 |
| VIVE 共享绝对位姿实体控制 | `./run_vive_shared_mujoco_real.sh --confirm-hardware` | `YANGHAO_VIVE_ABSOLUTE_REAL`：三步共享标定、默认失能；H 限速安全回位 |
| v1 交互控制 | `./INTERACTIVE_CONTROL.md` | 当前有效的仿真与实体控制说明 |
| v1 映射检查 | `./verify_real_mujoco_alignment_v1.py` | 失能状态下验证实机/MuJoCo |

进入真机运动前，应先阅读最终 CAN 调试文档的“真机运动前清单”。
