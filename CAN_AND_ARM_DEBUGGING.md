# 历史 CAN-FD 调试记录（原按 OpenArm 2.0 误判）

> **历史文档警告（2026-07-28）：** 当前实机已确认为 OpenArm 1.0。
> 本文的 CAN-FD、达妙 ID 和总线排查记录仍可参考，但 v2 模型、零位姿态、
> offset 和左右接口旧结论已由 `INTERACTIVE_CONTROL.md` 取代。

更新日期：2026-07-22  
系统：Ubuntu 24.04 x86_64  
机械臂：当前已确认为 OpenArm 1.0 双臂，每臂 J1–J7 + J8 夹爪  
CAN 适配器：达妙 DM-USB2FDCAN 双通道，Linux `gs_usb`

## 1. 文档范围

本文只保留本次实机调试的最终结论、可复用操作和故障排查经验，不保留已经被后续验证推翻的中间状态。

本次调试完成了：

- 左右臂 16 台达妙电机的 ID、Master ID 和通信模式检查；
- 经典 CAN 1 Mbps 电机统一迁移到 OpenArm 默认 CAN-FD；
- 重复 ID 的物理隔离和修复；
- 左臂夹爪 ID 8 的识别和速率修改；
- `gs_usb` USB 发送通道卡死的定位与恢复；
- OpenArm 官方 `openarm-can-cli` 最终验收。

本次没有执行：

- `enable`；
- `set_zero`；
- 位置、速度、力矩控制；
- VR 真机遥操作。

## 2. 最终硬件状态

### 2.1 接口映射

| 机械臂 | Linux SocketCAN | 电机 ID |
|---|---|---|
| 右臂 | `can0` | `1–8` |
| 左臂 | `can1` | `1–8` |

不要只凭转接板 CAN1/CAN2 丝印推断 Linux 接口编号。USB 重新插拔或换驱动后，应通过已知电机的只读回复重新确认。

### 2.2 标准 ID

| 物理位置 | ESC / Sender ID | Master / Response ID |
|---|---:|---:|
| J1 | `0x01` | `0x11` |
| J2 | `0x02` | `0x12` |
| J3 | `0x03` | `0x13` |
| J4 | `0x04` | `0x14` |
| J5 | `0x05` | `0x15` |
| J6 | `0x06` | `0x16` |
| J7 | `0x07` | `0x17` |
| J8 / 夹爪 | `0x08` | `0x18` |

左右臂位于独立 CAN 总线，因此可以复用同一组 ID。

### 2.3 最终参数

左右两臂共 16 台电机全部满足：

- Control Mode：`1 (MIT)`；
- `can_br=9 (5M)`；
- CAN-FD 仲裁速率 1 Mbps；
- CAN-FD 数据速率 5 Mbps；
- ESC ID 为 `1–8`；
- Master ID 为 `0x11–0x18`；
- 无重复 ID和缺失电机；
- 无 Bus-Off、总线错误、仲裁错误和丢包。

## 3. SocketCAN 正确配置

本机 DM-USB2FDCAN 的 `gs_usb` 不支持 `restart-ms`。OpenArm 默认配置命令可能出现：

```text
Error: Device doesn't support restart from Bus Off.
```

这不表示适配器不支持 CAN-FD，只是不支持自动 Bus-Off restart。

推荐手动配置：

```bash
for iface in can0 can1; do
  sudo ip link set "$iface" down
  sudo ip link set "$iface" type can \
    bitrate 1000000 sample-point 0.75 \
    dbitrate 5000000 fd on \
    dsample-point 0.75 dsjw 2
  sudo ip link set "$iface" txqueuelen 100
  sudo ip link set "$iface" up
done
```

检查：

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

正常结果应包含：

```text
<NOARP,UP,LOWER_UP,ECHO>
mtu 72
can <FD> state ERROR-ACTIVE
bitrate 1000000 sample-point 0.750
dbitrate 5000000 dsample-point 0.750
```

`ERROR-ACTIVE` 是 CAN 控制器的正常状态名称，不代表当前正在报错。仍需结合 error、dropped 和内核日志判断。

## 4. OpenArm 官方工具验收

最终使用官方工具成功识别全部电机：

```bash
openarm-can-cli -i can0 show_param --id 1,2,3,4,5,6,7,8
openarm-can-cli -i can1 show_param --id 1,2,3,4,5,6,7,8
```

快速筛选关键参数：

```bash
for iface in can0 can1; do
  echo "===== $iface ====="
  openarm-can-cli -i "$iface" show_param \
    --id 1,2,3,4,5,6,7,8 2>&1 | \
    rg 'MOTOR ID:|Master ID|Motor \(ESC\) ID|Control Mode|CAN Baudrate|NO RESPONSE|Failed|ERROR'
done
```

`discover` 会切换多组速率，并可能因 `restart-ms` 在恢复阶段失败。ID 已知时优先使用 `show_param`。若必须运行 `discover`，结束后手动恢复 1M/5M，并再次执行 `ip -details` 和 `show_param`。

## 5. 达妙参数只读协议

参数请求使用仲裁 ID `0x7FF`：

```text
D0:D1 = 目标 ESC ID，小端
D2    = 0x33，读参数
D3    = RID
D4:D7 = 0
```

常用 RID：

| RID | 参数 |
|---:|---|
| `0x07` | Master ID |
| `0x08` | ESC ID |
| `0x0A` | Control Mode |
| `0x23` | CAN Baudrate / `can_br` |

CAN-FD + BRS 读取 ID 3 的 Master ID：

```bash
cansend can0 7FF##10300330700000000
```

正常回复：

```text
013##5 03 00 33 07 13 00 00 00
```

经典 CAN：

```bash
cansend can0 7FF#0300330700000000
```

## 6. 经典 CAN 与 CAN-FD

最初多数电机仍为：

```text
can_br=4
经典 CAN 1 Mbps
```

OpenArm 默认使用：

```text
can_br=9
CAN-FD 1 Mbps / 5 Mbps
```

CAN-FD 下无回复时，将接口明确切到经典模式再查询：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 fd off
sudo ip link set can0 up
```

仅在 FD 接口上发送经典帧，不一定等价于接口明确 `fd off`。本次左臂夹爪只有在关闭 FD 后才最初被发现。

## 7. 修改 can_br

以 ID 5 为例：

```bash
# 明确失能
cansend can0 005#FFFFFFFFFFFFFFFD

# 写 RID 0x23 = 9
cansend can0 7FF#0500552309000000

# 读回 RAM
cansend can0 7FF#0500332300000000

# 仅在读回为 9 后保存一次 Flash
cansend can0 7FF#0500AA0000000000
```

注意：

- `cansend` 成功只表示帧进入内核队列，不表示电机已经修改；
- 必须看到电机确认和读回值；
- 部分电机第一次拒绝写入，在“失能 → 等待 → 写入 → 读回”后成功；
- 不同固件切换速率的时机可能不同；
- Flash 有寿命限制，只在读回正确后保存一次；
- 保存后必须断电重启并再次验证。

## 8. 修改 ID

本次修复过：

- 一台 J3：ESC ID `1→3`，Master 保持 `0x13`；
- 右臂 J1：误设为 ID 3，与 J3 冲突，隔离后改为 ID 1 / Master `0x11`。

以 `3→1` 为例：

```bash
# 用旧 ID 修改 ESC ID
cansend can0 7FF#0300550801000000

# RAM 中已经切换到新 ID，后续使用新 ID
cansend can0 7FF#0100550711000000

# 读回
cansend can0 7FF#0100330800000000
cansend can0 7FF#0100330700000000

# 失能并保存
cansend can0 001#FFFFFFFFFFFFFFFD
cansend can0 7FF#0100AA0000000000
```

断电重启后必须确认旧 ID 不回复，新 ID、Master ID、MIT 和速率均正确。

## 9. 重复 ID：必须物理隔离

右臂 J1 曾与真正的 J3 同为 ID 3。表现：

- ID 1 缺失；
- ID 3 偶尔出现重复回复；
- 总线不一定 Bus-Off。

两台电机若发送相同仲裁 ID 和相同数据，CAN 总线可能只表现为一帧，因此不能通过回复数量准确统计同 ID 电机。

可靠流程：

1. 关闭动力电源；
2. 只连接疑似错误的单台电机；
3. 上电并扫描真实 ID；
4. 单独修改、读回和保存；
5. 断电验证；
6. 再接回整臂。

禁止在两台同 ID 电机同时在线时执行 `change_id`，否则命令会同时命中两台。

## 10. 左臂夹爪排查结论

夹爪最初为：

```text
ESC ID=8
Master ID=0x18
MIT
can_br=4，经典 CAN 1 Mbps
```

写入 `can_br=9` 后夹爪在旧模式下停止回复，随后新旧速率都无响应，一度误判为夹爪或线束故障。

最终根因是 USB-CAN 的 `can0` 发送通道卡死。重新插拔整个 USB-CAN 后：

- 已知正常 ID 2 电机恢复回复；
- 夹爪 ID 8 以 CAN-FD 5 Mbps 回复；
- 读回 `can_br=9`；
- 说明写入和保存实际成功。

目标电机在新旧速率都失联时，应先用已知正常电机验证适配器，不要继续写 Flash。

## 11. gs_usb 发送通道卡死

### 11.1 现象

接口看起来仍正常：

```text
UP / LOWER_UP / ERROR-ACTIVE
1 Mbps / 5 Mbps
```

但已知正常电机也无 ACK，TX errors/dropped 增长，内核日志出现：

```text
gs_usb ... can0: failed to xmit URB ... -ENOENT
gs_usb ... can0: Unexpected unused echo id ...
```

`ERROR-ACTIVE` 不能证明 USB 发送链路健康。

### 11.2 诱因

- 频繁切换速率和 down/up；
- 无 ACK 时队列积压；
- 中断扫描；
- 取消 sudo/pkexec 授权；
- 遗留 shell 等待 sudo。

### 11.3 检查

```bash
ip -details -statistics link show can0
journalctl -k -n 100 --no-pager | \
  rg -i 'gs_usb|failed to xmit|unused echo|can0'
pgrep -af 'candump|cansend|openarm'
```

### 11.4 恢复

1. 停止所有 CAN/OpenArm 进程；
2. 将接口置为 DOWN；
3. 重新插拔整个 USB-CAN；
4. 确认 USB 与 can0/can1 重新枚举；
5. 重新配置两路；
6. 用已知正常电机只读验证；
7. error/dropped 不再增加后再继续。

USB 重插后接口通常回到 DOWN，队列和计数器归零。

## 12. 授权中断与错误残留速率

取消 sudo/pkexec 可能使接口停在 250 kbps、1M/2M、1M/8M 或 DOWN。脚本即使打印 `RESTORED`，恢复命令也可能实际被拒绝。

必须检查：

```bash
ip -details link show can0
```

确认最终确实是 1M/5M 后，才能运行 OpenArm 控制程序。

## 13. 没有 ACK 不等于 ID 错误

- ID 不匹配：设备不返回参数，但同速率正常节点仍会 ACK 有效帧；
- 完全无 ACK：优先怀疑供电、H/L/GND、FD 模式、位速率或 USB-CAN 发送异常。

典型表现：

```text
write: No buffer space available
TX errors/dropped 增长
只有 0x7FF 本机回显
发送计数不增长
```

不要用无限扩大 ID 扫描解决无 ACK，应先恢复总线或 USB 适配器。

## 14. 达妙速率表

| can_br | 速率 |
|---:|---:|
| 0 | 125 kbps |
| 1 | 200 kbps |
| 2 | 250 kbps |
| 3 | 500 kbps |
| 4 | 1 Mbps，经典 CAN |
| 5 | 2 Mbps，CAN-FD |
| 6 | 2.5 Mbps，CAN-FD |
| 7 | 3.2 Mbps，CAN-FD |
| 8 | 4 Mbps，CAN-FD |
| 9 | 5 Mbps，CAN-FD |
| 10 | 8 Mbps，CAN-FD |
| 11 | 10 Mbps，CAN-FD |

全速率扫描时：

- 只扫必要 ID；
- 每次配置前 down；
- 经典 CAN 明确 `fd off`；
- 不添加 `restart-ms`；
- 不取消中途授权；
- 扫描后独立恢复并检查；
- 无 ACK 时不要高频连续发包。

## 15. 易错点速查

| 现象 | 常见误判 | 正确处理 |
|---|---|---|
| `ERROR-ACTIVE` | 认为正在报错 | 正常状态名，继续看计数器和日志 |
| `restart-ms` 失败 | 认为不支持 CAN-FD | 手动配置且不加该参数 |
| 只有 `0x7FF` | 认为电机回复 | 真正反馈应来自 `0x11–0x18` |
| `cansend` 成功 | 认为参数已写入 | 必须看电机确认和读回 |
| ID 扫不到 | 立即改 ID | 先检查 ACK、速率、FD 模式和 URB |
| 两台同 ID | 在线改其中一台 | 必须物理隔离 |
| ESC ID 已改 | 后续仍用旧 ID | RAM 已切换，后续用新 ID |
| 保存 Flash | 循环重复保存 | 读回正确后只保存一次 |
| No buffer space | 只增大 qlen | 根因通常是无 ACK |
| 取消授权 | 相信脚本恢复 | 手工检查最终接口状态 |
| USB 重插 | 忘记配置接口 | 接口通常回到 DOWN |
| 正常电机也失联 | 继续怀疑目标电机 | 检查 gs_usb URB，必要时重插 |
| 板上 CAN1/CAN2 | 直接映射 can0/can1 | 用实际只读查询确认 |

## 16. 真机运动前清单

- [x] `can0` 左臂 ID 1–8 被官方 CLI 识别；
- [x] `can1` 右臂 ID 1–8 被官方 CLI 识别；
- [x] Master ID、MIT、`can_br=9` 正确；
- [x] 当前 CAN 无错误和丢包；
- [ ] 核对电机型号与物理关节；
- [x] 型号纠正为 OpenArm 1.0；旧 v2 零位校准项作废；
- [ ] 动作入口拒绝 NaN、Infinity 和错误维度；
- [ ] 加入失联、目标跳变和状态偏差保护；
- [ ] MuJoCo 同动作测试通过；
- [ ] 固定底座、清空工作区、准备急停；
- [ ] 首次真机采用低增益、小幅度、单关节测试。

真机前动作入口至少增加：

```python
positions = np.asarray(position, dtype=np.float64)
if positions.shape != (8,):
    raise RuntimeError("Expected exactly 8 joint values")
if not np.all(np.isfinite(positions)):
    raise RuntimeError("Joint action contains NaN or Infinity")
```

在剩余检查完成前，不要直接批量 enable、set_zero、连续位置控制或 VR 真机遥操作。
