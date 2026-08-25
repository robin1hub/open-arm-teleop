# Windows 下配置 OpenArm 2.0 达妙电机 ID

适用设备：OpenArm 2.0 双臂、达妙电机  
操作系统：Windows 10/11  
目标：为左右臂分别设置 J1–J8 的 Sender/ESC ID 与 Receiver/Master ID  
重要：本文只配置和读取 ID，不校准零位、不使能电机、不测试运动

## 1. 最终目标

每条机械臂都必须独立使用下面的 ID 表：

| 物理关节 | Sender / ESC ID | Receiver / Master ID | 十进制 Master ID |
|---|---:|---:|---:|
| J1 | `0x01` | `0x11` | 17 |
| J2 | `0x02` | `0x12` | 18 |
| J3 | `0x03` | `0x13` | 19 |
| J4 | `0x04` | `0x14` | 20 |
| J5 | `0x05` | `0x15` | 21 |
| J6 | `0x06` | `0x16` | 22 |
| J7 | `0x07` | `0x17` | 23 |
| J8（夹爪） | `0x08` | `0x18` | 24 |

左右臂位于不同 CAN 总线，因此左右臂都使用同一组 ID。不要把左臂设置成 9–16。

## 2. 当前已知问题

### 右臂

- 命令 ID `0x01` 的电机实际从 `0x13` 返回，Master ID 错误；
- 命令 ID `0x02` / Master ID `0x12` 正常；
- J3–J8 当前没有正常反馈。

### 左臂

- 多个或全部电机可能仍为默认 Sender ID `0x01`；
- 并联后会同时响应同一查询并产生冲突；
- 必须逐个隔离配置。

因此两条机械臂都需要逐个核对 8 个电机，不能只处理左臂。

## 3. 安全要求

操作前必须：

- 将机械臂牢固固定；
- 清空机械臂运动范围；
- 准备急停或可立即断开的 24V 总电源；
- 换线和拆线时关闭 24V 电源；
- 一次只连接一个待配置电机；
- 给每个电机/线缆贴上物理关节标签；
- 使用限流电源，设置合理电流上限；
- 确认调试器 GND 与电机 GND 相连。

本流程严禁：

- 点击 `Calibrate`；
- 点击 `Enter` 进入电机控制；
- 点击 `Send` 发送运动命令；
- 设置 Torque/Velocity/Position 测试值；
- 修改机械零位；
- 更新固件；
- 在多个相同 ID 电机并联时写 ID；
- 带电插拔电源、UART 或 CAN 接头。

## 4. 需要准备的工具

- Windows 10/11 电脑；
- 达妙官方 Debugging Tools；
- 达妙 UART 调试器或官方文档指定的调试连接设备；
- 对应 UART 线束；
- 24V 直流电源；
- 急停或便于快速断开的电源开关；
- 标签纸/记号笔；
- 本文末尾的配置记录表。

注意：官方 Windows 调试工具通过 UART 通信，不是通过 Linux SocketCAN 的 `can0/can1`。UART 波特率通常为：

```text
921600 bps
```

DM-USB2FDCAN 是否能被该 Windows 工具作为 UART 使用，取决于具体硬件版本和驱动。如果软件中没有对应 COM 口，需要使用达妙官方 UART 调试器，而不是强行选择其他串口。

## 5. 安装 Windows 调试软件

1. 从 OpenArm 官方电机 ID 配置文档提供的链接下载达妙 Debugging Tools；
2. 解压或安装到不含中文和特殊字符的短路径，例如：

   ```text
   C:\DamiaoTool\
   ```

3. 安装调试器驱动；
4. 打开 Windows 设备管理器；
5. 展开“端口 (COM 和 LPT)”；
6. 插入调试器，记录新出现的 COM 端口；
7. 启动达妙 Debugging Tools；
8. 如果界面为中文/英文，可使用左下角语言切换按钮；
9. 在软件中选择对应 COM 口；
10. 串口波特率选择 `921600`。

如果 Windows Defender 阻止程序，不要永久关闭 Defender。检查“Windows 安全中心 → 病毒和威胁防护 → 保护历史记录”，确认文件确实来自官方来源后，只允许该文件运行。

## 6. 确认物理 J1–J8

写入 ID 前，必须能明确每个电机对应哪个物理关节。

推荐从基座向末端统一编号：

```text
基座 → J1 → J2 → J3 → J4 → J5 → J6 → J7 → J8/夹爪
```

不要仅根据当前 CAN ID判断物理关节，因为多个电机可能都是 ID 1。

如果无法确定某个电机对应的 J 编号：

- 查看 OpenArm 装配图和线束图；
- 查看电机安装位置和型号；
- 向整机供应商确认；
- 不要通过让电机转动来猜测。

给左右臂分别贴标签：

```text
R-J1 ... R-J8
L-J1 ... L-J8
```

## 7. 单个电机配置流程

下面流程对每个电机重复一次。

### 7.1 断电和隔离

1. 关闭 24V 电源；
2. 等待电源完全放电；
3. 将其他电机从通信总线隔离；
4. 确保本次只有一个电机连接到调试器；
5. 再次确认其物理标签，例如 `L-J3`。

如果无法逐个断开电机通信，则停止操作，不要在整臂并联状态写 ID。

### 7.2 连接

按照达妙电机和调试器官方针脚定义连接：

- UART TX/RX 交叉连接；
- GND 必须共地；
- 电机使用独立 24V 电源；
- 不根据线色猜测针脚；
- 先连接信号，再连接电源，最后上电。

### 7.3 读取当前参数

1. 打开 24V 电源；
2. 在 Windows 工具中打开正确 COM 口；
3. 点击 `ReadParam`；
4. 确认读取成功；
5. 记录以下信息：

   - 当前 Sender/ESC ID；
   - 当前 Receiver/Master ID；
   - CAN baudrate；
   - Control Mode；
   - 电机型号；
   - Hardware Version；
   - Firmware/Software Version；
   - Serial Number（若可读取）。

如果 `ReadParam` 失败：

- 检查 COM 口；
- 检查 UART 波特率是否为 921600；
- 检查 TX/RX 是否接反；
- 检查 GND；
- 检查 24V 电源；
- 不要反复点击 WriteParam。

### 7.4 设置 ID

根据物理关节查表。例如配置左臂 J3：

```text
Sender / ESC ID = 0x03
Receiver / Master ID = 0x13
```

如果工具使用十进制输入：

```text
Sender = 3
Master = 19
```

注意不要把 Sender 和 Master 填反。

### 7.5 保存

1. 仔细核对物理关节标签；
2. 仔细核对 Sender ID；
3. 仔细核对 Master ID；
4. 截图保存写入前的界面；
5. 点击一次 `WriteParam`；
6. 等待明确的成功提示；
7. 不要连续点击；
8. 不要把写参数放入循环。

Flash 有写入寿命，只有必要时才保存。

### 7.6 断电验证

1. 关闭 24V 电源；
2. 等待数秒；
3. 重新上电；
4. 再次点击 `ReadParam`；
5. 确认 Sender/Master ID 在重启后仍正确；
6. 将结果填入记录表；
7. 关闭电源；
8. 断开当前电机；
9. 才能连接下一个电机。

## 8. 推荐配置顺序

为了减少混淆，每条手臂从夹爪向基座或从基座向夹爪固定一个方向，不要中途改变。

推荐：

```text
右臂：R-J1 → R-J2 → ... → R-J8
左臂：L-J1 → L-J2 → ... → L-J8
```

每完成一个电机立即：

- 在电机/线束上贴 ID；
- 保存截图；
- 填写记录表；
- 标记“写入后已断电重读”。

## 9. 右臂特别检查

右臂 J1 当前观测到：

```text
Sender ID = 0x01
实际 Master ID = 0x13
```

目标应为：

```text
Sender ID = 0x01
Master ID = 0x11
```

必须确认该响应电机确实是物理 R-J1，不能只根据 Sender ID 推断。

右臂 ID 2 的反馈 ID `0x12` 正常，但仍应通过 UART 读取确认其物理位置确实是 R-J2。

## 10. 左臂特别检查

左臂多个电机可能都是默认：

```text
Sender ID = 0x01
```

因此左臂在整臂并联状态下不能通信，也不能使用 CAN 批量修改 ID。必须逐个物理隔离后配置。

不要先把所有电机依次改成 2，再改成 3。每个物理关节必须直接写入其最终 ID 对。

## 11. 配置完成后的整臂检查

完成单臂 8 个电机后：

1. 关闭电源；
2. 恢复该机械臂的内部电源与 CAN 线束；
3. 检查不存在松动、反插或夹线；
4. 接到对应的达妙 CAN 端口；
5. 再打开 24V 电源；
6. 此阶段仍不要使能电机。

回到 Ubuntu 后配置接口：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can \
  bitrate 1000000 sample-point 0.75 \
  dbitrate 5000000 dsample-point 0.75 dsjw 2 \
  fd on loopback off
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can \
  bitrate 1000000 sample-point 0.75 \
  dbitrate 5000000 dsample-point 0.75 dsjw 2 \
  fd on loopback off
sudo ip link set can1 up
```

然后只读发现：

```bash
openarm-can-cli -i can0 discover
openarm-can-cli -i can1 discover
```

左右臂都应分别显示 8 对唯一映射：

```text
0x01 → 0x11
0x02 → 0x12
0x03 → 0x13
0x04 → 0x14
0x05 → 0x15
0x06 → 0x16
0x07 → 0x17
0x08 → 0x18
```

注意：`discover` 可能在结束时把 CAN 接口留在扫描使用的其他数据速率。发现完成后，应重新执行正常 1M/5M 配置，再进行 `show_param`。

只读参数检查：

```bash
openarm-can-cli -i can0 show_param --id 1,2,3,4,5,6,7,8
openarm-can-cli -i can1 show_param --id 1,2,3,4,5,6,7,8
```

## 12. 配置记录表

### 右臂

| 关节 | 电机型号 | 原 Sender | 原 Master | 新 Sender | 新 Master | CAN Baud | Control Mode | 序列号 | 写入成功 | 断电重读成功 |
|---|---|---:|---:|---:|---:|---|---|---|---|---|
| R-J1 | | | | 1 | 17 | | | | | |
| R-J2 | | | | 2 | 18 | | | | | |
| R-J3 | | | | 3 | 19 | | | | | |
| R-J4 | | | | 4 | 20 | | | | | |
| R-J5 | | | | 5 | 21 | | | | | |
| R-J6 | | | | 6 | 22 | | | | | |
| R-J7 | | | | 7 | 23 | | | | | |
| R-J8 | | | | 8 | 24 | | | | | |

### 左臂

| 关节 | 电机型号 | 原 Sender | 原 Master | 新 Sender | 新 Master | CAN Baud | Control Mode | 序列号 | 写入成功 | 断电重读成功 |
|---|---|---:|---:|---:|---|---|---|---|---|---|
| L-J1 | | | | 1 | 17 | | | | | |
| L-J2 | | | | 2 | 18 | | | | | |
| L-J3 | | | | 3 | 19 | | | | | |
| L-J4 | | | | 4 | 20 | | | | | |
| L-J5 | | | | 5 | 21 | | | | | |
| L-J6 | | | | 6 | 22 | | | | | |
| L-J7 | | | | 7 | 23 | | | | | |
| L-J8 | | | | 8 | 24 | | | | | |

## 13. 完成判定

Windows ID 配置只有满足以下条件才算完成：

- [ ] 16 个电机全部逐个读取过；
- [ ] 16 个电机均有明确物理关节标签；
- [ ] Sender ID 与物理 J 编号一致；
- [ ] Master ID 等于 Sender ID + `0x10`；
- [ ] 每个电机写入后均断电重读；
- [ ] 保存了参数记录和截图；
- [ ] 未执行零位校准、固件更新或运动测试；
- [ ] 恢复整臂后左右总线各发现 8 个唯一 ID；
- [ ] Ubuntu `show_param` 结果与记录表一致。

达到这些条件后，仍然只能进入“只读状态监测与小动作前检查”，不能直接开始 VR 真机遥操作。

## 14. 参考资料

- OpenArm 2.0 Motor ID Configuration：<https://docs.openarm.dev/api-reference/setup/motor-id/>
- OpenArm CAN CLI：<https://docs.openarm.dev/api-reference/can/cli/>
- 本项目 CAN 调试记录：`CAN_AND_ARM_DEBUGGING.md`

