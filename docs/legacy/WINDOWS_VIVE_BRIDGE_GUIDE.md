# Windows 智能体任务书：VIVE Pro 2 遥操作数据桥接

## 1. 背景与最终目标

我们正在搭建 OpenArm 2.0 双臂遥操作系统：

```text
VIVE Pro 2 + 两只手柄 + SteamVR（Windows）
                  │
                  ▼
          Windows VIVE Bridge
                  │ UDP / 局域网
                  ▼
       Ubuntu OpenArm 控制机（Dora）
                  │
                  ▼
        IK → MuJoCo 或真实 OpenArm
```

Windows 侧只负责：

1. 配置并验证 VIVE Pro 2、基站和控制器；
2. 从 SteamVR 读取 HMD 与左右控制器的数据；
3. 做设备识别、时间戳和追踪有效性检查；
4. 按本文协议通过 UDP 将原始追踪数据发送给 Ubuntu；
5. 提供诊断界面/日志，便于先在 MuJoCo 中验证。

Windows 侧不得直接控制真实机械臂、不得生成 CAN 帧，也不得自行使能电机。坐标标定、IK、关节限位和 CAN 控制由 Ubuntu 侧完成。

## 2. 开始前必须确认

请先只读检查并向用户报告：

- Windows 版本；
- CPU、GPU 与显卡驱动版本；
- VIVE Pro 2 是否通过 Link Box 2.0 连接到 DisplayPort 和 USB 3.0；
- 基站数量与版本；
- 是否有两只 VIVE 控制器；
- Windows 与 Ubuntu 是否处于同一局域网；
- Windows 防火墙是否允许本程序发送 UDP；
- Ubuntu 控制机的 IPv4 地址。

任何型号、连接方式或网络地址不明确时，先询问用户，不要猜测。

## 3. VIVE/SteamVR 软件配置

建议安装顺序：

1. 安装或更新显卡官方驱动；
2. 安装 Steam；
3. 安装 SteamVR；
4. 安装 VIVE Console for SteamVR；
5. 连接 Link Box、头显和基站；
6. 配对左右控制器；
7. 运行 SteamVR 房间设置，使用 Standing 模式；
8. 确认 HMD、左右控制器和所有基站在 SteamVR 中均正常；
9. 检查两只控制器在静止时是否稳定、移动时是否连续、遮挡时是否正确变为无效。

完成后记录软件版本和硬件序列号，但不要把个人账户信息、访问令牌或其他敏感数据写入项目。

## 4. Bridge 技术路线

第一版建议使用 Valve OpenVR API，因为它与 SteamVR/VIVE 的设备角色和追踪数据集成直接。后续可以改用 OpenXR，但 UDP 对外协议必须保持兼容。

推荐实现语言：

- 生产版本：C++17 + 官方 OpenVR SDK；
- 快速原型：Python + OpenVR 绑定，但必须固定依赖版本并说明其来源。

不要根据 SteamVR 临时设备索引硬编码左右手。必须通过控制器角色识别：

```text
TrackedControllerRole_LeftHand
TrackedControllerRole_RightHand
```

追踪空间使用：

```text
TrackingUniverseStanding
```

OpenVR 位姿来自 3×4 变换矩阵。提取：

- position：矩阵最后一列，单位为米；
- rotation：3×3 旋转矩阵转换为归一化四元数；
- quaternion 输出顺序固定为 `[qw, qx, qy, qz]`。

OpenVR 坐标保持原样发送，不在 Windows 端猜测 OpenArm 基座方向。OpenVR 坐标约定为右手系：X 向右、Y 向上、Z 向后。Ubuntu 端负责标定和轴映射。

## 5. 必须读取的数据

每个采样周期至少读取：

### HMD

- 位置；
- 姿态；
- `connected`；
- `pose_valid`；
- tracking result/status。

### 左右控制器

- 位置；
- 姿态；
- `connected`；
- `pose_valid`；
- tracking result/status；
- 食指扳机模拟量 `0.0–1.0`；
- Grip/握把状态，作为 deadman 候选；
- 菜单键及可用的 A/B/X/Y 或等价按钮；
- 可选：线速度和角速度。

输入建议通过 OpenVR Action System/Action Manifest 绑定，不依赖旧式固定按钮编号。若 VIVE Wand 不具备 A/B/X/Y，应将实际可用按钮映射成语义动作，例如 `record`, `cancel`, `calibrate`, `stop`。

## 6. UDP 数据协议

默认目标：

```text
Ubuntu_IP:5006/UDP
```

IP、端口和发送频率必须通过命令行参数或配置文件设置，不得硬编码。

建议发送频率与 SteamVR 刷新同步，通常约 90 Hz。每个 UDP 数据报包含一个完整采样，使用 UTF-8 JSON：

```json
{
  "protocol": "openarm-vive-v1",
  "sequence": 1024,
  "timestamp_ns": 1784600000000000000,
  "source": {
    "runtime": "SteamVR/OpenVR",
    "tracking_space": "standing",
    "coordinate_system": "right-handed-x-right-y-up-z-back"
  },
  "hmd": {
    "connected": true,
    "valid": true,
    "tracking_status": "running_ok",
    "position": [0.0, 1.65, 0.0],
    "quaternion": [1.0, 0.0, 0.0, 0.0]
  },
  "left": {
    "connected": true,
    "valid": true,
    "tracking_status": "running_ok",
    "position": [-0.25, 1.15, -0.35],
    "quaternion": [1.0, 0.0, 0.0, 0.0],
    "trigger": 0.0,
    "grip": 0.0,
    "deadman": false
  },
  "right": {
    "connected": true,
    "valid": true,
    "tracking_status": "running_ok",
    "position": [0.25, 1.15, -0.35],
    "quaternion": [1.0, 0.0, 0.0, 0.0],
    "trigger": 0.0,
    "grip": 0.0,
    "deadman": false
  },
  "actions": {
    "calibrate": false,
    "record": false,
    "success": false,
    "failure": false,
    "stop": false
  }
}
```

协议要求：

- `sequence`：从 0 单调递增的无符号整数；
- `timestamp_ns`：Windows 发送程序的单调时钟纳秒值，不使用本地日期字符串；
- position：长度 3，`float`，单位米；
- quaternion：长度 4，顺序 `[qw,qx,qy,qz]`，发送前归一化；
- trigger/grip：限制到 `[0,1]`；
- 无效追踪时保留字段结构，但 `valid=false`，不得伪造上一帧为有效数据；
- JSON 中不得出现 `NaN`、`Infinity` 或缺失的必选字段；
- 每个数据报应小于常规以太网 MTU，禁止拆包；
- 不要从 Ubuntu 接收任何“使能电机”指令。第一版保持单向数据通道。

## 7. Bridge 命令行接口

至少支持：

```powershell
OpenArmViveBridge.exe `
  --host 192.168.50.143 `
  --port 5006 `
  --rate 90 `
  --dry-run
```

参数语义：

- `--host`：Ubuntu IPv4 地址；
- `--port`：UDP 端口，默认 5006；
- `--rate`：最大发送频率；
- `--dry-run`：只读取并显示数据，不发送网络包；
- `--log-level`：日志级别；
- 可选 `--record-jsonl PATH`：保存原始测试数据用于离线复现。

程序启动后必须打印：

- SteamVR/OpenVR 初始化结果；
- HMD 和左右控制器是否找到；
- 设备型号/序列号；
- 当前追踪空间；
- UDP 目标；
- 实际采样/发送频率；
- 丢帧、无效追踪和异常数量。

不得每帧刷屏；状态变化时打印，并周期性输出汇总。

## 8. Windows 侧安全行为

Bridge 必须遵守：

1. 任一控制器断开或追踪无效时，立即发送 `valid=false`；
2. SteamVR 退出时发送数帧无效/停止状态，然后退出；
3. 不复用过期位姿冒充新采样；
4. deadman 默认必须是 `false`；
5. deadman 只能来自操作者持续按住的实体输入；
6. 程序崩溃后不得自动重启并恢复为 active；
7. 不直接做 IK，不输出关节角，不访问 CAN；
8. 不提供绕过急停、追踪有效性或 Ubuntu 安全状态机的选项。

## 9. 分阶段验收

### 阶段 A：SteamVR 本地 dry-run

不连接 Ubuntu，不发送数据。确认：

- 正确识别左/右控制器；
- position 随平移连续变化；
- quaternion 已归一化且旋转连续；
- trigger/grip 范围正确；
- 遮挡或关闭控制器时 `valid=false`；
- 设备重连后不会左右手互换。

### 阶段 B：UDP 回环测试

发送到 Windows 本机测试接收器，验证：

- JSON 可解析；
- sequence 连续；
- 时间戳单调；
- 频率稳定；
- 包大小合理；
- 没有 NaN/Infinity；
- 停止 SteamVR 后接收端能看到无效/停止状态。

### 阶段 C：Ubuntu 网络测试

只启动 Ubuntu 的数据接收与日志节点，不启动 IK、MuJoCo 或真机。验证网络连通、延迟、丢包和字段解析。

### 阶段 D：MuJoCo

Ubuntu 将 VIVE 数据连接到 OpenArm IK 和 MuJoCo。完成：

- 初始姿态标定；
- 轴方向确认；
- 平移比例从 0.5 开始；
- 左右臂对应关系；
- 旋转方向；
- 扳机与夹爪；
- deadman 松开立即暂停；
- 追踪丢失/断网立即暂停。

### 阶段 E：真实机械臂

不属于 Windows 智能体的执行范围。只有 MuJoCo 全部测试通过后，Ubuntu 侧才能在人工确认、急停就绪和低速限制下接入真机。

## 10. 交付物

Windows 智能体应交付：

```text
openarm-vive-bridge/
├── README.md
├── LICENSE 或第三方依赖说明
├── CMakeLists.txt / 项目文件 / requirements.txt
├── config.example.json
├── actions/                 # OpenVR Action Manifest 与 bindings
├── src/
├── tests/
├── tools/udp_receiver.*     # 本地协议检查工具
└── docs/
    └── WINDOWS_SETUP.md
```

并提供：

- 可重复构建命令；
- 运行命令；
- SteamVR binding 设置说明；
- 一份脱敏的 JSONL 样例；
- dry-run 截图或日志；
- UDP 回环测试结果；
- 已知限制；
- 所有实际版本号与提交号。

## 11. Windows 智能体执行原则

- 先检查、再提问、再安装；
- 安装 VIVE/SteamVR 或修改防火墙前先征得用户同意；
- 不关闭 Windows 安全功能来规避问题；
- 不保存用户密码或 Steam 凭证；
- 不接触真实机械臂控制；
- 每完成一个阶段先报告证据，再进入下一阶段；
- 遇到型号、按钮映射或追踪空间不确定时，以实际设备和运行时枚举结果为准，不凭经验硬编码。

