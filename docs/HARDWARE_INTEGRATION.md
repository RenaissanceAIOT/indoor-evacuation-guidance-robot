# 实验室全向移动平台接入

## 1. 系统连接

```text
12.6 V Battery
├── Motor power → STM32 motor-control board → 4 encoder motors
├── 5 V / 5 A → Raspberry Pi 5
└── Isolated 5 V / 5 A → servos / robotic arm

Raspberry Pi 5
├── USB/TTL serial ↔ STM32 control board
├── USB/Ethernet ↔ 2D LiDAR
└── USB3 ↔ RGB-D camera
```

推荐启动顺序：

1. 确认机器人架空或处于受控区域，物理停止开关可触达；
2. 检查电池、主控和舵机电源是否共地且极性正确；
3. 接入 USB 或 TTL 的 TX/RX/GND；
4. 启动实时控制板并确认麦克纳姆运动模式；
5. 启动 ROS 2 主控；
6. 查看 `/diagnostics`、`/battery_state` 和 `/system/ready`；
7. 依次测试正 `vx`、正 `vy`、正 `wz`，最后落地低速验证。

机器人运行时不连接普通充电器。充电、调试和运动测试应分开进行。

## 2. 串口参数

| 参数 | 值 |
| --- | --- |
| USB 设备属性 | `1a86:55d4` |
| 设备软链接 | `/dev/escape_base` |
| 波特率 | 230400 bps |
| 字节序 | big endian |
| 帧头 | `AA 55` |
| 校验 | 前序字节累加和低 8 位 |
| 遥测频率 | 50 Hz |

udev 规则位于 `hardware/udev/99-lab-mobile-base.rules`。

## 3. 功能码

| 功能码 | 方向 | 数据 |
| --- | --- | --- |
| `0x10` | STM32 → ROS | IMU、vx/vy/wz、电池电压 |
| `0x50` | ROS → STM32 | vx/vy/wz，各乘 1000 |
| `0x51` | ROS → STM32 | IMU 零偏校准 |
| `0x52` | ROS → STM32 | 灯光模式、时间与 RGB |
| `0x53` | ROS → STM32 | 保存灯光配置 |
| `0x54` | ROS → STM32 | 蜂鸣器开关 |
| `0x5A` | ROS → STM32 | 运动底盘类型 |
| `0x60` | ROS → STM32 | 六个舵机角度，度数乘 10 |
| `0x6F` | ROS → STM32 | 舵机零位校准 |

### 0x10 遥测负载

```text
int16 ax, ay, az
int16 gx, gy, gz
int16 vx, vy, wz
int16 battery_voltage
```

换算：

- `acc_mps2 = raw / 32768 × 2 × 9.80665`
- `gyro_rad_s = raw / 32768 × 500 × π / 180`
- `velocity = raw / 1000`
- `voltage = raw / 100`

### 机械臂控制帧

六个 int16 角度与协议固定字段合计 17 字节。由于实验平台固件可能存在版本差异，首次连接机械臂前必须：

1. 拆除负载并把机械臂置于安全支撑状态；
2. 抓取已知有效控制命令；
3. 对比长度字节、总字节数、角度比例和校验；
4. 以 ±5° 低速逐关节验证；
5. 将固件版本与抓包结论写入测试记录。

## 4. 电源阈值

| 电压 | 运行策略 |
| --- | --- |
| `≥ 10.12 V` | 当前软件允许任务；不代表续航充足 |
| `9.84–10.12 V` | 软件禁止运动并取消任务，不自动返航 |
| `< 9.84 V` | 禁止运动、电池健康告警，由人工处理 |

当前 `BatteryState.percentage` 采用线性估计，只用于实验提示，不能代替基于负载、温度和电池曲线的 SOC 模型。

## 5. 坐标与机械标定

- `+x`：机器人前方；
- `+y`：机器人左侧；
- `+z`：机器人上方；
- `+yaw`：俯视逆时针；
- 相机 optical frame：`+z` 前、`+x` 右、`+y` 下。

标定项目：

- 实测底盘长宽高、轮距、轴距和最低离地间隙；
- 测量雷达、相机、IMU、机械臂安装外参；
- 直行 2 m、横移 1 m、旋转 360°，估计里程计误差；
- 静置采集 IMU，更新偏置和协方差；
- 用软障碍确认 Nav2 footprint 与实际外廓；
- 测试速度超时、拔掉串口、雷达中断和 Nav2 退出时的停车行为。
