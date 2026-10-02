# 技术基础与参数约定

## 实验室研究平台参数

本仓库使用实验室现有移动机器人平台的技术参数作为工程输入，不分发原始设备资料、图片或底层固件。代码只实现完成系统集成所需的接口与换算。

| 参数类别 | 工程基线 |
| --- | --- |
| 实时处理器 | STM32F407VET6，168 MHz，512 KB Flash，192 KB SRAM |
| 运动形式 | 四轮麦克纳姆全向移动 |
| 电机 | 12 V 编码器减速电机，1:30，额定 290±20 rpm，额定扭矩 1.5 kg·cm |
| 惯性测量 | 六轴 IMU，串口数据量程 ±2 g、±500 °/s |
| 电池 | 12.6 V / 10 Ah，持续放电 10 A、峰值 20 A |
| 上位机电源 | 5 V @ 5 A |
| 执行机构电源 | 独立 5 V @ 5 A |
| 串口 | USB/TTL，230400 bps，大端 int16 |
| CAN 扩展 | 1 Mbps |
| 遥测频率 | 50 Hz |
| 遥测内容 | 三轴加速度、三轴角速度、vx/vy/wz、电池电压 |

以上参数用于软件接口、供电预算和安全阈值设计，不代表对任意同类设备都适用。实机部署必须记录控制板固件版本、设备序列、测量日期和标定结果。

## 二进制串口协议

```text
AA 55 | LENGTH | CODE | PAYLOAD... | CHECKSUM
```

- `LENGTH`：完整帧字节数；
- `CODE`：数据或控制功能；
- `PAYLOAD`：按功能码解释，多字节数值高位在前；
- `CHECKSUM`：之前所有字节累加和的低 8 位。

参考数据帧：

```text
AA 55 0B 01 03 E8 FC 18 00 0A 14
```

它承载 `1000, -1000, 10` 三个 int16，用于单元测试字节序与校验算法。

## 软件技术基础

- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Jazzy documentation](https://docs.nav2.org/jazzy/)
- [Nav2 first-time robot setup guide](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://docs.ros.org/en/ros2_packages/rolling/api/robot_localization/)
- [Ubuntu on Raspberry Pi](https://ubuntu.com/download/raspberry-pi)

## 结果溯源约定

- 硬件参数变更必须同步修改配置、接线记录和测试报告；
- 速度、协方差、footprint 与膨胀半径属于现场标定量；
- 性能数字必须对应测试日期、样本数、commit 和原始日志；
- 模型权重必须记录版本、许可证和 SHA-256；
- 地图、相机画面和实验参与者数据默认不进入公开仓库。
