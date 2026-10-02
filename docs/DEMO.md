# 部署与演示脚本

下面是复现实验步骤，不是已录制演示的成绩单。`hardware`、`mapping`、`navigation` 都会启动底盘，切换阶段时先停止上一组 launch，避免串口与 TF 重复。雷达/相机驱动需按实际传感器单独启动。

## 0. 每次演示前

```bash
source /opt/ros/jazzy/setup.bash
source ~/escape_robot_ws/install/setup.bash
ros2 run escape_robot_bringup preflight  # 如未安装脚本，按下列命令人工检查
ls -l /dev/escape_base
ros2 topic hz /scan
ros2 topic hz /camera/color/image_raw
```

确保底盘周围至少保留 1 m 缓冲区，急停或电源开关可立即触达。

## 1. 底盘 Bringup

```bash
ros2 launch escape_robot_bringup hardware.launch.py port:=/dev/escape_base
```

另开终端：

```bash
ros2 topic echo /diagnostics
ros2 topic hz /odom
ros2 topic echo /battery_state --once
```

架空测试三自由度，每个指令只发一次，350 ms 看门狗会停车：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.05}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.15}}"
```

## 2. 建图

先启动具体雷达驱动并确保其发布 `/scan`，随后：

```bash
ros2 launch escape_robot_bringup mapping.launch.py
rviz2 -d $(ros2 pkg prefix escape_robot_description)/share/escape_robot_description/rviz/navigation.rviz
```

低速走完整个区域，保证走廊转角和出入口被多角度观测。保存地图：

```bash
mkdir -p ~/escape_robot_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/escape_robot_ws/maps/site_a
```

## 3. 导航与疏散引导

在 `exits.yaml` 中把出口坐标改为 RViz 地图中的实测位置：

```bash
ros2 launch escape_robot_bringup navigation.launch.py \
  map:=$HOME/escape_robot_ws/maps/site_a.yaml \
  enable_perception:=false
```

在 RViz 中先设置初始位姿，再发送警情：

```bash
ros2 topic pub --once /hazard/active std_msgs/msg/Bool "{data: true}"
ros2 topic echo /guidance/status
```

模拟东出口封锁：

```bash
ros2 topic pub --once /hazard/blocked_exits std_msgs/msg/String "{data: 'east'}"
```

解除警情：

```bash
ros2 topic pub --once /hazard/active std_msgs/msg/Bool "{data: false}"
```

## 4. 视觉演示

视觉依赖建议放在独立虚拟环境或容器，避免改动 ROS 系统 Python。准备轻量模型后：

```bash
ros2 run escape_robot_perception object_depth --ros-args \
  --params-file src/escape_robot_bringup/config/perception.yaml
ros2 topic echo /perception/detections
```

展示时同时录制：

```bash
ros2 bag record -o demo_run \
  /tf /tf_static /scan /odom /odometry/filtered /imu/data_raw \
  /battery_state /cmd_vel /guidance/status /guidance/selected_exit \
  /perception/detections
```

## 5. 推荐视频镜头

1. 机器人硬件与节点图（10 s）；
2. RViz 建图和真实底盘同步移动（15 s）；
3. 行人横穿，机器人减速/绕行（15 s）；
4. RGB-D 检测框、深度和三维坐标（10 s）；
5. 触发警情、出口选择、声光提示和自主引导（20 s）；
6. 封锁当前出口后自动重规划（15 s）；
7. 架空条件下停止速度指令，展示零速与 diagnostics（10 s）；断开串口实验只能在验证 MCU 独立看门狗后进行。
