# 工程验证记录

记录日期：2026-10-02。版本：公开工程整理版。

## 本地实际执行

环境：macOS、Python 3.12；没有 ROS 2 运行环境。

| 检查 | 实际结果 | 范围 |
| --- | --- | --- |
| 协议单测 | 8 / 8 通过 | 字节序、限幅、非有限值、解析与 SI 换算 |
| 路由与目标生命周期单测 | 12 / 12 通过 | 风险路由、封锁、无效代价拒绝、确定性选择、单目标所有权与晚回调 |
| 深度反投影单测 | 5 / 5 通过 | optical 轴向、米制坐标、无效深度/内参拒绝；不测试 YOLO 精度 |
| 合成场景回放 | 3 / 3 符合预期 | east → west → BLOCKED；不是实机数据 |
| Python 语法 | 25 个 src Python 文件通过 | 不执行 ROS 节点 |
| 配置与描述 | 7 个 YAML、5 个 package.xml、1 个 Xacro 通过 | 语法有效，不保证参数最优 |
| 目录与 Markdown 本地链接 | 5 个软件包、25 个链接通过 | 根目录结构、隐藏配置、README 与 docs 内部导航 |

复现：

```bash
python3 -m pip install -r requirements-dev.txt
make test
python3 scripts/replay_scenario.py
python3 scripts/check_repository.py
make lint
```

## 云端与现场

2026-10-02 已实际完成 [CI run 36982584269](https://github.com/RenaissanceAIOT/indoor-evacuation-guidance-robot/actions/runs/36982584269)，源码提交 [`ccb803d`](https://github.com/RenaissanceAIOT/indoor-evacuation-guidance-robot/commit/ccb803d085bbf939fdd29bdd0951eeea9cc0f4fe)。`pure-python` 与 `ros2-build` 两个任务均为 success。本节是运行后补充的记录，不是配置目标。

ROS 环境：Ubuntu 24.04 GitHub runner、`ros:jazzy-ros-base` 容器、Python 3.12.3；属于 x86_64 云端验证，不是 Raspberry Pi 5 实测。

| 检查 | 云端实际结果 | 范围 |
| --- | --- | --- |
| rosdep 与 colcon build | 5 个软件包全部构建成功 | 包依赖、安装与构建配置 |
| colcon test / test-result | 25 tests，0 errors，0 failures，0 skipped | 协议 8、任务/路由 12、深度几何 5 |
| Xacro 展开 | 通过 | 参数替换和 URDF 生成，不证明外廓标定正确 |
| simulation launch 参数加载 | 通过 | 启动文件导入和参数定义，不是完整导航运行 |
| ROS mock 消息运行 | 通过 | 运动与位姿积分、命令看门狗、停止/恢复、联锁心跳过期、非有限值拒绝 |

mock 检查使用独立命名空间 `/portfolio_mock_check`，不打开串口，不加载地图或视觉模型，不验证碰撞。日志中的 `ROS mock smoke passed` 对应真实 ROS 消息调用，不是离线回放。

早期工作流暴露了 Python 包测试发现问题；已补齐 pytest 依赖与 `tests_require`，保留原始失败运行记录，不改写历史。后续修改应以对应提交的任务结果为准，不能把工作流定义本身当作通过证据。

未执行：实机串口抓包、机械臂控制、雷达/相机 launch、Nav2 场景导航、传感器故障停车、视觉模型推理性能和楼宇人员实验。验收方法见 [TEST_PLAN](TEST_PLAN.md)。
