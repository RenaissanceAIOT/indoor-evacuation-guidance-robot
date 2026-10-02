# 工程验证记录

记录日期：2026-10-02。版本：公开工程整理版。

## 本地实际执行

环境：macOS、Python 3.12；没有 ROS 2 运行环境。

| 检查 | 实际结果 | 范围 |
| --- | --- | --- |
| 协议单测 | 8 / 8 通过 | 字节序、限幅、非有限值、解析与 SI 换算 |
| 路由与目标生命周期单测 | 8 / 8 通过 | 风险路由、封锁、单目标所有权与晚回调 |
| 合成场景回放 | 3 / 3 符合预期 | east → west → BLOCKED；不是实机数据 |
| Python 语法 | 23 个 src Python 文件通过 | 不执行 ROS 节点 |
| 配置与描述 | 7 个 YAML、5 个 package.xml、1 个 Xacro 通过 | 语法有效，不保证参数最优 |
| Markdown 本地链接 | 无缺失目标 | README 与 docs 内部导航 |

复现：

```bash
python3 -m pip install -r requirements-dev.txt
make test
python3 scripts/replay_scenario.py
python3 scripts/check_repository.py
make lint
```

## 云端与现场

[GitHub Actions](https://github.com/RenaissanceAIOT/indoor-evacuation-guidance-robot/actions) 执行 pure-python 与 ros2-build 两个任务。ROS 构建任务在官方 Jazzy 基础容器安装 rosdep 依赖并执行 colcon build/test；以对应提交的任务结果为准，不把工作流定义本身当作通过证据。

未执行：实机串口抓包、机械臂控制、雷达/相机 launch、Nav2 场景导航、传感器故障停车、视觉模型推理性能和楼宇人员实验。验收方法见 [TEST_PLAN](TEST_PLAN.md)。
