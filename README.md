# UAV-USV interception workspace

这是无人机—无人艇感知、跟踪、预测、规划与控制实验的 ROS 2 Humble 工作空间。

## 快速使用

先启动 QGroundControl，然后执行：

```bash
cd /home/qin/data/uav_usv
./scripts/uav_lab.sh
```

脚本会编译工作空间并依次启动 PX4 SITL、Micro XRCE-DDS Agent、移动目标和当前基准控制器。命令台中：

- `X`：起飞并进入跟随模式；
- `Y`：进入截击模式；
- `Q`：退出命令台，已经打开的实验终端保持运行。

只编译工作空间：

```bash
./scripts/build_workspace.sh
```

## 目录

```text
uav_usv/
├── src/
│   ├── px4_msgs/             PX4 ROS 2 消息
│   ├── uav_control/          当前可运行节点与后续算法模块
│   ├── uav_usv_interfaces/   项目统一消息接口
│   └── uav_usv_bringup/      launch与实验参数
├── data/
│   ├── experiments/          CSV实验结果
│   ├── bags/                 rosbag数据
│   ├── datasets/             感知数据集
│   └── models/               检测或强化学习模型
├── docs/                     架构与迁移说明
└── scripts/                  编译和一键启动脚本
```

当前稳定基准保留为 `uav_control/trajectory_impact_sim.py`。后续算法放入 `perception`、`tracking`、`guidance`、`controllers` 和 `mission` 子目录，不再继续扩大单一节点。

