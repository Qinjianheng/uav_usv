# 项目架构

## 数据链路

```text
相机/深度/激光
      -> perception/TargetObservation
      -> tracking/TargetState
      -> guidance/TrajectoryPoint[]
      -> controllers/速度或加速度指令
      -> PX4 Offboard
```

`/ground_truth/*` 只用于仿真评估。正式闭环控制应使用 `/tracking/target_state`，避免把仿真真值当作感知结果。

## 模块职责

- `perception`：检测、深度、激光测距、坐标变换与观测协方差。
- `tracking`：时间同步、EKF/IMM、目标速度估计和状态有效性。
- `guidance`：截击时间、目标轨迹预测、PN/APN及路径规划。
- `controllers`：PID、MPC和后续安全约束下的强化学习控制器。
- `mission`：INIT、TAKEOFF、FOLLOW、INTERCEPT、COMPLETE状态机。
- `common`：坐标、限幅、时间戳和通用数据结构。

所有控制器最终应实现统一接口：输入无人机状态、目标状态、参考轨迹和时间步，输出控制指令与诊断信息。

## 坐标约定

PX4本地坐标采用NED：x向北、y向东、z向下。相机输出必须先通过TF转换，禁止直接把相机光学坐标作为PX4设定值。

