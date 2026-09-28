# P6 第 1 组：UAV 地面静止、USV 横移

日期：2026-09-28；基线 `40dda14811fb249f7a637360abe238934e000b0c`。仅执行 P6 的第一组单因素实验。视觉继续 shadow-only，控制配置仍为 `target_state_source: simulation_truth`；未发送 `X/Y`，PX4 保持未解锁的 OFFBOARD ground hold。采集结束后仅停止本次启动的 Gazebo、PX4、DDS 和 ROS 进程。

## 配置与原始数据

独立配置 `data/experiments/current/p6_20260928/usv_lateral_uav_ground_hold/config.yaml` 从 P5 `static5_config.yaml` 复制，目标设为线性横移：`initial_x=8 m`、`initial_y=-20 m`、`velocity_y=horizontal_speed=0.8 m/s`、`start_on_command=false`。不改任务状态机或定位器。现有 `vision_p5_roi_capture.py` 保存有限 ROI、原始及映射时间戳、两个时刻的 UAV/USV Gazebo 位姿和在线观测。

首段预检采集 `vision_p5_capture_20260928_104016_260403.csv` 的 203 条观测均无效、无 ROI。原因是首版配置只改了 `velocity_y`，而目标节点线性轨迹的实际巡航速度受 `horizontal_speed=4 m/s` 限制；采集时真值目标 Y 已到 51–151 m，离开视野。该段不用于测量结论。修正独立配置后，重启同一组实验，得到 `vision_p5_capture_20260928_104218_760906.csv`、同名前缀目录中的 `roi.jsonl`/`.npz`、位姿和观测 JSONL，以及 `roi_analysis.json`。旧 P5 数据未覆盖。

## 同帧结果

第二段 28 s 采集 262 条观测，168 条有效；无效原因为 `POSITION_TIMESTAMP_AFTER_HISTORY` 63、`IMAGE_INVALID` 29、其余 2。保存 17 个完整有效 ROI，其中 10 个 raw RGB/Depth 差为 40–60 ms。Gazebo UAV 模型位置在采集内固定；PX4 速度三个轴的 RMSE 分别约 0.007/0.010/0.007 m/s；USV 真值 Y 速度为 0.8 m/s。有效帧内目标 Y 从 −8.91 m 到 +4.73 m。

| 口径 | 结果 |
|---|---:|
| 整段 168 有效帧，相机系 B RMSE / P95 / max | 0.117 / 0.141 / 0.207 m |
| 保存的 17 ROI，相机系 B RMSE / P95 / max | 0.114 / 0.123 / 0.129 m |
| 168 有效帧 raw skew P50 / P95 / max | 52 / 52 / 100 ms |
| 40–60 ms 的 10 ROI：USV 相对相机平移 P50 / P95 / max | 0.040 / 0.048 / 0.048 m |
| 同 10 ROI：理论球心位移 P50 / P95 / max | **0.0056 / 0.0276 / 0.0332 m** |
| 同 10 ROI：实际 B 相机系误差 RMSE | 0.113 m |

这组目标横移条件下，约 52 ms 的采样差确实对应约 4–5 cm 的目标相对平移，但沿固定 RGB 像素射线传播到 B 球心的理论位移较小；已测相机系误差仍约 0.11 m，且主要有约 +0.10 m 的相机 Z 向有符号偏差。离线把真值几何给出的理论 skew 位移从 17 帧 B 误差中扣除，RMSE 为 0.114→0.116 m，未见改善；这是评价用的理想反事实，**不能作为在线校正**。因此本组没有证据表明 50 ms skew 是该低速横移场景的主要误差，也没有依据修改配对门限或同步代码。

## 边界与结论

本组 UAV 是**地面未解锁静止**，并非稳定悬停；USV 速度为 0.8 m/s，低于历史 P5 的 4 m/s 动态场景，且只有 17 个保存 ROI。结论仅适用于本次视角、速度和有效样本。63 条 PX4 位置历史晚于查询时刻的拒绝需要与视觉几何误差分开看；本轮未排查或修改时间映射。历史 4 m/s 场景中 40–60 ms 子集的理论位移 P95 达 0.198/0.235 m，不能用本次低速结果否定其重要性。后续两组实验尚未开始。
