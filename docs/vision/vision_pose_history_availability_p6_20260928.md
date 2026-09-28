# P6.2：安全起飞与 FOLLOW pose-history 可用性（2026-09-28）

## 直接结论

1. **上次不能解锁的已证实直接条件**：PX4 当时 `pre_flight_checks_pass=false`，`arming_state=1`，ARM ACK `result=1`。具体哪项 PX4 预检未通过没有留存 HealthReport/控制台证据，**不能确认**。项目自身的 `flight_ready` 原先只检查 OFFBOARD、未解锁和状态新鲜，确实可能先于 PX4 预检通过而允许 X。默认模块化启动由 `trajectory_tracker_node` 发布此门控，旧版 `trajectory_impact_sim` 也有同一问题；两处均已改为要求 PX4 `pre_flight_checks_pass=true`。
2. 本次只启动一次正常 SITL。X 前现场采样：PX4 `pre_flight_checks_pass=true`、`arming_state=1`、`nav_state=14`（OFFBOARD）、`failsafe=false`，项目 `flight_ready=true`；位置 `xy_valid/z_valid/v_xy_valid/v_z_valid` 均 true，姿态持续更新。实际话题列表没有 HealthReport。未获取从 PX4 启动到预检通过的连续时间记录，因此“上次只是预检尚未完成”仍是合理解释，**不是已证明的具体根因**。QGroundControl 由现有启动脚本自动启动，本轮判断没有依赖它。
3. 仅在上述两个许可同时成立后发送 X。PX4 随后 `arming_state=2`、`nav_state=14`、`failsafe=false`；`takeoff_complete=true`，任务进入 FOLLOW，位置约 `(3.009, 0.017, -5.012)` m（PX4 local NED）。未发送 Y。本次会话的安全起飞通过。
4. 稳定 FOLLOW 约 30 s 的 243 条 observation **243 条有效（100%）**，`POSITION_TIMESTAMP_AFTER_HISTORY` **0/243（0%）**。旧地面 hold 为 168/262 有效、63/262 同类拒绝（24.0%）。本次结果支持该现象与地面/未解锁条件相关，尚不能在状态更新、ROS 回调、时钟映射之间确定地面时的单一机制。正常 FOLLOW 未达到继续修改在线时间链的阈值，故保留 `pose_wait_timeout=0.15 s`、`maximum_pose_wait_gap=0.15 s`、history/QoS/插值/配对逻辑。
5. 新 FOLLOW 242 帧具有完整物理参考：相机系 E1 RMSE **0.0451 m**，同帧 PX4 NED E4 RMSE **0.0460 m**。跨框直接相减的 E3 为 **0.2316 m**，仅作诊断，不当作在线目标获取误差。E4 使用同帧 UAV 两套位姿建立逐帧映射；E4≈E1 有几何恒等成分，不证明固定 G→P 标定。

## 时间统计与判读

定义 `future_gap = measurement_stamp − history_end_stamp`，负值表示 history 已覆盖图像时刻；`latest_age = measurement_stamp − mapped_stamp`。以下单位均为 ms，均来自有效 observation；AFTER_HISTORY 组没有样本，分位数不适用。

| 指标 | n | P50 | P95 | max |
|---|---:|---:|---:|---:|
| position future gap / latest age | 243 | −13.63 | −7.95 | −7.53 |
| attitude future gap / latest age | 243 | −14.41 | −11.81 | −11.53 |
| 位置相邻 PX4 样本 bracket 间隔 | 242 | 24.00 | 24.00 | 24.13 |
| 姿态相邻 PX4 样本 bracket 间隔 | 242 | 20.00 | 23.81 | 24.13 |
| 在线 position source/mapped 可见步长 | 242 | 104.00 | 200.00 | 304.00 |
| 在线 attitude source/mapped 可见步长 | 242 | 100.00 | 200.12 | 300.00 |

最后两行是 **按图像 observation 抽样的 source/mapped stamp 步长**，受约 8 Hz 观测率影响，不能解释为 PX4 发布周期。相邻样本 bracket 间隔才是本次 CSV 中可直接获得的 PX4 pose 采样间隔。正常有效帧中 position 和 attitude history 均领先图像时刻，未见未来 bracket 缺失。没有发生同类拒绝，因而无法从新 FOLLOW 数据反推出旧地面拒绝时的 `future_gap` 分布；旧 CSV 也没有这 10 个在线 history 字段。

## 代码范围与控制边界

- `trajectory_impact_sim.py`：在构造时初始化预检状态，在 `vehicle_status_callback()` 原样记录 PX4 字段，在 `prepare_flight_on_ground()` 将其纳入 `flight_ready`，并只各记一次等待/通过日志。ARM 指令未改。
- `control/trajectory_tracker_node.py`：实际默认启动路径中作同样的状态记录和门控；纯函数 `ground_flight_ready()` 承载四项布尔条件。任务状态机、跟踪控制与起飞指令未改。
- `scripts/vision_p6_pose_history_analysis.py`：仅离线统计 CSV；不发布 ROS 消息，也不参与控制。
- 对应单元测试覆盖 preflight false→true→false、状态回调，以及时间差正负号和拒绝率。

规划/控制仍取 `simulation_truth`，视觉保持 shadow-only；没有把 Gazebo 真值送入在线视觉或 KF，没有修改在线 pose 时间、RGB-D、球心或滤波参数。

## 本次验证与留存文件

- 定向测试（控制、视觉和新分析）：首次 161 通过、2 失败，失败由旧 `SimpleNamespace` 测试桩缺少新增状态字段导致；修正测试桩后全包 `src/uav_control/test` **457 passed、1 skipped**，另有 2 条已有 `SelectableGroups` 弃用警告。
- `colcon build --packages-select uav_control uav_usv_bringup --symlink-install`：2 包通过。修改文件 `py_compile` 通过；`python3 -m flake8` 首次报新增测试 1 行超长，修正后复验。`git diff --check` 通过。
- 仅运行一次正常 SITL，正常安全解锁、起飞、进入 FOLLOW，采集后对本次新建的 Gazebo/PX4/DDS/ROS 进程组发送 SIGINT；未接管或终止既有用户会话。
- 新数据目录：`data/experiments/current/p6_20260928/pose_history_follow/`。其中 `vision_static_capture_20260928_162802_547880.csv` 是原始逐帧数据，同名前缀 `.json` 是采集元数据，`config.yaml` 是本次配置副本，`preflight_status_observation.json` 留存起飞前后状态，`pose_history_follow_analysis.json` 为时间统计，`pose_history_follow_frame_analysis.json` 为 E1/E4 对照。历史数据未覆盖。

## 尚未解决与下一步

上次会话 `pre_flight_checks_pass=false` 的**具体 PX4 失败项仍未知**；若重现持续 false，应先获取 HealthReport（若该版本发布）或 PX4 commander 只读预检输出，再考虑修复上游。现在 FOLLOW 拒绝率为 0、E1/E4 约 0.05 m，**不需要在线 pose-history 修复**。下一阶段可按既定顺序开展新 B 前端的动态/转弯 KF 基线，而不调整本轮时间门限。
