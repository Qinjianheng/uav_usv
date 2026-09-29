# P7.2b：PX4 Local NED 轴向与原点关系独立验证

日期：2026-09-29；仓库基线：`b61db500136450cfacff0cdced8ea35c5116992f`。本轮结论为 **A：在本实验覆盖的约 8 m 北/东双轴位移、约 1.2 m/s 运动和偏离原点约 8 m 处的 −45°～+96° yaw 范围内，PX4 local NED 与 Gazebo fixed NED 的 XY 轴共轴，继续使用 `R=I` 的连续操作性目标真值**。没有找到跨四个运动段都成立的固定 yaw 旋转，也没有观察到飞行中局部原点重置。此结论不声称 PX4 EKF 位置无误差，更不将 PX4 groundtruth 当作独立物理测量。

## 方法与数据质量

新建隔离的 P7 启动文件，仅运行只读采集器和唯一的安全门控 PX4 Offboard 指令节点；不加载任务控制器、视觉节点或 KF。仅在 PX4 `pre_flight_checks_pass=true`、`failsafe=false`、已进入 OFFBOARD、未解锁且预流满 2 s 后接受控制台 `X`。完成稳定悬停 25 s、+N/−N、+E/−E、近原点原地 yaw 后降落；补充实验在 +N 约 8 m 处做相同 yaw 序列。未发送 `Y`，未追 USV。运动前检查无用户已有 PX4/Gazebo 会话，结束只停止本轮创建的 PID/PGID；收尾检查无残留仿真进程。

采集器以 10 Hz、统一 ROS 时间 `now−60 ms` 查询 Gazebo UAV 模型、PX4 `VehicleLocalPosition` 和姿态；逐行保存 ROS/仿真戳、原始和映射 bracket、位置/速度/四元数、参考经纬高与 `ref_timestamp`、各 reset 计数、epoch、状态和拒绝原因。各输入须双侧 bracket、每侧 ≤50 ms；不外推，跨 reset 或时间回退拒绝。全流程原始数据位于 [`data/experiments/current/p7_axis_20260929/`](../../data/experiments/current/p7_axis_20260929/)：完整双轴场 `20260929_151016_362549/`（2984 行，2983 有效；飞行运动段 1003/1003 有效），偏原点 yaw 场 `20260929_152238_306981/`（1165 行，1161 有效；飞行段 652/652 有效）。两场飞行均 `failsafe=false`，参考时间与 reset 计数在飞行窗口内恒定。主场采集 bracket 最大单侧间隔 P50/P95/max 为 21.47/22.83/23.00 ms；偏原点场为 17.02/19.25/19.62 ms。

ROS `/fmu/out/vehicle_local_position_groundtruth` 在本机无发布者，故使用两场 PX4 ULog：`/data/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-29/07_10_09.ulg` 与 `07_22_33.ulg`。ROS↔ULog 时钟偏移仅用 **PX4 EKF 采集位置对同一 EKF 的 ULog 位置**拟合；主场仅用 NORTH_OUT/EAST_OUT 前半段，补充场仅用 NORTH_OUT/NORTH_RETURN 前半段。Gazebo 位置和 groundtruth 均未参与时钟拟合。之后对 ULog EKF/groundtruth 作双侧 ≤50 ms 插值，分别保留左右原始时间戳。主场 2717 个严格配对，时钟自检训练 RMSE 0.0098 mm、全部样本 0.0071 mm；groundtruth ULog 侧间隔 P95 18.76 ms。偏原点场 1161 个配对，自检训练 RMSE 0.0180 mm、groundtruth 侧间隔 P95 19.05 ms。采集 bracket 全部落在 10–25 ms 组，<10 与 25–50 ms 两组无样本，因此不能由分组比较推断时间间隔对 EKF 残差的因果贡献。

## 轴向模型：位移训练、独立留出

只用 UAV **位置位移**拟合世界轴：`Δp_P≈R(θ)Δp_G`。NORTH_OUT 和 EAST_OUT 各自前半段训练固定二维旋转；各自后半段与两个返回段留出。`s=1`，不拟合 affine。逐帧机体姿态差只列为反例模型，不用于定义世界轴。原始六段位移和精确结果在主场 [`axis_fit.json`](../../data/experiments/current/p7_axis_20260929/20260929_151016_362549/axis_fit.json) 与 [`axis_holdout.json`](../../data/experiments/current/p7_axis_20260929/20260929_151016_362549/axis_holdout.json)。

| 模型 | 训练段位移 RMSE | 留出段位移 RMSE | 全六段同口径 RMSE |
|---|---:|---:|---:|
| M0：严格共轴 `R=I` | 0.119 m | **0.147 m** | **0.139 m** |
| M1：固定旋转 `θ=+0.00707 rad=+0.405°` | 0.114 m | 0.164 m | 0.149 m |
| M2：分段机体 yaw 残差作为旋转 | — | — | 0.162 m |

+N 前半段 Gazebo/PX4 位移分别为 (4.768, 0.013)/(4.846, 0.011) m；+E 前半段为 (0.156, 4.691)/(0.089, 4.824) m。不共线轴明确对应 North→North、East→East。六段方向差在 −1.26°～+0.85°；+N/+E 前半段尺度比分别 1.0165/1.0278，而后半段分别 0.9537/0.9395，返回段 0.9906/0.9923。它们随运动阶段改变，不能作为固定世界轴旋转或固定尺度标定。M1 只略降训练误差，却增大留出误差；不采用 `+0.405°` 作为补偿。M2 同样不支持把 `R_PX4,body R_Gazebo,bodyᵀ` 当世界 frame 变换。

近原点悬停估计的平移中位数约 (−0.0060, −0.0009) m，只是该会话 EKF 的位置偏移描述，**没有写入真值公式**。偏离原点的 yaw 补充场更能区分随 yaw 转动的世界轴：352 帧中 Gazebo 模型 yaw −0.794～+1.686 rad，模型 XY 中位 (7.977, −0.017) m，物理模型位置跨度 (0.103, 0.095) m，PX4 local 位置跨度 (0.076, 0.086) m，PX4 EKF−groundtruth 偏移跨度仅 (0.068, 0.050) m。若约 8 m 的世界位置随近 90° 机体 yaw 旋转，位置应有米级摆动；实际仅厘米级。模型并非绝对静止，因而这些厘米波动不可全归结为 yaw 耦合。详见补充场 [`yaw_offset_analysis.json`](../../data/experiments/current/p7_axis_20260929/20260929_152238_306981/yaw_offset_analysis.json)。

## 桥接、EKF、时间与原点分层

PX4 `GZBridge::poseInfoCallback()` 的 `vehicle_local_position_groundtruth` 直接取同一 `/world/default/pose/info` 模型位置并按 ENU `(x,y,z)`→NED `(y,x,−z)` 转换。因此 ULog groundtruth 是**桥接/时间语义对照**，不是第二个独立传感器；独立双轴位移和偏原点 yaw 才是轴向验证的关键。主场 2717 帧 Gazebo→PX4 groundtruth 三维 RMSE **0.000190 m**、P95 0.000557 m；PX4 EKF→groundtruth 三维 RMSE **0.0798 m**、P95 0.151 m、最大 0.221 m、XYZ 有符号均值 (−0.0099, −0.0037, −0.0560) m。偏原点场的对应 RMSE 为 **0.000202 m** 与 **0.0728 m**。详见两场的 [`px4_groundtruth_comparison_final.json`](../../data/experiments/current/p7_axis_20260929/20260929_151016_362549/px4_groundtruth_comparison_final.json) 和 [`px4_groundtruth_comparison.json`](../../data/experiments/current/p7_axis_20260929/20260929_152238_306981/px4_groundtruth_comparison.json)。

主场稳定悬停 EKF−groundtruth 的 XYZ 中位约 (−0.0058, −0.0008, −0.0416) m；相对悬停偏移投影到运动方向，N 出/返、E 出/返段分别约 +0.101/+0.141/+0.125/+0.140 m。方向相关且起返有变化，属于 **PX4 EKF 状态/时间链剩余差异**，不能凭本实验再断言固定时间延迟。完整分段误差见 [`residual_decomposition.json`](../../data/experiments/current/p7_axis_20260929/20260929_151016_362549/residual_decomposition.json)。主场 `ref_timestamp=5604000`、`xy/z/heading` reset 计数 `4/2/1` 恒定；补充场分别 `4872000`、`5/2/1` 恒定。两场跨会话的参考值不同属于重新启动，不是飞行中跳变。每场姿态 `quat_reset_counter` 也在飞行窗口内恒定。本轮未测得可解释为“PX4 local origin 飞行中跳变”的事件。

## 静态与 ±Y 旧目标数据复核

在本次独立轴向验证通过后，按现有 `scripts/kf_truth_capture.py` 的 `R=I`、同图像时刻相对几何公式重新汇总 P7 已保存的 P6 历史静态和 ±Y 数据，见 [`revalidated_target_truth.json`](../../data/experiments/current/p7_axis_20260929/revalidated_target_truth.json)。这些是**旧 RGB-D 观测的离线复核**，本轮没有重跑追踪，也没有形成 KF 当前版的动态基线。

| 场景 | 有效帧 | raw 位置 RMSE / P50 / P95 / max (m) | XYZ 有符号均值 (m) | 末 5 s−首 5 s XYZ (m) |
|---|---:|---:|---:|---:|
| 静态 | 239 | 0.242 / 0.243 / 0.253 / 0.257 | (+0.023, +0.239, +0.032) | (−0.002, +0.006, −0.001) |
| +Y，命令 4 m/s | 231 | 0.092 / 0.091 / 0.109 / 0.118 | (+0.058, +0.065, −0.010) | (+0.046, −0.028, +0.005) |
| −Y，命令 −4 m/s | 222 | 0.226 / 0.215 / 0.300 / 0.311 | (+0.144, −0.160, −0.035) | (+0.041, +0.022, −0.001) |

三场均无米级首末段累计漂移；目标方向和差分速度与命令 0/±4 m/s 一致。静态 0.242 m 不需等于旧 E4 的 0.046 m：旧 E4 用逐帧姿态残差旋转相对目标向量，已知静态约 5 m 杠杆和 −0.047 rad 残差可产生约 0.24 m 横向项。新定义保留此项，是连续的 **PX4 local NED 操作性评价参考**，并非独立地理绝对真值；−Y 的 0.226 m 也保留为真实待分析残差。

## 代码、运行过程和边界

本轮新增：`scripts/p7_px4_local_axis_capture.py`（只读同刻查询、时间与 reset 拒绝）；`scripts/p7_px4_local_axis_motion.py`（独立安全门控 N/E/yaw 轨迹）；`scripts/p7_px4_local_axis_analysis.py`（双轴位移拟合与留出）；`scripts/p7_px4_groundtruth_analysis.py`（ULog 自时钟匹配及双侧插值）；`src/uav_usv_bringup/launch/p7_axis_experiment.launch.py`（隔离启动）；`src/uav_control/test/test_p7_px4_local_axis.py`、`test_p7_axis_motion.py`（纯计算和安全状态测试）；本报告及上述数据。没有修改在线视觉、KF、预测、控制器、任务状态机、`scripts/kf_truth_capture.py` 或配置。

先导启动时 PX4 status topic 版本误写为 `_v1`，预飞门保持关闭，**未解锁**；已按运行时实际 `_v4` 修正。首个完整双轴场结束时持续发送旧悬停 setpoint 与 AUTO_LAND 冲突，停止本轮自有指令节点后 PX4 自行落地、landed=true 且解除武装；现改为 LANDING/DONE 不再发送悬停参考，偏原点补充场已实测正常降落解除武装。补充场 DONE 阶段采集器从 ROS timer 内直接 `rclpy.shutdown()` 阻塞，落地后仅对本轮自有采集节点发送 SIGINT 保存数据；现改为 `capture_complete` 标志加外层 `spin_once()` 退出，**此最后一项改动仅经过静态/单元验证，尚未再次仿真实测自动退出**。先导失败采集保留在 `20260929_150559_207684/`，不进入最终指标。历史日志未覆盖或回滚。

验证记录：新增定向测试 `python3 -m pytest -q src/uav_control/test/test_p7_px4_local_axis.py src/uav_control/test/test_p7_axis_motion.py` 为 **10 passed**；7 个新增 Python 文件 `py_compile` 通过；脚本、launch、测试 `python3 -m flake8` 通过；`colcon build --packages-select uav_usv_bringup` 通过。首次用独立 `pytest` 命令时当前 shell 无可执行文件，改用同一 Python 的 `-m pytest` 后通过。最终 `git diff --check` 通过；由于新增文件未追踪，另对 8 个新增代码/报告文件逐行检查尾随空白与末尾换行，全部通过。当前仅认可结论 A 的实验范围内共轴和 `R=I` 操作性参考；P7.3 可开始 **Current-B + Current-KF** 同时刻动态基线，但须分别报告视觉前端、PX4 EKF 位置和 KF 的贡献，先建立基线，不调 Q/R。尚未独立拆出 EKF 状态估计与时延的因果份额，也未校准任意更大范围世界坐标系。视觉和 KF 始终 shadow-only，规划控制仍用 `simulation_truth`；未切换闭环，未 commit/push。
