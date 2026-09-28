# P7.0：动态 KF 真值坐标与固定映射审计

## 结论

P6.3 的 3.7 m（+Y）/6.9 m（−Y）“固定 G→P 映射漂移”**主要是评价构造的旋转杠杆项**，不能据此认定 PX4 local origin 在飞行中漂移了数米。旧脚本 `frame_mapping()` 令 `R(t)=R_PX4_att(t) R_Gazebo_body(t)^T`，再令 `t(t)=p_PX4(t)−R(t)p_Gazebo_UAV(t)`。`R(t)` 包含时变 PX4 航向估计与模型真姿态的差；把它用于距 Gazebo 世界原点约 140–250 m 的 UAV 绝对位置，会把百分之一弧度量级的姿态差放大成米级平移。该 `R(t)` 不能直接解释为固定的 Gazebo 世界 NED→PX4 earth-fixed NED 坐标轴变换。

P6.3 两次飞行中，直接比较已在 NED 轴上的同帧 UAV 位置 `p_PX4−p_Gazebo`，首末各 5 s 中位差仅变化约 **(−0.008, −0.228, −0.064) m**（+Y）和 **(−0.009, +0.275, +0.055) m**（−Y）。旧旋转映射平移的 X 变化分别为 −3.735/−6.924 m，其中旋转杠杆项单独贡献约 −3.725/−6.904 m。这解释了旧固定映射失败的主要数值来源；剩余约 0.2–0.3 m 的 UAV 位置差变化仍需作为 PX4 状态估计/采样时间误差计入真值不确定度。

## frame、原点与时间

- **G（Gazebo fixed NED）**：`/world/default/pose/info` 的世界 ENU 位置按 `(north,east,down)=(ENU.y,ENU.x,−ENU.z)` 转换。PX4 `GZBridge::poseInfoCallback()` 发布的 `vehicle_local_position_groundtruth` 也直接使用这个 ENU→NED 变换；它读取同一模型 pose，而非 EKF 估计原点。
- **P（PX4 EKF local NED）**：PX4 `VehicleLocalPosition.msg` 明言这是融合的 earth-fixed NED，局部原点为 EKF2 启动时车辆位置。`EKF2::PublishLocalPosition()` 用 EKF 位置发布 x/y/z；`ref_lat/ref_lon/ref_alt/ref_timestamp` 来自 EKF global origin。其轴定义与 G 同为北、东、下，但原点及估计状态有误差，**不能仅因二者名字都含 NED 就假定数值完全相等**。
- **初始化时刻**：本项目 ULog 的动态 +Y/−Y 会话中 `ref_timestamp` 在 PX4 启动后分别建立为 5.168/5.004 s，之后在所采动态段固定。静态 P6.2 会话为 4.280 s。初始化前 `xy_global=false`、参考经纬度为空；初始化后为 true。这个信息来自 PX4 ULog，不是从 CSV reset counter 猜测。
- **图像时间**：`TargetObservation.measurement_stamp` 取 RGB 图像采样时刻，经 Gazebo `/clock` sim/system anchor 插值到 ROS 时间。Gazebo entity/model pose 的 header sim stamp 经相同 clock 映射后插值查询该时刻。PX4 `VehicleLocalPosition.timestamp_sample`/`VehicleAttitude.timestamp_sample` 经当前 `Px4RosClockMapper(source_is_ros_time=true)` 原样作为 ROS 时间，再在同一 `measurement_stamp` 插值查询。每次查询均有左右 bracket；无 bracket 时返回 `BEFORE/AFTER_HISTORY`，不做外推。CSV 的 query status、左右 source/mapped stamps、interval 和 reset flags 使逐帧对齐可复核。

## 排除或保留的原因

| 候选原因 | 证据与判断 |
|---|---|
| PX4 local origin/reference 在动态段重设 | P6.3 ULog +Y/−Y 采集位置范围内，`ref_timestamp`、`ref_lat/lon/alt`、`xy_global/z_global` 均恒定；`xy_reset_counter=5`、`z_reset_counter=2`、`heading_reset_counter=1`，CSV `quat_reset_counter=1` 也恒定。**不支持动态段发生 origin/reference/reset 跳变**。启动阶段计数变化存在，但不在验收段。 |
| Gazebo/PX4 时间未对齐 | 仍可能贡献分米级动态位置残差；两源均在同一 ROS 图像时刻插值，P6.3 model/PX4 position bracket 通常约 20–24 ms，不能证明绝对物理时延为零。它不能单独解释旧映射 X 的 3.7–6.9 m，因为同帧 UAV 位置本身仅有分米级差，且按 UAV 约 4 m/s，要产生数米需近秒级错位。 |
| PX4 状态估计动态差 | `p_PX4−p_Gazebo` 首末变约 0.23–0.28 m，确有剩余；不把它武断归于固定延迟、EKF 原点或代码错误。 |
| 坐标轴/评价定义 | 旧 `frame_mapping()` 把模型/PX4 **姿态差**作为世界坐标变换并作用在绝对位置上。将旧平移减去无需该旋转的 `p_PX4−p_Gazebo` 后，旋转杠杆项几乎解释全部米级 X 漂移。**这是主要且可复现的归因**。 |

因此旧 E4 作为**单帧的相对几何对照**仍有用，但其逐帧 `R,t` 不是可冻结的长期全局标定。历史 P6.3 报告关于 E1/E4 的获取率与同帧几何结果仍成立；“固定 G→P 不稳定”应具体限定为旧姿态残差构造的固定映射失败。

## 可用于长期 KF 评价的方案与门槛

优先使用同一时刻的相对几何，在已共同定义为 earth-fixed NED 的轴上建立操作性目标真值：

`q_P,relative(t) = p_PX4,UAV(t) + [q_G,target(t) − p_G,UAV(t)]`

其中球心到目标参考点的 +NED Z 偏移按当前前端定义施加一次。这个式子用的是 **ENU→NED 后的相对位置**，不将 PX4/Gazebo 机体航向残差旋转到几百米的绝对位置上；PX4 UAV 位姿只用于同帧锚定与姿态诊断，不能用 PX4 姿态误差去“修正”真实的 earth-fixed 相对方向。它与每帧按姿态差拟合全局 `R,t` 的旧 E4 不同。这个参考仍携带 PX4 自身位置估计误差，评价的是当前在线前端/KF 在 PX4 local NED 的**操作性误差**，不是独立的地理真值。

另一种独立物理对照是用 Gazebo target fixed NED 和经过验证的固定原点平移；P6.3 直接轴对齐的早段平移固定后，raw RMSE 为 0.142/0.194 m、P95 为 0.221/0.270 m，首末误差均值变化约 0.25/0.31 m，**没有旧映射的米级漂移**，但分米级状态估计/时间误差仍明显，不宜用它比较只有 1–3 cm 差的 raw 与 KF。

下一步记录器必须保存原始 G/P 位姿、左右 bracket 时间、最大对齐边距、`ref_timestamp`/reset epoch 和两种速度口径；遇时间回退、reset、缺 bracket、超过时间上限则断段并标 invalid。先复用 P6.2 静态与 P6.3 ±Y 数据检验连续性/速度；未通过时**不能**给 KF 绝对位置、速度 RMSE，更不能调 Q/R。

## 主要代码和日志依据

- `scripts/vision_p6_target_frame_analysis.py`：`frame_mapping()`、`frame_error_vectors()`、`_detail()`。
- `scripts/vision_static_capture.py`：`clock_callback()`、`pose_callback()`、`add_px4_sample()`、`drain()` 与逐帧 query metadata。
- `src/uav_control/uav_control/perception/rgbd_target_localizer.py`：`Px4RosClockMapper` 直接 ROS 时间分支、Gazebo 时钟插值。
- `/home/qin/Projects/PX4-Autopilot/msg/versioned/VehicleLocalPosition.msg`；PX4 `src/modules/ekf2/EKF2.cpp::PublishLocalPosition()`；`src/modules/simulation/gz_bridge/GZBridge.cpp::poseInfoCallback()`。
- `data/experiments/current/p6_20260928/target_acquisition_dynamic_{plus_y,minus_y}/` 的 CSV、`frame_analysis.json`；PX4 ULog `/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-28/08_50_37.ulg` 与 `08_54_07.ulg`。ULog 采集段通过单调 UAV Y 位置范围筛出，并交叉核对 CSV reset counters；原始日志未改。

## P7.2 复算后的限定（2026-09-28）

连续相对几何记录器的静态 raw RMSE 为 0.242 m，而旧逐帧 E4 为 0.046 m；差值逐帧严格等于旧映射的姿态旋转项，静态旋转项 RMSE 0.248 m。因此上文提出的相对几何式只是尚待独立校准的操作性 P-frame 参考，不能因其连续就宣称已解决 KF 绝对真值问题，也不能把 E4≈相机误差误读为 PX4 姿态误差不存在。完整数字、失败门槛和下一步见 `docs/tracking/kf_dynamic_baseline_p7.md`。
