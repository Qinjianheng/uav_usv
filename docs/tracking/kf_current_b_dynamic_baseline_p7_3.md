# P7.3：Current-B 与 Current-KF 同图像时刻动态基线

日期：2026-09-29；基线 HEAD `954fafc03f7ed77f80421bbdbd8ff57c7ccd5440`。**结论：主要位置误差不在 KF 的 `dt` 或 Q/R，而在相机观测到世界 NED 的姿态变换及动态平台/时间参考。** 静态操作性 raw/KF RMSE 为 0.197/0.197 m；稳态 +Y 为 0.121/0.120 m；−Y 含追赶过渡的 30 s 为 0.160/0.166 m。−Y 后 20 s 收敛到 0.115/0.115 m。+Y 起飞后的短追赶段另有 0.804/0.844 m，不能用随后稳态段掩盖。没有修改在线视觉、KF、预测或控制算法；不调 Q/R。

## 1. 当前链路与评价定义

`rgbd_target_localizer.py` 的 `RgbDepthPairBuffer` 按原始图像戳配对（门限 0.1 s）；球心 Current-B 使用实际 mask+depth、ray-radius 恢复。观测时间取 **RGB 原始采样时间经 Gazebo image-clock 映射后的 ROS 时间**，不是 ROS 收包或发布时刻；深度原始时间单独保留。定位器在该时刻插值 PX4 `VehicleLocalPosition` 与姿态，用 `camera_target_to_local_ned()` 只转换一次并发布 `TargetObservation.stamp`。本轮独立采集的 PX4 位置与在线插值位置差 RMSE 为 0，姿态差 P95 ≤2.2×10⁻⁸ rad；未发现最近值替代插值或重复坐标转换。

`target_kalman_filter.py` 只接受有效 `local_ned` 位置观测：首次初始化，之后按相邻**图像时间** `dt` 先 `predict` 再 `update`，乱序观测拒绝；20 Hz 定时器只把已更新状态投影到当前发布时间，不修改滤波器内部更新时间。`TargetState.stamp` 是发布时间，`source_stamp` 是最后真实图像观测时间。评价器对每个 RGB 图像时刻只取此前已发布且来源不晚于该图像的 KF 状态，按 KF 常速度投影至图像时刻；未使用未来更新。`UPDATED_SINCE_PREVIOUS_PUBLISH` 与 `PREDICT_ONLY_SINCE_PREVIOUS_PUBLISH` 是从相邻发布消息的 `source_stamp` 推断，非 KF 内部拒绝事件计数。

采用 P7.2b 已验证的 `R=I`：**操作性真值** `p_PX4,UAV(t)+[p_G,target_entity(t)−p_G,UAV(t)]+target_reference_z_offset`；**物理真值**为同图像时刻 Gazebo 目标实体中心加该高度偏移。两者差正是同帧 `PX4 UAV−Gazebo UAV`。模型姿态反事实调用定位器原有 `camera_target_to_local_ned()`，保持观测、外参和 PX4 位置一致，只替换为同帧 Gazebo 机体姿态；于是 `raw操作性误差=相机几何项+姿态项`，最大逐帧闭合误差 <3×10⁻¹⁷ m。`raw物理误差=raw操作性误差+同帧PX4−Gazebo模型`，最大闭合误差 <3×10⁻¹⁴ m。各分量**是向量分解，RMSE 不能直接相加**。真值和反事实均只参与离线评价，没有发布给视觉/KF/控制。

## 2. 实验、时间门限与总体误差

使用专用 [`p7_kf_baseline.launch.py`](../../src/uav_usv_bringup/launch/p7_kf_baseline.launch.py) 包含现有安全起飞/FOLLOW 链与只读 [`vision_p6_dynamic_capture.py`](../../scripts/vision_p6_dynamic_capture.py)：视觉与 KF 同一次会话采集，仅发送 `X`，未发送 `Y`。三场从当前 `baseline.yaml` 复制独立配置，保持 P6/P7 的 linear 目标、4 m/s² 启动斜率、零海浪振幅及几何诊断；目标 Y 速度仅设 0、+4、−4 m/s。控制仍由现有 `simulation_truth` predictor 提供目标状态，视觉/KF 保持 shadow-only。每场分析从 Gazebo UAV 首次达到稳定高度约 5 m 后 2 s 起取 30 s；+Y 首次捕获窗口较短，故**同一仿真会话**追加只读 40 s 采集，稳态指标采用后者，早期窗口单列。静态窗口 294/294 严格配对，+Y 稳态 226/226，−Y 237/242（4 帧 `DEPTH_MAD_HIGH`，另 1 帧未形成完整配对）；对应有效率 100%、100%、97.9%，有效样本速率约 9.8、7.5、7.9 Hz。采集/ULog 真值的左右单侧间隔 P95 分别约 16–23/19–20 ms，无外推、无跨 reset。原始文件和每帧源 bracket、拒绝原因保存在 [`data/experiments/current/p7_kf_20260929/`](../../data/experiments/current/p7_kf_20260929/)；下表唯一采用各场 `analysis_validated/analysis.json`，探索版 JSON 保留但不用于结论。

下表的误差均相对 **PX4 local NED 操作性真值**，同一配对帧计算；单位 m。

| 场景 | n | Current-B RMSE / P50 / P95 / max | Current-KF RMSE / P50 / P95 / max | B 的 XYZ 有符号均值 | KF 的 XYZ 有符号均值 |
|---|---:|---|---|---|---|
| Static 30 s | 294 | 0.197 / 0.197 / 0.210 / 0.214 | 0.197 / 0.197 / 0.211 / 0.216 | (+0.029,+0.192,+0.028) | (+0.029,+0.193,+0.028) |
| +Y 稳态 30 s | 226 | 0.121 / 0.120 / 0.130 / 0.133 | 0.120 / 0.121 / 0.131 / 0.138 | (+0.101,+0.066,−0.003) | (+0.101,+0.065,−0.003) |
| −Y 含过渡 30 s | 237 | 0.160 / 0.120 / 0.286 / 0.388 | 0.166 / 0.118 / 0.304 / 0.469 | (+0.074,−0.084,−0.055) | (+0.073,−0.084,−0.055) |
| +Y 早期短窗口，仅诊断 | 136 | 0.804 / 0.108 / 2.141 / 2.435 | 0.844 / 0.101 / 2.315 / 2.583 | (−0.435,+0.108,−0.015) | (−0.442,+0.117,−0.009) |

KF 的位置 RMSE 相对 raw：Static **+0.0001 m**，+Y **−0.0003 m**，−Y **+0.0057 m**，+Y 早期 **+0.039 m**；负号才表示改善。它没有消除各场有符号偏差，−Y 的 P95/max 反而增大。Current-B 消息**没有原生速度**，仅以相邻有效图像位置差分作为 raw 速度噪声参照，RMSE 为静态/+Y/−Y **0.120/0.123/0.274 m/s**；KF 直接输出速度相对操作性目标速度 RMSE **0.021/0.022/0.099 m/s**。操作性速度定义为目标命令真值速度 `+ PX4 EKF UAV velocity − PX4 groundtruth UAV velocity`；它不是把原 P7 truth recorder 的“目标命令速度”误当成该参考位置的严格导数。动态 −Y 早期仍有 KF 速度误差，不能据稳态低噪声就称动态追踪已完善。

## 3. 误差来源与 ±Y 不对称

| 同帧分量 3D RMSE (m) | Static | +Y 稳态 | −Y 含过渡 | +Y 早期短窗口 |
|---|---:|---:|---:|---:|
| 相机几何：Gazebo 模型姿态反事实 − 操作性真值 | 0.045 | 0.090 | 0.124 | 0.233 |
| PX4 姿态贡献：实际 Current-B − 模型姿态反事实 | 0.202 | 0.157 | 0.179 | 0.744 |
| 同图像时刻采集 PX4 UAV 位置 − Gazebo UAV 模型 | 0.046 | 0.218 | 0.439 | 0.650 |
| ULog PX4 EKF − ULog groundtruth，**同 PX4 时基** | 0.046 | 0.392 | 0.329 | 0.408 |
| raw 相对 Gazebo 物理实体位置 | 0.195 | 0.192 | 0.527 | 1.072 |

静态约 0.197 m 主要来自姿态作用在约 6.7 m 目标距离上的几何杠杆（模型/PX4 yaw 残差中位约 −0.040 rad），相机系自身约 0.045 m；没有“固定世界轴旋转”或软件重复旋转的证据。+Y 早期 UAV 为追赶目标曾约 6.4 m/s、目标距离约 17 m，前 10 s raw/KF 达 **1.118/1.173 m**，其中姿态分量 RMSE **1.037 m**；其后回到约 0.088/0.088 m。−Y 首 10 s UAV 平均约 −5.53 m/s 而目标 −4 m/s，raw/KF **0.226/0.238 m**；后 20 s 为 **0.115/0.115 m**。+Y 后期相同 20 s 为 **0.121/0.120 m**。因此“−Y 恒速视觉始终比 +Y 差”不成立；全段差异主要由选择到的追赶阶段、相对运动和姿态误差共同造成。早期 +Y 的更大异常也如实保留，见 [`phase_comparison_validated.json`](../../data/experiments/current/p7_kf_20260929/phase_comparison_validated.json)。

## 4. 时间归因与可用性限制

ULog 时钟仅从**ROS 采集 PX4 EKF 位置 ↔ 同一 ULog EKF 位置**拟合，不用 Gazebo 或目标误差反调。拟合自检 RMSE 静态/+Y/−Y 为 0.00005/0.00416/0.00020 m；ULog groundtruth 用左右 ≤50 ms bracket。与独立的图像/Gazebo 仿真时钟偏移相比，中位时间原点差约 **−0.44/+48.24/+28.10 ms**，−Y P95 达 +37.72 ms。用 **UAV 自身 groundtruth 速度×该时间差**，可解释 ULog groundtruth 与 Gazebo 同图像时刻模型的 3D 差 **约 0/0.194/0.133 m**；扣除后 RMSE 约 **0.000001/0.000051/0.000844 m**。这证明两个时间标签的相对差异足以造成动态图像对照的明显位移，不能将“ULog GT−Gazebo 模型”直接写成 PX4 坐标轴或 EKF 原点跳变。其时钟语义还未证明哪一端该改，故没有在线固定时间补偿。目标实体与数学 `/target/state` 的同帧差约 0/0.112/0.121 m，显示可视化实体与数学目标也有独立的动态采样/发布差；评价视觉时继续用实际实体图像时刻位置。

RGB/Depth 原始采样 skew 中位约 48 ms，但按 `(目标命令速度−UAV groundtruth 速度)×skew` 估计的相对位移 P50/P95 为 Static **0.0007/0.0019 m**、+Y **0.0010/0.0030 m**、−Y **0.0013/0.116 m**。−Y 部分追赶帧的理论位移与相机误差范数相关系数约 **0.78**，提示它可能参与早期退化；+Y 稳态相对速度很小，不能仅凭 48 ms skew 宣称它是主因。该乘积是运动学诊断，未把 RGB/Depth 理论位移当作已实测的独立因果量。

在 −200～+200 ms、10 ms 步长 lag sweep 中，用**所有 lag 共同有效且无外推**的帧比较，Static 209 帧（目标不运动，最优 lag 无辨识意义），+Y 稳态 70 帧、−Y 63 帧。动态 raw/KF 最低点均约 **+20 ms**；该子集小、含姿态和相机系统偏差，不能把它作为在线固定补偿。KF 来源图像年龄 P50/P95 为 Static **0.200/0.200 s**、+Y **0.200/0.348 s**、−Y **0.100/0.200 s**；它已在发布时做常速度投影，`source_stamp` 年龄本身不等于输出滞后。图像采集→观测发布年龄 P50/P95 约 Static 0.100/0.152 s、+Y 0.104/0.153 s、−Y 0.075/0.132 s。

## 5. 修改、验证与下一步

本轮新增 [`scripts/p7_kf_dynamic_analysis.py`](../../scripts/p7_kf_dynamic_analysis.py)（严格配对、双真值、平台/姿态/相机分解、共同样本 lag sweep）、专用 launch、[`test_p7_kf_dynamic_analysis.py`](../../src/uav_control/test/test_p7_kf_dynamic_analysis.py)；扩充 [`test_target_kalman_filter.py`](../../src/uav_control/test/test_target_kalman_filter.py) 验证图像时间 `dt`、predict→update 和乱序拒绝。在线 `rgbd_target_localizer.py`、KF 实现、Q/R、MINCO/MPC、任务控制及配置基线均未修改。最终验证：四个相关测试文件 **85 passed**；新增及受影响 Python 文件 `py_compile`、`flake8` 通过；`colcon build --packages-select uav_control uav_usv_bringup` 两包通过；`git diff --check` 通过。本轮没有发现足以直接修改在线时间基准、位姿转换或 KF 更新顺序的确定性代码错误；首先需要独立界定 PX4/Gazebo 时间标签与 USV 可视化发布时延，并复核动态姿态估计和强追赶视角。**当前没有足够证据进入 Q/R 调参**。

三场都采用原 PX4 预飞检查通过且 `failsafe=false` 后发 `X`。静态场最终正常 disarmed。+Y/−Y 远离起点的海面位置执行 `AUTO_LAND` 后未触发地面接触，PX4 未报 landed；采集已完成后只终止本轮自有 PX4/Gazebo/DDS 进程，未触及用户会话。实验工具没有改动飞行安全限制。原始 ROS CSV、同场 KF JSONL、场景 YAML、`analysis_validated/paired_frames.jsonl` 与 `analysis.json` 均保存于上述目录；对应 PX4 ULog 为 `/data/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-29/07_50_28.ulg`、`07_56_36.ulg`、`08_03_12.ulg`。无 commit/push。
