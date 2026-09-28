# P6.3：动态目标位置获取最终验收与当前 B 前端 KF 基线

## 验收结论（2026-09-28）

【获取率】+Y 4 m/s：**231/232，99.57%**；−Y 4 m/s：**224/224，100%**。两向 `POSITION_TIMESTAMP_AFTER_HISTORY` 均 **0**。+Y 仅 1 条 `RGB_FRAME_UNMATCHED`，未从分母剔除；`DEPTH_MAD_HIGH`、`RGB_DEPTH_TIME_MISMATCH`、`IMAGE_INVALID` 均为 0。

【精度】静态 FOLLOW 的 E1/E4 RMSE 为 **0.0451/0.0460 m**；动态 +Y 为 **0.0881/0.0883 m**；动态 −Y 为 **0.1733/0.1706 m**。E1 为相机系相对物理球心的误差，E4 为同帧 PX4 NED 下的目标参考点误差。E3 是 Gazebo/PX4 跨框直接差值，仅保留诊断，不用作视觉精度。

【时间】有效 RGB/Depth 对的绝对 raw skew：+Y P50/P95/max **48/52/100 ms**；−Y **52/52/52 ms**。+Y 一条被拒帧的 raw skew 为 652 ms，使全体帧 max 达 652 ms。动态有效帧的 position/attitude history 均覆盖测量时刻，两向没有 pose-history 拒绝。

【结论】**RGB-D target position acquisition P6 accepted. Perception baseline frozen after P6.3.** 不修改 localizer、球心/代表射线、外参、MAD、pose timing、RGB/Depth 配对或 PX4 EKF；规划控制继续 `simulation_truth`，视觉继续 shadow-only。

【KF】只给当前 B 前端的同时间原始测量与 KF 诊断基线，**不调 Q/R**。动态会话的固定 Gazebo→PX4 映射明显漂移，不能据此可靠给出 KF 绝对位置/速度真值 RMSE 或 `KF_error − raw_error`；详见下文。

## 场景与样本口径

从当前 P6.2 静态配置复制两份，除 `moving_target.velocity_y=+4.0/-4.0` 外无参数差异。保持 `maximum_depth_mad=0.25 m`、`pose_wait_timeout=0.15 s`、`maximum_pose_wait_gap=0.15 s`、当前 ray-radius B 前端；主目标预测源仍为 `simulation_truth`。每场正常安全起飞：X 前逐次确认 PX4 `pre_flight_checks_pass=true`、`failsafe=false` 与 `/simulation/impact/flight_ready=true`；X 后确认 PX4 armed、OFFBOARD、任务 FOLLOW 和目标真值 `velocity_y=±4.0 m/s`。未发送 Y，每向只采一段约 28 s。ROS 采集器 `vision_static_capture.py` 的逐帧字段原样复用；小型子类仅将 KF `TargetState` 另存一份 JSONL。每场仅有一个原始 CSV，未采 ROI 视频。

完整 E1/E4 统计仅包括同一图像帧有完整 Gazebo 实体、UAV 模型、PX4 位姿且不跨 reset 的样本；+Y 为 231 帧，−Y 为 222 帧（另 2 条有效观测缺完整物理参考）。获取率则使用全部 observation，不因评价真值缺失而缩小分母。

## 动态获取与几何精度

| 场景 | 获取率 | AFTER_HISTORY | E1 RMSE/P50/P95/max (m) | E4 RMSE/P50/P95/max (m) |
|---|---:|---:|---:|---:|
| 静态 FOLLOW（P6.2） | 243/243 | 0 | 0.0451 / 0.0447 / 0.0503 / 0.0530 | 0.0460 / 0.0456 / 0.0512 / 0.0540 |
| +Y 4 m/s | 231/232 | 0 | 0.0881 / 0.0932 / 0.1030 / 0.1074 | 0.0883 / 0.0932 / 0.1031 / 0.1076 |
| −Y 4 m/s | 224/224 | 0 | 0.1733 / 0.1706 / 0.2506 / 0.2566 | 0.1706 / 0.1677 / 0.2479 / 0.2543 |

相机系 E1 有符号 XYZ 均值：+Y **(+0.060, −0.056, +0.024) m**，−Y **(+0.140, +0.071, +0.050) m**。E4 有符号 XYZ 均值：+Y **(−0.057, +0.064, −0.008) m**，−Y **(−0.068, −0.146, −0.014) m**。−Y 的相机系误差高于 +Y 约 0.085 m，保留此方向差；两向 E4 均与 E1 基本一致，未见相机→PX4 NED 链新增米级误差。没有按方向加固定补偿。

历史 P5 离线 B 的 +Y RMSE/P95/max 约 0.197/0.371/0.422 m，−Y 约 0.191/0.274/0.421 m。本次在线 B 两向均未明显劣于历史量级。采集窗口、UAV 轨迹和场景时刻并非严格配对，不能把数值差直接归因于本轮算法改进；本轮**没有修改在线几何算法**。

## 同步与 pose-history

`future_gap = measurement_stamp − history_end_stamp`；负值表示 pose history 已覆盖图像时刻。单位 ms。

| 场景/流 | P50 | P95 | max |
|---|---:|---:|---:|
| +Y position | −50.82 | −2.50 | −0.30 |
| +Y attitude | −52.43 | −4.83 | −3.09 |
| −Y position | −16.53 | −11.08 | −10.31 |
| −Y attitude | −17.69 | −14.76 | −14.31 |

两向均为 0 条 `POSITION_TIMESTAMP_AFTER_HISTORY`，也没有新的系统性 clock/pose 拒绝。+Y 的最近 position bracket 最窄约 0.30 ms，属于有效覆盖，不据此调等待门限。

按有效且具物理参考的帧分组，`|raw RGB/depth skew|<10 ms` 的 E1 RMSE：+Y **0.094 m（49 帧）**、−Y **0.199 m（65 帧）**；40–60 ms：+Y **0.086 m（181 帧）**、−Y **0.161 m（157 帧）**；>80 ms：+Y **0.092 m（1 帧）**、−Y **无样本**。这些组的距离和视角未受控，不能推断 skew 的独立因果效应。+Y 唯一 `RGB_FRAME_UNMATCHED` 的 raw skew 为 652 ms、`measurement_stamp=0`，已被前端拒绝；有效帧最大 100 ms。未做时间补偿。

## 当前 B 前端的 KF 同时间基线

将每条有效 `TargetObservation.measurement_stamp` 作为查询时刻，以相邻两条有效 KF `TargetState.stamp` 线性插值位置和速度；每侧最多 0.12 s，不外推。+Y 配对 **231/231** 条有效观测；−Y 配对 **219/224** 条，余 5 条没有满足 bracket 的 KF 样本。所有差值均在 PX4 local NED 的同一时间计算。

| 指标 | +Y | −Y |
|---|---:|---:|
| KF − raw 位置差范数 RMSE / P95 | 0.0125 / 0.0198 m | 0.0235 / 0.0297 m |
| 局部线性趋势外高频残差 RMSE：raw → KF | 0.00815 → 0.00566 m | 0.01704 → 0.00896 m |
| KF source age P50 / P95 | 0.200 / 0.252 s | 0.200 / 0.348 s |
| 有效 observation 间隔 P50 / P95 | 0.100 / 0.200 s | 0.100 / 0.200 s |
| 沿 KF 速度方向的 lag proxy P50 / P95 | 0.00004 / 0.00189 s | −0.00008 / 0.00263 s |
| KF 速度相邻配对帧步长 P95 | 0.0184 m/s | 0.0282 m/s |
| 离线 replay innovation 范数 P50 / P95 | 0.00970 / 0.01923 m | 0.00990 / 0.03179 m |
| 离线 replay innovation covariance 对角均值 | 0.0374 m² | 0.0408 m² |

局部趋势外残差比较说明这两段近匀速数据中 KF 抑制了高频波动；沿轨 lag proxy 接近 0，但它只比较 raw 与 KF，**不是相对真值的动态滞后证明**。innovation 使用日志中的 range 重建测量协方差并离线重放当前 KF，属于诊断量，不能冒充在线内部记录。−Y 的 KF−raw 最大差为 0.204 m，但 P95 仅 0.0297 m；该尾部保留在 JSON。目标转弯未在本轮测试，不能据此调 KF 参数或宣称转弯表现。

**为何绝对 KF 真值 RMSE 留空：**现有 E4 真值是按每帧 UAV 位姿构造的 G→P 对照，不能当成长期固定标定。以稳定 FOLLOW 早段的固定变换评估后续目标，raw 对照误差 RMSE 为 **0.911 m（+Y）/2.451 m（−Y）**，远大于 E1/E4。独立看变换平移，首末各 5 s 的中位数在 X 方向漂移 **−3.74/−6.92 m**。所以这批动态数据不能可靠定义长期 PX4 NED 真值；raw/KF 的绝对真值 RMSE、成对 `KF_error − raw_error`、KF 速度真值 RMSE 均不报告。这是平台参考映射限制，**不推翻同帧 E1/E4 的视觉获取验收**，也不把逐帧 E4 当作 KF 评价真值。

## 文件、验证与后续边界

本轮只新增评价代码：`scripts/vision_p6_dynamic_capture.py`（继承既有静态采集，额外记录 KF JSONL）、`scripts/vision_p6_dynamic_acceptance.py`、`scripts/vision_p6_kf_paired_baseline.py` 与 `src/uav_control/test/test_vision_p6_dynamic_acceptance.py`。在线 localizer、KF、预测器、控制器、配置默认值均未修改。两份实验配置只改目标 Y 速度，且均保存在各自数据目录。

原始数据和分析：

- `data/experiments/current/p6_20260928/target_acquisition_dynamic_plus_y/`：一个 CSV、采集元数据 JSON、`kf_states.jsonl`、`frame_analysis.json`、`pose_history_analysis.json`、配置副本。
- `data/experiments/current/p6_20260928/target_acquisition_dynamic_minus_y/`：同上。
- `data/experiments/current/p6_20260928/target_acquisition_dynamic_acceptance_v2.json`：两向统一验收统计；早期同批分析 `target_acquisition_dynamic_acceptance.json` 保留，v2 明确区分有效对与被拒对的 skew max。
- `data/experiments/current/p6_20260928/target_acquisition_dynamic_kf_paired_baseline.json`：同时间 KF 诊断与固定映射限制。

实际验证：两个独立 SITL 会话均安全解锁、进入稳定 FOLLOW，目标速度分别为 ±4 m/s；每场仅一次约 28 s 采集，均未发送 Y。对每个本轮创建的会话核对 PID/PGID 后停止其 Gazebo/PX4/DDS/ROS 进程；未终止既有用户会话。新增脚本 `py_compile`、`flake8`，相关定向测试 **6 passed**，`git diff --check` 均通过。没有在线代码修改，故未运行全量测试或 `colcon build`。没有 commit/push。

**下一阶段**：保持视觉定位冻结。若要给 KF 绝对真值 RMSE，先独立解决动态 G→P 固定参考漂移，或建立可验证的 PX4-local 真值记录；再设计目标转弯段评估滤波滞后与预测。本轮数据只支持当前 B 前端下近匀速的噪声、innovation 和连续性基线，不支持直接调整 Q/R。
