# P6.1：目标位置坐标系归因与评价修正

日期：2026-09-28；分析基线 `62494953c20064612e7f7e97213945a041a9c2aa`。本报告只使用已有 P6 地面横移与 P5 左离轴原始 CSV 做 E1–E4，同帧真值仅进离线评价。在线 RGB-D、KF、预测、规划、控制及 PX4 参数均未修改；规划控制继续使用 `simulation_truth`，视觉保持 shadow-only。

## 先回答关键问题

1. `TargetObservation.position` 由 `camera_target_to_local_ned()` 用 PX4 local position 和 PX4 attitude 计算，属于 **PX4 EKF local NED（P）**。Gazebo `/world/default/pose/info` 的 ENU 转换后是 **Gazebo 固定 NED（G）**，机体系为 **FRD（B）**。代码默认 `frame_id='local_ned'` 未标出 P/G 区别。
2. 旧采集器把在线 P 位置与 G 中球体实体参考点直接相减，`vision_to_entity_3d` 因此是 **cross-frame diagnostic**，不能单独称为视觉目标定位误差。
3. P6 地面有效 168 帧 E1/E2/E3/E4 的 RMSE 分别为 **0.1174/0.1174/1.2405/0.1174 m**；P5 最新左离轴完整 248 帧为 **0.0746/0.0746/0.6849/0.0758 m**。约 1.24/0.685 m 的直接差值主要对应 P/G 航向及位置参考不一致，不能作为视觉前端米级错误的证据。
4. P6 目标平均距离 8.838 m，乘该会话映射 yaw 0.1332 rad 约为 1.18 m，与 E3 量级相符。P6 地面旋转项 RMSE 1.2206 m、位置项 0.0384 m；左离轴整段分别 0.6793/0.1280 m。旋转/航向项更大，但**各项向量可相加，RMSE 不能相减求贡献**。左离轴 yaw 映射从起飞附近约 0.13 rad 变到末尾稳定约 0.042 rad，不能称为跨会话固定 yaw 偏置，也不能推出应给在线视觉加固定补偿。
5. P6 的 63 条 `POSITION_TIMESTAMP_AFTER_HISTORY` 发生在未解锁 ground hold；新增采集字段后尝试一次稳定 FOLLOW 短验证，但 PX4 预检未通过，故具体时序根因**尚未确认**。历史 P5 左离轴稳定飞行 0/256 条同类拒绝，六场 P5 原始采集也没有此类 AFTER_HISTORY，支持条件相关性，但不足以在 A/B/C/D 中定因。在线 `pose_wait_timeout` 未改。
6. P6 相机系有符号均值约 **(+0.010,+0.023,+0.103) m**，剩余正 Z 偏差显著。七场 ROI 比较中没有一个替代代表射线满足“所有场景 RMSE/P95/max 不退化”；在线 B0 保留。

## 同帧定义与可审计结果

以 RGB 的 `measurement_stamp` 为查询时刻，完整有效帧要求实体、UAV 模型、PX4 位置和姿态均可插值，且 PX4 reset 计数不跨样本。球体实体中心为视觉目标；最终目标参考点按当前定位器定义为球心加 G/P 各自的正 NED Z `target_reference_z_offset=0.42 m`。四个指标均比较这一参考点。评价同时输出“先在 G 加 offset 后作刚体映射”的严格 E4，以及“先映射球心再在 P 加 offset”的在线语义核对；二者差异 RMSE 在 P6 为 0.0002 m、左离轴 0.0027 m。

令 G 中 UAV 模型位置与 FRD 姿态为 `p_g,R_g`，P 中 PX4 位姿为 `p_p,R_p`。逐帧定义 `R_gp=R_p R_gᵀ`、`t_gp=p_p−R_gp p_g`，把 G 目标参考点变为 `R_gp p_target_g+t_gp`。E1 是 B 相机球心减物理相机球心；E2 用**定位器同一相机→NED 函数**和精确 Gazebo UAV 位姿重建后在 G 中比较；E3 是在线 P 目标减未经对齐的 G 参考点；E4 是在线 P 目标减映射到 P 的同帧 G 真值。独立 PX4 样本重放在线结果，最大重放差小于打印精度；E1 和旧 `vision_to_entity_3d` 范数也与原 CSV 对齐。

| 场景/阶段 | n | E1 相机 RMSE | E2 G 重建 RMSE | E3 跨框 RMSE | E4 同框 RMSE | E4 P95/max | 映射 yaw 均值±标准差 |
|---|---:|---:|---:|---:|---:|---:|---:|
| P6 地面横移 | 168 | 0.1174 | 0.1174 | 1.2405 | 0.1174 | 0.1405/0.2069 | +0.1332±0.0004 rad |
| 左离轴整段 | 248 | 0.0746 | 0.0746 | 0.6849 | 0.0758 | 0.1130/0.1285 | +0.0726±0.0422 rad |
| 左离轴前 10 s | 85 | 0.1090 | 0.1090 | 1.1151 | 0.1091 | 0.1130/0.1285 | +0.1306±0.0090 rad |
| 左离轴末 15 s | 102 | 0.0458 | 0.0458 | 0.2316 | 0.0485 | 0.0519/0.0537 | +0.0424±0.0007 rad |

单位：误差为 m。完整 XYZ signed mean、P50/P95/max、每帧 P/G 真值、映射 yaw/平移及样本筛选见 `data/experiments/current/p6_20260928/target_frame_analysis_v2.json`。P6 有效帧 E3 signed mean 为 (+0.243,+1.094,−0.107) m；E4 为 (+0.035,−0.019,−0.099) m。左离轴 E3 为 (−0.059,+0.503,−0.064) m；E4 为 (+0.042,+0.007,−0.018) m。

**可识别性限制：**逐帧 `R_gp,t_gp` 使用同一帧 UAV 的两套位姿构造，必然把该帧 UAV 对齐；E4 接近 E1 是几何上可预期的，不能单独证明存在时间不变的 G→P 坐标标定，也不能把所有 PX4 姿态误差重新命名为“纯框架误差”。P6 地面映射 yaw 仅变化约 0.0016 rad；用前 20% 中一帧映射评估后 80%，RMSE 为 0.123 m，支持**该地面会话**映射稳定。左离轴全段固定映射的后 80% RMSE 为 0.854 m；末 15 s 内同法为 0.092 m，显示起飞过渡中映射变化，稳定阶段也有剩余状态估计/位置漂移。其 EKF 航向初始化、磁融合与坐标重置的独立物理原因仍需另证，不能据此修改外参或航向参数。

E3 的逐帧**向量恒等式**为 `E3 = E4 + (R_gp−I)(p_target_g−p_g) + (p_p−p_g)`。地面旋转项 RMSE 1.2206 m、位置项 0.0384 m；左离轴整段 0.6793/0.1280 m，其中位置项最大 0.454 m。左离轴末 15 s 旋转/位置项为 0.2397/0.0954 m。这里的旋转项既可能包含局部框架方向差，也可能包含 PX4 当前姿态估计误差；该分解只描述导致**跨框直接比较**增大的数值机制，不将其当作在线定位器 bug。

## pose-history 可用率与短验证

P6 地面原始采集 262 条观测中 168 条有效、63 条 `POSITION_TIMESTAMP_AFTER_HISTORY`、29 条 `IMAGE_INVALID`、其余 2 条。旧 CSV 缺少在线 position/attitude history 的起止时间与 source/mapped 时间，不能从拒绝原因反推出到底是更新频率、ROS 回调时序、等待窗口还是时钟映射。本轮在 evaluation-only `vision_static_capture.py` 原样增加 10 个 `TargetObservation` pose-history 时间字段；不调整在线 0.15 s `pose_wait_timeout`、QoS、配对或外推。

为验证飞行阶段，本轮只启动一次静态目标配置，发送 `X` 后等待 FOLLOW，计划采集 28 s。实际 PX4 `arming_state=1`（未解锁）、`pre_flight_checks_pass=false`、`nav_state=14`，解锁 `VehicleCommandAck.result=1`；任务停在 TAKEOFF，局部 Z 仍接近地面。同期 QGroundControl AppImage 进程出现段错误，但**没有证据证明它导致预检失败**。因此没有启动 FOLLOW 采集，也没有新 pose-history CSV；仅保存独立配置 `data/experiments/current/p6_20260928/pose_history_hover/config.yaml`。已停止本次启动的 Gazebo/PX4/DDS/ROS 进程，不绕过预检。历史 P5 left_mad_recheck 249/256 条有效、0 条 AFTER_HISTORY；这支持先解决仿真预检、再按新字段判定 63 条原因，不支持现在盲调等待时长。

## 相机代表射线统一复算

同一保存 ROI、相同深度中位数与已知半径，只有射线像素中心改变：B0 为当前**有效深度像素中位数**，B1 分别检查红掩膜像素中位数和质心，B2 为红掩膜 bbox 中心。七场均使用各自相同的完整有效 ROI，Gazebo 真值只作最终评价。P6 在线 B 与离线 B0 最大差约 `3.2e−7 m`；旧 P5 采集在线仍为 A，故其在线中心与离线 B0 不应相等。下表为相机系 RMSE/P95（m），详细 max、XYZ 均值与有效率在 `representative_ray_analysis.json`。

| 场景 | B0 当前 | B1 红像素中位 | B1 红质心 | B2 bbox |
|---|---:|---:|---:|---:|
| static5 | .0933/.1147 | .0941/.1147 | .0927/.1141 | .0938/.1147 |
| static7 | .0936/.1147 | .0936/.1147 | .0934/.1141 | .0944/.1147 |
| left | .0933/.1130 | .0931/.1130 | .0929/.1119 | .0920/.1093 |
| right | .0923/.1246 | .0931/.1246 | .0916/.1216 | .0916/.1210 |
| +Y | .1966/.3710 | .1958/.3690 | .1924/.3469 | .1838/.3295 |
| −Y | .1912/.2738 | .1881/.2694 | .1837/.2684 | .1753/.2649 |
| P6 地面 | .1141/.1232 | .1205/.1321 | .1221/.1317 | .1130/.1188 |

B2 在 ±Y 和 P6 有一定改善，但 static5/7 的 RMSE 略退化，右侧 max 从 .1246 到 .1359 m；B1 红质心在 P6 地面从 .1141/.1232 退化为 .1221/.1317 m，且其 Z signed mean 从 +.1016 升到 +.1057 m。没有候选满足预设全场景门槛，**不修改在线代表射线**，不调整 camera pitch 或固定 Z 补偿，不采用历史已表现出异常尾部的球面拟合。P6 低速约 52 ms skew 对球心的理论位移 P95 为 0.0276 m，与剩余约 0.11 m 相机误差分开报告；历史 4 m/s 场景的同步问题仍保留。

## 修改、验证与下一步

实际新增 `scripts/vision_p6_target_frame_analysis.py`（E1–E4、刚体映射、旋转/位置向量分解、分阶段与固定映射留出评估），`scripts/vision_p6_representative_ray_analysis.py`（七场 ROI 代表射线复算），`src/uav_control/test/test_vision_p6_target_frame_analysis.py`；修改 `scripts/vision_static_capture.py` 的 `pose_history_diagnostics()` 和 CSV 写入，原样保存 10 个在线时间字段。所有改动仅在评价/采集与测试层；保留旧 `vision_to_entity_3d`，本报告及新 JSON 明确其跨框语义。在线视觉/KF/控制行为不变。

输出数据：`data/experiments/current/p6_20260928/target_frame_analysis_v2.json`、`representative_ray_analysis.json`；`target_frame_analysis.json` 是本轮早期同批计算版本，均保留；P6 第一组与 P5 旧数据原路径未覆盖。新增短验证因 PX4 预检失败，**没有可报告的改后飞行目标获取率**。下一动作仅为查明并恢复 PX4 安全预检后，用已增加的字段完成一次 20–30 s 稳定 FOLLOW pose-history 采集，再决定是否需要在线等待/回调修复。

验证：`python3 -m py_compile` 对两段新增分析脚本、修改的采集器及相关测试通过；修改文件的 `python3 -m flake8` 通过；针对性 pytest **91 passed**，`src/uav_control` 包完整测试 **454 passed、1 skipped**（两条既有 `SelectableGroups` 弃用警告）；`git diff --check` 通过。改动仅在评价 Python 脚本与测试，没有改 ROS 包安装入口或在线节点，因此未运行 `colcon build`。短 FOLLOW 仿真因预检失败、没有采集结果，不能把这些代码检查称为 pose-history 问题已修复。
