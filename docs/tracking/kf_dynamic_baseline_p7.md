# P7 动态 KF 基线：真值坐标审计与阶段门槛

日期：2026-09-28；代码基线：`d807b777b6df50a766c69013e1b0dde6795ca1dd`。本轮**没有完成 KF 绝对动态基线**。P7.0 揭示旧固定 G→P 映射的米级漂移主要是评价公式的姿态旋转杠杆伪影；P7.1 建立了纯离线、同图像时间的相对几何真值记录器；P7.2 静态 sanity 的 raw RMSE 0.242 m 与旧 E4 的 0.046 m 不一致。按原计划的失败即停止门槛，本轮未运行 P7.3 KF 绝对指标和 P7.4 转弯实验，也未调整 KF Q/R。

## 1. 根因与 frame 语义

详见 [P7.0 审计](kf_truth_frame_audit_p7.md)。旧 `scripts/vision_p6_target_frame_analysis.py::frame_mapping()` 用每一帧的 `R_PX4_att R_Gazebo_bodyᵀ` 旋转 Gazebo **绝对**位置，再算平移。动态 ±Y 中模型离世界原点约 140–250 m，时变姿态残差被放大为固定映射 X 平移首末段差 −3.735/−6.924 m。相同时间直接将 Gazebo UAV ENU 转 NED 后与 PX4 UAV 位置相减，首末段差仅约 (−0.008,−0.228,−0.064)/(−0.009,+0.275,+0.055) m。P6.3 动态采集段 ULog `ref_timestamp`、参考经纬高、位置与航向 reset 计数均不变；不能把旧数米差解释为 PX4 原点跳变。

新记录器使用 `q_P = p_PX4,UAV + [q_G,target_ref − p_G,UAV]`，G 的 target 球心和 UAV 模型位置均先统一到世界 NED，球心参考点加 +NED Z 0.42 m 一次。不使用逐帧机体姿态残差旋转目标；PX4/Gazebo 姿态都保留供诊断。它是当前在线 PX4 local NED 的**操作性参考**，包含 PX4 UAV 位置估计的不确定度，并非独立地理真值。目标速度主字段采用已有 `/target/state` 的 NED 命令速度；位置差分速度另存作一致性检查，不能将命令值冒充实际实体瞬时速度。

## 2. 记录、筛选和 P7.2 结果

`scripts/kf_truth_capture.py`只读取 P6 已有 CSV，不发 ROS 消息。以 `TargetObservation.measurement_stamp` 为时间；四路 Gazebo entity/model、PX4 position/attitude 必须有左右 bracket，左右每侧距查询时刻 ≤0.05 s；拒绝外推、缺真值、跨 reset 和时间回退。记录原始 source 与 mapped ROS bracket、原始位姿、PX4 ULog 的参考建立时间标识、reset epoch 和拒绝原因。该 CSV 本身已由原采集器按图像时刻插值，脚本不再二次插值。没有启动新 Gazebo/PX4。

| P6 历史场景 | 有效真值/总帧 | 新连续真值下 raw RMSE / P95 (m) | 同帧子集旧 E4 RMSE (m) | 旧姿态旋转项 RMSE (m) | raw 误差末 5 s − 首 5 s 均值 XYZ (m) |
|---|---:|---:|---:|---:|---|
| 静态 FOLLOW | 239/243 | 0.242 / 0.253 | 0.046 | 0.248 | (−0.002,+0.006,−0.001) |
| +Y 4 m/s | 231/232 | 0.092 / 0.109 | 0.088 | 0.116 | (+0.046,−0.028,+0.005) |
| −Y 4 m/s | 222/224 | 0.226 / 0.300 | 0.171 | 0.214 | (+0.041,+0.022,−0.001) |

同一有效帧上逐向量恒等式：`new_raw_error − old_E4_error = (R_PX4 R_modelᵀ − I)(q_G,target_ref − p_G,UAV)`，三场最大数值残差分别约 1.8×10⁻¹⁵、4.8×10⁻¹⁴、4.5×10⁻¹⁴ m。这说明新旧指标差值来自是否旋转世界相对几何，不是记录器漏加目标高度或混用不同图像时刻。静态模型/PX4 yaw 残差中位约 −0.047 rad、目标水平相对距离约 4.98 m，对应约 0.24 m 的横向旋转杠杆，与新旧差值一致。旧 E4 消掉了这项，因此旧 E4≈相机误差不能证明 PX4 姿态对世界目标定位没有贡献。−Y 的约 0.055 m RMSE 增量同样不能忽略；不能仅展示 +Y 接近。

三场最大 bracket 单侧时间差 P95 分别约 23.18/23.23/23.05 ms；新连续真值没有观察到米级累计漂移。位置差分速度中位数约静态 (0,0,0)、+Y (0,3.986,0)、−Y (0,−3.987,0) m/s，与 0/±4 m/s 命令方向相符；但差分相对命令的速度误差 P95 在动态两场约 0.59/1.68 m/s，不能将单帧差分当作低噪速度真值。原始 XYZ、P50/P95/max、有效帧、时间边距与恒等式数值见 `data/experiments/current/p7_20260928/p7_truth_sanity.json`。

## 3. 阶段结论和下一步

**结论 C（本轮验收门槛）：尚不能凭现有资料把新操作性参考认证为可用于 KF 调参的独立绝对真值。** 这不等于已证明新参考错误：它连续、坐标运算自洽；失败点是原计划要求静态 raw RMSE 与 E4 同量级，而 E4 的姿态残差旋转恰恰遮蔽了约 0.248 m 的差值。PX4 local 轴相对物理北东的独立标定、姿态误差与 EKF 位置原点的联合关系，仍需用独立的双轴运动/全球定位对照核实。当前数据能够说明旧固定映射不可用、E4 不是长期真值，但不能仅凭“让 E4 变小”选择 KF 真值或加固定 yaw 补偿。

遵照 P7.2 失败停止规则，本轮**不报告** raw/KF 的正式同真值 position/velocity RMSE、KF−raw 绝对误差差、真实 cross-correlation/转弯 lag、创新峰值或恢复时间；没有开展 P7.3/P7.4，也没有进入 P7.5 调参。P6.3 的 KF−raw 相对诊断仍只是历史结果，不升级为绝对评价。下一步最小任务是独立验证 PX4 local NED 轴与世界 NED 的关系：结合 ULog GPS/groundtruth 及不共线的 UAV X/Y 运动，在与 RGB 图像同一时刻估计轴向与原点的稳定性；然后用静态和两向数据重新判定哪个误差是姿态估计、哪个是 frame 对齐。验证通过后再做同时间 KF 配对和两种转弯试验。不得把 Gazebo 真值反馈到在线视觉/KF/控制。

## 4. 文件、边界和验证

本轮新增 `scripts/kf_truth_capture.py`（纯离线转换、bracket/epoch 校验）、`src/uav_control/test/test_kf_truth_frame.py`（坐标、速度、时间、reset 测试）、`docs/tracking/kf_truth_frame_audit_p7.md` 和本报告。最终数据在 `data/experiments/current/p7_20260928/`：`static_truth_v2.jsonl`、`plus_y_truth_v2.jsonl`、`minus_y_truth_v2.jsonl`，各有 `.summary.json`，以及 `p7_truth_sanity.json`。目录中无 `v2` 后缀的三份 JSONL 是本轮转换器完善 source bracket 前的探索产物，最终指标只使用 `v2`。P6 原始 CSV、ULog、配置及旧分析未覆盖。

实际运行：P7 定向 pytest **4 passed**；`python3 -m py_compile scripts/kf_truth_capture.py` 通过；`python3 -m flake8 scripts/kf_truth_capture.py src/uav_control/test/test_kf_truth_frame.py` 通过。首轮因无 ROS 环境时导入 `rclpy` 失败，已将同一四元数残差公式实现为纯离线 SciPy 计算后通过；shell 中无独立 `flake8` 命令，使用等价的 `python3 -m flake8`。最终 `git diff --check` 和状态见收尾命令。本轮没有在线 Python/配置变更，所以未做 colcon build 或新仿真。在线 B 前端与 KF 参数不变，视觉/KF 保持 shadow-only，规划控制继续 `simulation_truth`；没有 commit/push，也没有发送 Y。
