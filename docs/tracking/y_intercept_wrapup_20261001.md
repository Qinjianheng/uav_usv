# 完整 Y 截获验证收尾（2026-10-01）

## 当前结论

按用户“先收尾吧”的指令停止继续修改和第三轮实验。两轮完整仿真均接收到 Y 并进入 MINCO/末段制导，但均未完成截获：当前结果为 FAILURE / TIMEOUT，不能宣称任务已解决。

当前模块化规划和评估器的截获半径都是 0.50 m；旧 trajectory_impact_sim 配置含 0.25 m。本次未扩大、降低或改变现有判据，两次最小距离也都大于 0.25 m。

| 运行 | 本次状态 | Y 后结果 | 最小三维距离 | 超时时间 |
|---|---|---|---:|---:|
| 110106_769753 | 修正低高度 yaw 前 | TIMEOUT | 0.763384 m | 30.001565 s |
| 110626_933861 | 修正低高度 yaw 后 | TIMEOUT | 1.700030 m | 30.022736 s |

两次运行的噪声、局部参考原点和进入 Y 的时刻不同。这是故障复现和修复后行为检查，不是严格匹配的效果提升试验；第二轮整体截获结果没有改善。

## 已确认并保留的修复

首轮日志确认：当 UAV 下降到约 1.5 m 以下时，目标仍可见、KF 新鲜、locked=true，但 yaw 仲裁命令被置零；26 个低高度已锁定的周期样本中 25 个 yawrate 为 0，图像 bearing 随后从约 0.14 rad 增加到 0.84 rad，目标退出画面。

原因位于 trajectory_tracker_node.py 的 _final_yaw：target_search_enable_height 同时抑制了起飞/搜索和已获准的末段视觉偏航。现已只对已请求 Y、MINCO_READY/MINCO_TRACKING/TERMINAL_MINCO、locked=true 且 visible=true 的情况放行低高度视觉对准。安全恢复、结束状态、低高度 FOLLOW 和未接受 Y 的情况仍保持原有禁止条件。

第二轮 14 个低高度已锁定周期样本中没有再次出现零 yawrate，说明这条仲裁错误已修正；目标仍在末段离开视野，任务依然超时。现有偏航仅对图像误差作比例反馈，近距离运动的视线角速度与图像偏航命令明显不一致；轨迹横向偏差、KF/BCTRA 预测、计划接纳/替换和机体姿态对视野的贡献尚未完成因果拆分，不能只凭此确定下一项修复。

没有新增 LOS 角速度前馈、下视相机控制接管或其他制导修改。控制仍仅使用视觉/KF 数据；真值仅供评价，EKF/KF Q/R、图像时间、0.125 s KF/control freshness、原有 0.15 s RGB-visible 门限和截获半径均未改变。

## 验证与工作区状态

- 新增 test_terminal_visual_yaw.py：3 个允许条件由失败转为通过，6 个禁止条件通过。
- 本次相关测试 69 项通过；完整 uav_control 测试 745 passed、1 skipped，两个既有 flake8 入口 API 废弃警告。
- 四个 ROS 包完整构建通过；本次文件 flake8 与 git diff --check 通过。
- 独立代码审查确认本轮 timer 先更新 visibility 再执行 yaw 仲裁，未发现重要问题。
- 已确认所有测试会话 PID 退出，实际 Gazebo/PX4/MicroXRCEAgent/modular_intercept 进程为空；监测器和命令控制台已关闭。没有把 Q 当作运行进程退出证据，也没有声称完成真实降落。
- 没有 Git 提交、清理、重置或删除用户日志。HEAD 仍为 master / 77872e9，前面 FOLLOW、导航和构建扫描修复及本次 yaw 修复都留在未提交工作区。

## 可复算证据与继续入口

汇总及输入 SHA256：y_intercept_evidence_20261001/comparison.json；当前代码 SHA256：final_source_hashes.json。run1_monitor.log 和 run2_monitor.log 含自动 Y 触发记录；pytest.log、build.log、flake8.log、independent_review.txt 和 cleanup_status.txt 保存验证证据。

原始文件位于 data/experiments/current/：

- modular_intercept_20261001_110106_769753_mission_1.csv / _vision.csv / _summary.json / _config.yaml；原生 ULog 03_00_24.ulg。
- modular_intercept_20261001_110626_933861_mission_1.csv / _vision.csv / _summary.json / _config.yaml；原生 ULog 03_05_24.ulg。

ULog 均位于 /home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-01/。未起飞的中间干净重启还留下 03_03_22.ulg，保留且未混入以上任务统计。

若继续，先读取当前代码和这两轮完整事件顺序，重点对齐 Y→首次 MINCO→低高度→图像边缘/测量误差→首次计划拒绝→失锁→恢复，区分失锁前因和失锁后派生的 PREDICTION_STALE/PLAN_STALE 统计。再提出一个有独立验证的最小修复，不放宽 freshness 或 capture radius。正常启动命令仍是 ./scripts/uav_lab.sh。
