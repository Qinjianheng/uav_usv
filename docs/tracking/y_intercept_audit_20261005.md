# Y 截获复验与导航采样时间修复（2026-10-05）

基线为 master/e4a32e4，加上 2026-10-02 未提交修改；保留相机下倾 25°。历史流程、预测、t_go、MINCO 边界条件见 `y_intercept_audit_20261002.md`。不能把历史多轮失败归为已证明的“唯一原因”。本次采用完整 X → 视觉 FOLLOW → Y 任务复验，真值只用于评价。

## 新基线与可复算证据

- `data/experiments/current/y_intercept_20261005_baseline.jsonl`
- `data/experiments/current/modular_intercept_20261005_150156_153738_mission_1*`
- 原生 ULog：`/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-05/07_01_36.ulg`
- 结果 TIMEOUT，最小距离 0.542215 m，评价半径仍为 0.50 m。

首次失锁之前有 11 次 `STATE_POSITION_MISMATCH`。以本轮 Gazebo sim/system 时钟对应关系映射拒绝时刻，并对原生 ULog `timestamp_sample` 做有界插值（禁止外推），在同刻重新采样候选 MINCO：位置误差均为 0.2027–0.2687 m，低于 0.30 m 接纳门限。原始 tracker 把位置/速度的接收时刻当作采样时刻，导致较早的测量与较晚的轨迹参考比较。此处 native PX4 estimate 是估计状态，不是真值；审计没有把真值送入在线控制。

脚本及结果保存于 `y_intercept_evidence_20261005/y_native_reject_audit.py`、`y_native_reject_audit.json`。

## 最小修复

新增 `/navigation/uav_state` 和 `UavState.msg`，复用 localizer 现有的因果 PX4/DDS offset 历史与 Gazebo 时钟映射。导航发布与目标检测结果无关。保留收到的 DDS `timestamp_sample` 和恢复出的 simulation sample time；P/V/A 不做位置补偿。没有时钟插值区间时等待，超过原 pose_wait_timeout 丢弃；时钟重置发布 invalid 并清空历史。

planner、tracker 改用该话题，状态 freshness 和轨迹交接误差采用物理采样时刻。planner 在目标 forecast epoch 晚于 UAV sample epoch 时，只模型推进初始规划边界 P/V/A，原 `uav_source_stamp` 保留采样时刻用于 freshness；已有轨迹时继续以旧参考在未来 handover 时刻的 P/V/A 重规划。没有外推图像时间或更改图像采集戳。

新增测试覆盖交付延迟导致的虚假位置门拒绝、无时钟右端点时等待、超时丢弃、重置 invalid、非法/未来/非有限状态拒绝和时间 provenance。四包构建通过；784 passed、1 skipped，两个已有 flake8 API 弃用警告。检查输出已保存于证据目录。

## 修复导航时间后仍失败：继续追踪剩余物理误差

- 原始记录 `y_intercept_20261005_navigation_1.jsonl`。
- 任务日志 `modular_intercept_20261005_152234_832799_mission_1*`。
- 原生 ULog `2026-10-05/07_22_03.ulg`。
- 正常起飞、FOLLOW、Y、MINCO_TRACKING、TERMINAL_MINCO；TIMEOUT，最小距离 1.824121 m。

3159 个导航位置样本与原生 ULog 最近 native timestamp_sample 的 P 完全相同，恢复时间误差范围 −0.253 至 +0.370 ms，中位数/P95 为 0。脚本 `y_navigation_provenance.py`。这是本轮重建精度验证，不代表 DDS offset 版本在任意环境下都不会改变。

本轮首个接受计划为 plan14；plan15/17/18 是 `TARGET_ENDPOINT_MISMATCH`，不是 UAV P/V 错误。Y+4.1 s 出现短暂图像交付延迟（有效图像到达时采集年龄 127.3 ms），进入 REACQUIRE；不能扩大 125 ms freshness 隐藏它。重新接纳 plan22/23 后，plan24/25 的 native 同刻速度误差约 1.11/1.10 m/s，plan26 的同刻位置误差约 0.51 m，这些拒绝对应真实跟踪偏离。导航修复不能算整个任务已完成。

tracker 当前生成 `V_cmd=V_ref+1.2*(P_ref-P_measured)` 并限速、限加速度；位置接口又发送 P_ref。本机 PX4 `PositionControl.cpp:127` 在速度 feed-forward 上再次叠加位置反馈。因此该模式没有直接执行 tracker 声称已受限的最终速度。当前 planner 已以旧接受参考重规划，满足旧配置注释中“只在不反复锚定测量速度后重启速度接口”的条件。本次继续只切换已有 `use_velocity_control` 参数，用 NaN position 将 tracker 的受限速度直接交给 PX4；原位置/速度门限、海面保护、125 ms 图像 freshness、0.50 m 截获半径均保留。

后续完整任务结果如下。离线测试与 planner success 均不替代实际捕获证据。


## 完整任务复验记录

| 修订阶段 | 原始 JSONL 后缀（`y_intercept_20261005_*.jsonl`） | 评价结果 | 最小距离 m |
|---|---|---|---:|
| 导航采样时间 | navigation_1 | TIMEOUT | 1.824121 |
| 速度接口 | velocity_1 | SEA_CONTACT | 1.0496 |
| 补回受限加速度 | velocity_acceleration_1 | TIMEOUT | 0.644396 |
| 过期接触计划恢复 | retry_1 | TIMEOUT | 0.666899 |
| 连续 FOLLOW | continuous_follow_1 | TIMEOUT | 1.436140 |
| 准备降低高度、短终端预测 | short_prediction_1 | TIMEOUT | 0.657592 |
| 前相机近裁剪 0.05 m | near_clip_1 | TIMEOUT | 0.524167 |

入口时刻与目标机动阶段不同，这些最小距离不能直接解释为捕获率提升。加速度遗漏、过期负 t_go、初始状态切换分别已有修复，完整捕获尚未验证。

## 起飞后连续 FOLLOW

X → TAKEOFF → FOLLOW。没有 pre-Y TARGET_ACQUIRE / TARGET_LOCK / REACQUIRE 往返；FOLLOW 内使用新鲜 RGB 方位接近，新鲜 KF 位置/速度追踪。共用速度整形历史，识别到目标不再要求先居中锁定才能跟随。无新鲜输入时有界减速/搜索，任务仍为 FOLLOW。

FOLLOW 接收 Y：已有 fresh lock 立即进入截获准备；否则保存请求，在 FOLLOW 内等待有效估计。Y 后仍保留准备、MINCO、终端和安全恢复子阶段，负责可执行轨迹和海面安全检查。

## Y 后估计与原生 USV 的同刻对照

脚本 `scripts/y_position_estimation_audit.py`。参考是原生 Gazebo 实体在图像采集时刻的 USV reference position。有界插值最大 0.15 s；禁止外推、拟合 lag 和以接收时间替代采样时间。KF 按自身状态 epoch；预测按 source epoch + horizon 对比该未来时刻原生位置。真值仅离线评价。

连续 FOLLOW 对应 `modular_intercept_20261005_155324_783839_mission_1*`，2358 个原生参考点。输出 `y_intercept_evidence_20261005/continuous_follow_position_audit.{json,png}`。

| Y+2–6 s 水平误差 | RMSE m | P95 m |
|---|---:|---:|
| 原始 RGB-D | 0.088 | 0.128 |
| KF 当前估计 | 0.083 | 0.109 |
| BCTRA +0.3 s | 0.095 | — |
| BCTRA +0.5 s | 0.113 | — |
| BCTRA +1 s | 0.194 | 0.324 |
| BCTRA +2 s | 0.604 | 1.186 |
| BCTRA +3 s | 1.697 | 3.302 |

最后持续执行 plan23：Y+5.178 s 接纳，剩余 t_go=2.805 s，终点相对实际接触时刻原生 USV 的水平偏差 1.938 m。约 8 cm 当前 KF 误差不能解释近 2 m 未来终点偏差。长预测受速度导数噪声、未知未来机动及模型截断影响；全程误差混入起飞、追赶、失锁、恢复，所以分固定窗口报告。

## 缩短预测及近距离几何问题

FOLLOW 保持 5 m。Y 准备平稳降到海面以上 1.5 m，速度整形共用；水平准备点仍按当前垂直可达时间生成，planner/tracker standoff speed 同为 0.8 m/s。完整 MINCO 动态约束不变，允许终端轨迹最大 1.5 s，末段 0.30 s freeze 保留。

首轮短预测对应 `modular_intercept_20261005_162624_040854_mission_1*`。Y+13.589 s 接纳首个 plan59，plan59/60/61 未来接触水平误差为 0.193/0.067/0.055 m；plan62–65 为 0.161–0.243 m，实际连续替换 P/V/A。末段 Y+14.529–14.677 s，marker center 在 camera X 缩到 0.453–0.297 m，0.20 m near clip 裁掉半径 0.25 m 验证球的近表面：红像素 38731→21940→2602→192→0，depth_min 卡在 0.200 m，mask 仅剩边缘，定位误差 0.276→0.293→0.339→0.352 m，之后失锁恢复。

前相机 RGB/depth near clip 改为 0.05 m，localizer/front monitor minimum depth 同步；相机外参、验证球、Q/R、125 ms freshness、捕获半径和海面保护不变。复验 `modular_intercept_20261005_163442_290076_mission_1*` 仍超时，最小距离 0.524167 m。

进一步复现 RGB-D 几何的近距离近似误差：原方法以所有有效表面深度中值和像素中值加半径恢复中心，部分可见球面时不对应真正中心射线。合成物理 ray/sphere 交点测试两例先失败。新增有界（最多 512 点）3D 球面中心恢复，并验证秩、半径一致性及表面残差；少量/退化点维持旧方法。仅针对当前已知半径红球接口，不代表无标记 USV 感知已验证。球面恢复完整复验 `modular_intercept_20261005_164113_770185_mission_1*` 仍 TIMEOUT，最小距离 0.632802 m。


## 可达时间预筛选与 MINCO 终端速度不一致（明确代码缺陷）

`finite_horizon_intercept_planner.py::reachability_at` 原先采用水平终端速度 `v_usv`；但 `_candidate` 强制 `v_usv + closing_speed * direction`。同一接触点的预筛选和轨迹终端约束不一致，预筛选凭空增加减速要求，可能排除原本可行的锁定 contact，推动滚动时刻不断向后。

近球面复验中 plan57–63 的 contact_delay 连续为 0.070–0.082 s，接纳余量长期维持约 0.9 s；required_time 由 horizontal_min_time 主导，尚未进行候选验证就被提升为新的搜索下限。每轮发生“终点生成可行，但 committed contact 被 lower bound 排除”的情况，短预测仍无法按原时刻收敛。

独立物理反例：UAV 初始位置 x=0、速度 5.5 m/s，USV 初始 x=1.05、速度 4 m/s；t=0.7 s 接触点 x=3.85，UAV 匀速 5.5 m/s 即可到达，终端闭合速度 1.5 m/s，垂直保持 z=-0.33 m。旧 planner 却返回 HORIZON_INSUFFICIENT。新增 `test_short_committed_contact_uses_minco_closing_velocity_bound` 先红，证明这不是模型噪声或实际追踪误差。

修复对允许的 closing speed 候选分别计算与 MINCO 相同的终端速度，取最小 required_time 作为搜索下界。该下界不替代各条候选的完整限速、限加速度与海面检查；所有安全/接纳门限保留。后续复验和最终收尾结果见 `terminal_sphere_validation_20261005.md`。

当前 Git HEAD 已由用户更新为 master/6818e5a（10.5.1），该提交包含此前进行中的修订；其后的可达时间修复以此为基线。没有由本任务执行自动提交。


## 2026-10-06 最终收尾

用户要求保留当前效果并收尾。最新相机安装位置 (base_link FLU 0.18,0,0.15，模型原点外参 0.18,0,0.39)、28° 下倾与暗部红色检测修复完成两次完整任务：均首次进近 SUCCESS，Y 后约 6.45 s，末端相对速度 2.57/2.32 m/s，捕获前没有保护恢复，129/129 帧视觉有效，成功后 Gazebo 自动暂停。此前三次 centered_short_y 在恢复中触发的 SUCCESS 不计作正常终端制导成功。

Y 前 5 m 高度/5 m 跟随距离保持；最终误差、原始图像、真值边界、检查、分段原因与未验证项集中于 `terminal_sphere_validation_20261005.md` 和 `y_intercept_evidence_20261005/final_camera_origin_validation.json`。最终 826 passed / 1 skipped，未自动提交、推送或清理用户数据。


## 2026-10-06 追加停止修复后的最终确认

此前“最终收尾”一节是当时的历史结果，之后用户继续反馈捕获后小球未停止和末段减速。暂停物理世界不足以阻止 ROS 墙钟目标积分及 set_pose，现由原生 world stats 同步暂停状态，冻结积分和可视化更新，发布零速度；恢复不补积分暂停间隔。

完整试验 complete_pause_1 为 TIMEOUT，尚未触发成功暂停；随后固定 6.5 m/s 接触速度和提前接纳试验 cruise_contact_1 也 TIMEOUT，所有 MINCO 候选均被动力学检查拒绝。不同运行的目标运动相位不同，不能归因于捕获后的停止分支，也不能宣称原配置具有普遍捕获率。

撤回上述未验证控制改动、保留停止修复后，stop_only_baseline_1 完整 X → FOLLOW → Y 任务 SUCCESS：6.335783 s、最小距离 0.478032 m、相对速度 2.277366 m/s，捕获前 TRACKING、127/127 视觉有效；成功自动暂停后的约 7 秒原生 pose/info 和 world stats 证明飞机与球姿态冻结，131 个 ROS 目标状态位置固定、速度为零。没有显式手动 pause。最终 829 passed / 1 skipped；用户确认“就这个效果准备收尾”，不继续末端不减速改造。当前仍存在末段参考减速，未将此项宣称解决。

最终配置、所有追加成功/失败的证据、预测误差和冻结验证集中于 terminal_sphere_validation_20261005.md 与 y_intercept_evidence_20261005/final_stop_only_validation.json。工作区保留修改和原始数据，未自动提交或推送。

## 2026-10-06：末端速度修复后两次重复实验收尾

用户随后再次反馈末端减速并要求继续修复。当前 tracker 保留导航物理测量 epoch，在同一 epoch 比较 MINCO 位置反馈；发布时刻单独用于前馈、限幅和到期判断。剩余接触时间 0～0.7 s、预测水平距离不超过 2.5 m 且 KF/BCTRA 新鲜时，执行层按短时未来目标方向保持水平速度并升至 6.5 m/s，限加速度转向；竖直轨迹和海面保护保留，MINCO 数学终点仍为目标速度加闭合速度。

修复后首次 SUCCESS 为 4.413230 s。用户确认成功后要求两次重复，期间不改代码或参数：重复 1 SUCCESS，4.622916 s、最小三维距离 0.482564 m、相对速度 2.801147 m/s；重复 2 SUCCESS，4.484602 s、最小三维距离 0.461706 m、相对速度 2.870920 m/s。三次均第一次末端进近捕获、捕获前恢复次数零。最后 0.7 s 水平指令升至 6.5 m/s，实测水平速度逐样本上升，捕获前最后记录分别为 6.803260、6.783259、6.828834 m/s。三次自动暂停后的原生 UAV/目标姿态、世界时间均冻结，目标速度为零。

当前测试 836 passed / 1 skipped / 2 个已有弃用提示，flake8 及两包构建通过。Y 前 FOLLOW 5 m/5 m、125 ms freshness、KF Q/R、0.50 m 捕获半径及真值仅评价边界保持。最新实现、验证范围、原始前缀、速度曲线和停止证据见 [terminal_cruise_validation_20261006.md](terminal_cruise_validation_20261006.md) 与 [汇总 JSON](y_intercept_evidence_20261005/terminal_cruise_repeat_validation.json)。本次独立启动的仿真已关闭，未自动提交或推送。
