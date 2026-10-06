# Y 截获验证与收尾（更新于 2026-10-06）

> 本文保留此前停止修复及成功/失败试验的历史记录。后续已修正导航反馈 epoch 并增加末端速度保持，用户要求的两次独立重复实验均成功；最新收尾依据见 [terminal_cruise_validation_20261006.md](terminal_cruise_validation_20261006.md)。下文关于“末段减速尚未解决”和 stop_only 最终配置的表述属于当时状态。

当前基线为 master/6818e5a（用户提交的 10.5.1），下述修订保留在工作区，未自动提交或推送。用户在最新 stop_only_baseline_1 成功后确认“就这个效果准备收尾”。最终保留原有截获速度和准备接纳规则，以及捕获后飞机和目标共同停止的修复；撤回未通过完整验证的固定 6.5 m/s 接触速度和提前接纳试验。

## 本次结果

此前 camera_origin_1/2 的完整 X → FOLLOW → Y 仿真都在首次末端进近中捕获，捕获前没有进入 SAFE_RECOVERY/REACQUIRE；捕获前最后控制状态均为 TRACKING、视觉锁定、SAFE。成功后 evaluator 自动暂停 Gazebo，两次原生 world stats 均显示 paused=true、simulation time/iterations 不再增长。随后追加试验有成功也有失败，不能把整个过程报告为 2/2 或 3/3 的总体捕获率。

最新最终配置 stop_only_baseline_1：**SUCCESS，Y → 捕获 6.335783 s，最小三维距离 0.478032 m，水平距离 0.313900 m，垂向误差 0.360529 m，相对速度 2.277366 m/s，径向闭合速度 1.344492 m/s**。Y 后至捕获的 127/127 个视觉帧有效，捕获前最后控制状态 TRACKING；SAFE_RECOVERY 发生在成功暂停以后（Y+6.431 s），不属于捕获前失锁。

最新一次自动成功暂停后，额外读取约 7 秒原生 `/world/default/pose/info`、world stats 和 ROS `/target/state`：飞机与小球姿态保持不变、simulation time 不增长、131 个目标状态位置不变且速度为零、64 个 world stats 均暂停。该验证没有手动发送 pause，也没有将评价结果送入在线制导。证据见 `final_stop_only_validation.json` 和 `stop_only_baseline_1_freeze_audit.json`。

| 指标 | camera_origin_1 | camera_origin_2 |
|---|---:|---:|
| Y → 捕获 s | 6.449859 | 6.450460 |
| 评价最小三维距离 m | 0.440815 | 0.491686 |
| 水平距离 m | 0.285701 | 0.322200 |
| 垂向误差 m | 0.335697 | 0.371405 |
| 末端相对速度 m/s | 2.573473 | 2.315411 |
| 径向闭合速度 m/s | 1.676275 | 1.218927 |
| 恢复次数 | 0 | 0 |
| Y 后有效视觉帧 / 总帧 | 129/129 | 129/129 |
| 捕获时最后有效图像年龄 ms | 31.263 | 32.875 |
| 捕获前最后控制器观测年龄 ms | 94.440 | 46.343 |
| 拟合中心水平偏差 P50/P95 px | 39.43/77.21 | 45.63/73.04 |
| 拟合中心垂直偏差 P50/P95 px | 35.26/88.04 | 37.33/113.38 |
| 最后 1 s RGB-D 水平 RMSE m | 0.094179 | 0.053197 |
| 最后 1 s KF 水平 RMSE m | 0.092680 | 0.068589 |
| 全程 UAV 水平速度峰值 m/s | 6.569886 | 6.359133 |

像素偏差相对 640×480 图像中心，统计包含 Y 准备和 MINCO；不表示每一帧球心均位于中心或完全在视场内。近距离部分球面仍能拟合中心。两轮末端原始 RGB-D 保存在 evidence 的 camera_origin_{1,2}_images/。

两轮采用同一场景、目标运动参数、配置和自动触发准则；目标运动相位随稳定 FOLLOW 的到达时间而异，不能称为完全匹配的同相位实验，更不能由两次成功推断一般捕获率。此前 centered_short_y 三轮虽然也触发 SUCCESS，但均先进入保护恢复；这些结果不计入本次正常视觉终端成功次数。

## 确认的问题与修复

1. 导航状态接收时间被当作物理采样时间，造成虚假交接位置误差。已有原生采样时间发布及同刻审计保留，详见 y_intercept_audit_20261005.md。
2. 近距离部分球面用中值表面射线加半径，造成中心位置偏差。采用最多 512 点的 3D 球面拟合，并检查秩、条件数、半径及残差；在线失败拒绝，不发布近似回退。此算法只用于已知半径的验证球。
3. 可达时间预筛选使用 v_usv，而 MINCO 终点使用 v_usv+closing_speed·direction，两者不一致。现按相同终端速度计算候选的可达下界，保留完整轨迹动态约束检查。
4. 原相机 near clip=0.20 m 提前删除近球面深度，前相机 RGB/depth near clip 和有效深度下限已同步为 0.05 m。
5. 末端原帧中阴影红球仍可见，红通道约 94；旧 red>=160 将可见目标判为 IMAGE_INVALID。改为 red>=90，保留颜色比例与 60 灰度的对比度要求。RGB/BGR 暗部回归先失败后通过；旧原帧回放证明原有严格球面质量门可通过，无须放宽拟合门限。
6. 暗部修复后观测缺口从约 0.4 s 缩短至约 0.2 s，但相机随后逼近/进入验证球面。原安装相对 base_link 为 (0.35,0,-0.05)，现移至前缘上方 (0.18,0,0.15)，下倾维持 28°。加上 x500_base 的 +0.24 m 模型偏移，定位外参为 body FLU (0.18,0,0.39)。SDF、默认外参和三份 YAML 同步；新增物理接触与校准回归。
7. Y 后用新鲜拟合中心的图像角反馈，辅以新鲜 KF 的视线角速度前馈；拟合中心不可用时仍受原图像 freshness 约束。准备阶段和末端均使用该 yaw 规则，Y 前 FOLLOW 规则不变。
8. 成功事件原先只发布结果，未暂停世界。现在评价器先发布结果，再调用已有的 Gazebo pause 客户端，并保存日志；暂停失败明确记录，不伪造成功。未把 evaluator 真值结果送入在线制导。
9. 恢复统计曾漏计 SAFE_RECOVERY/REACQUIRE，现已计入。旧三次擦过捕获不能再被误报为首次正常制导成功。
10. 新拆分的 guidance 加速度默认值改为继承旧 maximum_horizontal_acceleration，避免旧配置意外增加 FOLLOW 加速度。主配置显式为 3.0，终端为 3.5；两次仿真有效参数与收尾版本相同。构造器回归验证默认继承和显式覆盖。
11. Gazebo 暂停只停物理更新，ROS 墙钟定时器仍可积分目标运动并调用 set_pose，导致飞机已停止而小球继续动。visualizer 订阅原生 world stats；moving_target 在 paused 时停止积分和 set_pose，发布冻结位置及零速度，恢复时不补积分暂停期间的时间。没有订阅评价结果来改变目标行为。暂停/恢复回归和最新自动捕获后的原生姿态验证均通过。

这些是沿输入、规划、控制和传感器边界分别确认的缺陷，不将历史所有失败强行归为一个原因。

## 保留的配置与边界

- Y 前 FOLLOW 仍为 5 m 高度、5 m 水平跟随距离；起飞后直接 FOLLOW，粗 RGB 方位接近在 FOLLOW 内执行，不增加前置任务状态。
- Y 后准备高度 1.2 m，动态准备点的距离由剩余垂直可达时间×1.5 m/s 决定；不是固定距离，也不是直接冲到目标中心。
- guidance 水平速度上限 6.2 m/s，tracker 指令上限 6.5 m/s，MINCO 规划上限 7.0 m/s；USV 标称 4.0 m/s。实际速度有超调，不能声称实测峰值严格低于指令上限。
- guidance 水平加速度上限 3.0 m/s²，MINCO/tracker 3.5 m/s²；竖直速度 4.0、加速度 3.0。相对速度包含竖直分量，径向闭合速度单列。
- planner 最大时域 1.5 s，predictor 时域 4.0 s，terminal freeze 0.30 s；末端闭合速度候选 1.2–1.5 m/s。
- 水平终点速度仍为预测 USV 速度加沿接近方向的闭合速度，不强制固定 6.5 m/s 接触。当前轨迹和位置反馈仍可在末端要求减速，本次并未宣称“不减速接触”已解决。用户确认当前捕获效果后停止进一步控制改动。
- 图像/KF/规划关键输入 freshness=125 ms，采集戳、因果等待/超时拒绝、禁止外推的边界保留。未改 KF Q/R、未用真值做在线位置或 yaw 校正、未增大捕获半径。
- evaluator 保持三维 0.50 m 半径，以及捕获/海面接触事件顺序判定；没有新添相对速度或 dwell 成功门槛。评价 UAV 输入为原生 PX4 估计，USV 输入为模拟真值，不等同于独立 Gazebo UAV 真值距离。
- 最后 1 s 位置误差按 RGB-D 图像时刻、KF 自身 epoch，对原生 Gazebo USV 参考做最多 0.15 s 有界插值，禁止外推或拟合延迟。预测长于记录末尾的样本没有真值区间时不计入。
- 记录器退出后 /target/state 只有 intercept_evaluator_node 一个订阅者。其余在线链路使用视觉/KF/导航。

## 为什么 Y 后看起来分段

当前是准备接近 → 短时域 MINCO 末端的两种控制律。5 m 高度下直接落到接触高度通常不能在 1.5 s 时域内满足动态与海面约束，所以先移动到可达区域，再执行末端。第一轮在 Y+5.15 s 接入 MINCO，Y+6.45 s 捕获。

MINCO 内部采用分段多项式，并约束段间位置、速度、加速度连续；滚动重规划以旧参考的交接 P/V/A 为边界，不要求每段停顿。不过首次从准备控制切到 MINCO 时，准备速度指令与 UAV 实际速度不同；本轮对应指令变化约 0.7 m/s（准备约 3.8，首次 MINCO 约 4.5）。这与准备段的缓降收敛共同产生可见的分段感。本次按用户满意后收尾的要求保留该策略，没有为消除观感而扩大预测时域、取消可达约束或再改控制。

## 检查与证据

- 最终完整 pytest：829 passed / 1 skipped，两个已有 ament flake8 SelectableGroups 弃用警告，见 accepted_final_tests.log。
- 最终显式项目 flake8 使用仓库 .flake8 的 99 字符标准；diff 检查和两包增量构建结果见 accepted_final_flake8.log、accepted_final_diff_check.log、accepted_final_build.log。此前接口及三包构建证据仍保留。
- 主汇总：y_intercept_evidence_20261005/final_camera_origin_validation.json。
- 任务 1：modular_intercept_20261006_090027_518071_mission_1*。
- 任务 2：modular_intercept_20261006_090311_805571_mission_1*。
- 完整 JSONL：data/experiments/current/y_intercept_20261006_camera_origin_{1,2}.jsonl。
- 自动 Y 先要求 visible/locked/fresh FOLLOW，再要求估计跟随点误差≤1 m、相对水平速度≤1 m/s、高度 5±0.35 m、|Vz|≤0.35 m/s，以 0.5 s 检查周期保持约 5 s；这是验证入口准则，不给人工 Y 增设距离门。
- pause 证据：camera_origin_{1,2}_world_paused.log；真值订阅证据：camera_origin_2_truth_subscribers.log。
- 最新任务：modular_intercept_20261006_093236_821382_mission_1*；JSONL：data/experiments/current/y_intercept_20261006_stop_only_baseline_1.jsonl；汇总/暂停/订阅边界：final_stop_only_validation.json、stop_only_baseline_1_freeze_audit.json、stop_only_baseline_1_truth_subscribers.log。

## 追加试验与因果边界

| 追加阶段 | 原始 JSONL 后缀 | 结果 | 最小距离 m |
|---|---|---|---:|
| 增加原生暂停同步，保留原有截获配置 | complete_pause_1 | TIMEOUT | 1.597999 |
| 固定 6.5 m/s 接触速度与提前接纳 | cruise_contact_1 | TIMEOUT，未发布任何 MINCO | 2.632180 |
| 恢复原有截获配置，保留暂停同步 | stop_only_baseline_1 | SUCCESS | 0.478032 |

complete_pause_1 失败时尚未触发成功暂停，之后显式手动 pause 只用于检查目标冻结，不计作自动成功停止证据。cruise_contact_1 的候选违反水平/竖直动力学约束，尚未进入 MINCO，不能用它判断高接触速度轨迹的实际减速。不同任务的目标相位不同；失败前未执行冻结分支、恢复后捕获并自动共同停止，说明停止修复与成功截获兼容，但不提供严格同相位的单因素捕获率结论。

离线同刻审计 cruise_contact_1 的 Y+0–2 s：RGB-D 水平 RMSE 0.094863 m、KF 0.172921 m、1 s 预测 0.842034 m。预测误差显著放大，不能直接解释为固定延迟。长时域同输入反事实和 CA 模型回放仅作诊断，均未用于在线控制；未改 Q/R、未扩大 freshness、未使用真值在线修正。此前成功轨迹的末段参考速度约 5.5–5.7 m/s，实测约 6.3–6.6 m/s，解释了可见的减速指令。进一步消除该现象尚未验证，本次按用户满意后收尾的决定保留原策略。

本次仿真组件已关闭，原始数据和用户文件保留。后续启动仍使用 ./scripts/uav_lab.sh；当前工作区未自动提交。已验证的范围是红球感知接口下本场景的视觉截获，不代表真实无标记 USV、任意机动目标或任意环境均已验证；专门的 miss 恢复复验未完成。
