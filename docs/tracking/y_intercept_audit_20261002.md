# FOLLOW 后 Y 截获失败审计与逐项复验（2026-10-02）

基线：master / e4a32e4，开始时工作区干净。先只读审计用户指定的五个文件与 docs/tracking，再在未修改基线上重新采集完整预测、规划拒绝阶段及轨迹系数。本文记录的是当前模块化视觉控制链；红球感知仍是接口测试，不代表无标记 USV 感知已验证。

## Y 的实际状态机

Y 仅在 FOLLOW / TARGET_LOCK 且 locked 时被 mission_manager.handle_command 接受，置 intercept_requested，进入 FAR_GUIDANCE。FAR_GUIDANCE 保持约 5 m 高度并收敛到按垂直可达时间设置的准备位置。planner 的 success 不改变任务状态；tracker 通过 acquisition freshness、剩余执行时间、起点 P/V 连续性、目标端点和海面安全检查后，PLAN_ACCEPTED 才驱动 MINCO_READY → MINCO_TRACKING。剩余时间 ≤1 s 或目标距离 ≤2 m 进入 TERMINAL_MINCO。失锁进入 SAFE_RECOVERY / REACQUIRE；计划不可执行进入 PLAN_RECOVERY；评估器用真值单独发布 CAPTURE / FAILURE。

## 五个指定文件的作用

| 文件 | 当前运行角色 |
|---|---|
| intercept_planner_node.py | 接收 /planning/target_prediction 与 PX4 UAV 状态；单 worker 计算；四处 freshness 校验；发布候选、等待 tracker ACK 后提交 contact |
| finite_horizon_intercept_planner.py | 计算可达时间下界、搜索候选时长、生成接触几何和 MINCO P/V/A 边界、约束验证 |
| fast_minco_planner.py | 继承前者，最多六个实时候选，关闭在线几何 L-BFGS；输出 MINCO_T3_FAST；solver 75 ms、总预算 80 ms |
| minco_trajectory.py | 各段五次多项式，端点 P/V/A 与段间连续性线性求解，按局部时间采样 P/V/A/jerk |
| rolling_intercept_guidance.py | RollingReferenceFilter 仅被已退役的 trajectory_impact_sim 调用；不在当前模块化控制路径，不能拿它的 lookahead 解释本次失败 |

当前入口为 modular_intercept.launch.py → independent planner + tracker。相关源码位于 src/uav_control/uav_control/{guidance,tracking,control,mission}。

## 目标点、时间与约束

KF 输出 stamp 是投影状态的 epoch；source_stamp 是原图像采集时刻。Predictor 输入将两者分别映射为 state.stamp 和 observation_stamp。TargetPrediction.source_stamp 是预测零时刻，samples.relative_time 从该 epoch 起算；valid_until 始终是 acquisition +0.125 s，generated_stamp 不会使旧输入变新。

planner 对候选 T 查询 target_prediction.state_at_absolute_time(start_stamp + T)。PredictionSeries 查询内部做 absolute−source_epoch，再在线性样本间插值，不外推。目标 XY 就是此刻预测 XY；Z 按 sea_surface_z、contact_clearance / preferred_clearance 修正，并要求接触点仍在 0.50 m 目标球内。

T 并不是 distance/speed。FiniteHorizon 对水平、垂直运动、制动和响应延迟求 required_time，对移动终点作固定点迭代，然后在 required_time、minimum_duration 和可用预测 horizon 之间探测候选。Fast 分别尝试 closing speed 1.5 /0.3 m/s。首次允许时长 1–4 s，末段最低 0.30 s；已有 contact 优先保持，明确不可达时按现有策略延后。contact=start_stamp + selected_T，remaining=contact−当前时刻。最后剩余 ≤0.30 s 冻结新请求。

首条 MINCO 起点取实测 UAV P/V/A；重规划取已接纳旧样条在预计未来交接时刻的 P/V/A，保持参考连续。uav_source_stamp 仍保存实测状态时间，不拿未来边界伪装 freshness。终点 P=安全接触点；水平 V=预测目标 V + closing_speed×接近单位方向；Vz≈0（−1e−9）；A=目标预测 A。默认三段等分、target_curve_weight=0，XY 等同同边界条件 quintic。轨迹采样/消息传输离线误差约 1e−14，未发现轴序或端点时间混用。

发布只创建 pending contact；仅匹配 plan_id、PLAN_ACCEPTED 且 trajectory_replaced=true 的 tracker ACK 才提交。planner 计算与发布时均拒绝 acquisition age >0.125 s，比较新旧预测在同一绝对 contact 的位置，漂移 >0.50 m 时拒绝；没有以生成时刻重写图像采集时刻。

## 历史审计的边界

y_intercept_wrapup_20261001.md 的两轮均 TIMEOUT：最小距离 0.763384 /1.700030 m。首轮低于1.5 m时已锁定的视觉 yaw 被禁止，已修正；第二轮 yaw 非零但仍失锁，所以该修复不能算任务完成。第二轮首次年龄拒绝在 Y+0.046947 s，17次都早于失锁；晚到 Y+59.6468 s 的 PREDICTION_STALE 是恢复后的派生错误。ULog 参考−native pose 同刻误差约0.033–0.104 m，没有证据把 PX4 跟踪脱轨作为首因。

px4_navigation_fix_20261001.md 已修正仿真瞬时 GNSS 被设置110 ms延迟的问题。本次不重新把该已修错误归为当前截获主因。

## 新基线：确认预测导数被重复发布污染

新增基线 data/experiments/current/modular_intercept_20261002_091002_124528_mission_1*，以及完整只读 JSONL data/experiments/current/y_intercept_baseline_20261002.jsonl.gz。连续 locked FOLLOW 20 s后自动发送 Y，只依赖 mission/controller 的视觉状态，真值没有进入触发或控制。

结果：TIMEOUT，最小3D距离1.239395 m。21个planner事件，仅plan2成功并执行；12次 INPUT_AGE_AT_FINISH/PUBLISH、6次 TARGET_PREDICTION_SHIFT、2次 SEA_CLEARANCE，另1次成功。plan2的T=3.928904 s，其预测目标在contact时刻的水平真值误差1.968334 m（预测−真值 XY=1.725771,0.946600 m）。规划拒绝使该旧预测长期继续执行。

完整消息给出可复算的上游错误：KF对每个图像测量立即发布，也在20Hz timer重复投影并发布同一图像；PredictionEngine.update 原来每次都以 state.stamp 更新转率/转加速度，因此同一观测被当作新速度测量。重复发布的速度完全相同，模型得到伪零转率，随后下一新图像又在较短dt上估计导数，产生大幅转加速度抖动。

例如预测1248→1249：原采集时刻均为Y+0.432972 s，KF epoch从Y+0.4551到0.4698 s，速度均约(−0.2977,4.0228)m/s；仅重复发布就让 turn_rate 从0.097672变成0.073254rad/s、turn_acceleration 从+0.152801变成−0.177759rad/s²，4s末端XY从(82.272,19.964)变成(86.988,21.016)，移动约4.83m。该跳变不是新目标观测，也不是MINCO优化造成。

结论边界：确认的主要代码错误在 KF→BCTRA 观测时间语义；同时存在定时相位带来的年龄预算问题。不能把“存在主错误”写成“全部失败只有这一个原因”。先独立修复预测导数污染，再用完整Y任务决定下一项修改。

## 第一项修复与验证

PredictionEngine 仅在 observation_stamp 严格更新时学习水平速度导数，dt使用原始 acquisition 时间；重复KF投影仍更新 forecast origin epoch/P/V，保留原始 observation_stamp/valid_until。较新epoch不能恢复更旧图像；过期/非法新输入仍撤销缓存。未改变视觉、KF Q/R、轨迹增益、门限、截获半径或海面保护。

新增 test_prediction_observation_cadence.py：三个行为测试先失败，修复后通过；36项预测相关测试通过；全包748 passed、1 skipped、两个已有flake8入口弃用警告；四包构建通过。只读原始KF回放中同绝对3s接触点相邻预测漂移中位数2.807→0.333m；仍有残余噪声与模型误差，不能以此替代真实截获验证。

该项完整复验见下一节。

## 第一项完整复验未通过，继续修复年龄预算

仅修重复观测的完整仿真仍 TIMEOUT，最小距离3.887407 m；进入Y/准备的位置、曲率与基线不同，不能将两次最小距离当成匹配效果对比。完整消息保存在 y_intercept_unique_observation_20261002.jsonl.gz，显示最初已有 INPUT_VALIDATION/PREDICTION_STALE，部分请求开始年龄达0.134 s，随后混合年龄拒绝、目标预测漂移及 tracker PREDICTION_MISMATCH，周期性进入 PLAN_RECOVERY。这项实验否证了“只修重复观测就足以完成任务”。

继续查证调度：KF新图像立即发布 → predictor仅缓存等20Hz timer → planner仅缓存等5/10Hz timer。本机 rclpy SingleThreadedExecutor 同ready batch先处理timer、再subscription，允许多层反复取上一周期缓存。基线失败请求start age约0.11 s、成功约0.06 s，相差一个50ms帧周期；部分重规划完成很快，却等待旧参考的未来handover时刻再发布，进一步耗尽原125ms预算。

第二项修复针对同一 freshness 预算链：predictor接收新图像后立即生成，仅每个acquisition一次有效预测；重复KF投影不重复发布，timer仍显式发布过期/无效状态。planner在有效prediction回调启动，正常/terminal分别按实际worker-start冷却0.2/0.1s，不在旧timer相位启动缓存请求；忙时不排队未来P/V/A，实际dispatch时重新生成boundary/contact。原来的任务授权、pending-contact阻挡、最后0.30s freeze、四处age拒绝和tracker ACK提交边界全部保留。连续源仍是20Hz图像，控制仍只用视觉/KF。

新增回归测试先失败后通过：新KF立即预测、重复投影不增大发布频率、超时heartbeat保留原采集deadline、新prediction立即规划、5/10Hz worker上限、busy时不排旧boundary、timer不重用缓存和末段freeze。相关110项通过；完整测试/构建及新仿真见下一节。

## 调度复验：恢复更新，但末段仍失锁

755项测试通过，1 skipped，四包构建通过。完整调度修复复验仍 TIMEOUT，最小距离0.893773 m。19个planner事件没有再出现年龄/预测漂移拒绝（18次成功、1次 SEA_CLEARANCE），发布输入年龄约0.044–0.116 s；tracker实际替换8条（plan1–6、11、12），另外7次 STATE_POSITION_MISMATCH、3次 STATE_VELOCITY_MISMATCH。不得把planner success数误写为实际轨迹替换数。

同刻目标真值核对：初plan1 contact预测水平误差3.128 m；随更新缩小，plan11=0.708 m、最后已接纳plan12=0.543 m。随后改善到0.353 m的plan13被tracker速度门拒绝，旧plan12继续执行。输入与轨迹构造都在工作；现阶段剩余问题已转移到末段视觉可见性、物理交接与短时预测的耦合，不能把此前调度修复等同整个任务完成。

最后图像bearing增长到约0.865 rad，接近相机半视场0.87 rad，Y+3.76s进入SAFE_RECOVERY。当前yaw=vision_gain×image_bearing，目标转动时其稳态bearing误差需要约LOS_rate/gain；即使执行完美，视线转速达到0.8rad/s也会需要约0.8rad角误差，几乎耗尽视场。新鲜KF相对位置/速度诊断显示末段LOS速率与比例yaw命令不一致；这些latest快照只作为controller所用数据诊断，不冒充native同刻真值跟踪误差。

第三项最小控制修复：仅已请求Y、MINCO_READY/TRACKING/TERMINAL、locked且visible时，重新检查KF原始采集freshness后加入 NED XY LOS_rate=(rx*v_rel_y−ry*v_rel_x)/(rx²+ry²)，再叠加原图像反馈。最终仍clip原1rad/s上限；FOLLOW/搜索不启用；非法/过期/缺失KF不提供前馈；零水平距离避免除零；SAFE_RECOVERY和低高度禁止门不变。没有改变XYZ控制、Q/R、截获半径、sea guard或目标真值控制边界。其完整任务复验待追加。

## 起飞触发纠正、第五轮 Y 复验与恢复控制

第四轮未触发有效 X：单次临时 CLI 发布仅被 USV 接收，mission仍GROUND_HOLD；不把未起飞数据计作Y失败或相机证据。改用持久可靠publisher、双方订阅存在、真实flight_ready与GROUND_HOLD门、三次100ms X脉冲。第五轮已确认TAKEOFF与约5m高度，并在locked FOLLOW后收到Y。自动X过早消耗ready=true导致启动器后续ready等待超时，但host组件仍在运行；测试工具现延迟10s让启动器先观察就绪，生产启动器没有修改。

第五轮 y_intercept_los_auto_20261002.jsonl.gz 最终TIMEOUT，最小距离6.542395m，没有任何trajectory发布；最初可行plan1/2均candidate_published=false、HORIZONTAL_PREPARATION_NOT_READY。plan1准备位置/速度误差1.080/0.661m与m/s，plan2为0.575/1.432。Y+1.699s首次KF过期（visible仍true、locked=false）触发REACQUIRE，XYZ指令从约(3.163,3.052,0.022)m/s跳到零。之后失锁、停车与再加速导致相对距离增大；后续多次HORIZON_INSUFFICIENT。未进入MINCO，因此这轮不能评价第三项末段LOS前馈。

代码确认 _search_or_recovery_command 曾按intercept_requested切换策略：Y前使用受限search_velocity减速，Y后直接_hold当前位置且清空previous_velocity。修复只统一海面保护以上的无目标减速路径；Y不再允许突变停止。原freshness、清除过期轨迹、低高度/terminal爬升优先级不变。新增行为测试先复现Y=true返回POSITION错误，修复后三项与48项相关测试通过。全包766 passed、1 skipped；四包构建通过；完整任务复验待追加。

## 相机安装角建议（当前未调整）

前视模型安装pitch=0.20944rad（12°），RGB/深度640×480，水平FOV=1.74rad（99.69°）。按方形像素vFOV=2atan((480/640)tan(hFOV/2))=83.27°。水平机身、忽略小相机平移的海面视场：12°时覆盖地平线上方29.64°到下方53.64°；25°时为上方16.64°到下方66.64°。5m高度/5m水平距离目标俯角45°，其下缘余量约8.64°→21.64°；最近可见海面水平距离约3.68→2.16m。机身俯仰/横滚与实际外参会改变这些静态几何边界。

建议25°作为独立A/B候选，可减少天空占比、提高近距离海面目标留在图内的余量；不会自动提高同FOV/分辨率下像素分辨能力。必须同步SDF、定位/监测安装外参与诊断配置，并检验RGB方位是否仍代表body水平射线：当前image_target_bearing只取横向像素atan，较大下俯角下其与body水平azimuth有垂直像素耦合。当前down_tof_monitor仅评价，不具备在线定位切换权限。现有时间预算/控制拒绝不能归因于安装角，先独立验证恢复修复。

## 相机角度修改及独立几何证据

第六轮仍在12°下运行，记录240s窗口中FOLLOW频繁重锁，未达到测试工具的20s连续锁定门、未自动发送Y，因此没有Y结果；不能将其写成恢复修复成功或失败。原始数据保留在y_intercept_braking_20261002.jsonl.gz。

对第五轮Y−5到Y+6s，用该轮ULog /2026-10-02/01_54_13.ulg的native timestamp_sample P/完整q、vision.csv原始clock anchors和评价专用USV真值，在共同有效、无外推的212帧重建12°/25°投影，0帧因无时间包络拒绝。标注OFFLINE_GEOMETRY_PX4_ESTIMATED_POSE_HELD_FIXED，不能视为全独立姿态/位置真值。Y+1.850s原始帧IMAGE_INVALID，12°竖直角0.8285rad超半视場0.7267rad；25°反事实为0.6016rad、余量0.1251rad，仍可见。Y+1.699至3.7s共同37帧中原配置几何在图14帧、25°为21帧。整体212帧189→196；余下帧仍出界，说明25°可改善部分出画但不保证任务完成。脚本与逐帧JSON已归档。

按用户相机建议与此前自动选择推荐做法授权，将前视固定下俯改为25°（0.4363323129985824rad）；SDF、baseline与geometry诊断配置、localizer/monitor默认值同步，平移/FOV/分辨率/门限不变。TargetBearing.bearing保留图像atan横角；新增body_bearing从同图像横纵质心射线与固定安装角旋转计算body FRD水平角。远RGB接近用匹配同采集帧的body_bearing，yaw居中继续用图像bearing；不引入depth、UAV姿态或真值。无匹配新字段时保留原image方向兼容；非法或机身后向射线拒绝接近。四项新增行为回归先失败后通过，134项相关测试通过；全包770 passed、1 skipped，四包构建通过。新消息IDL需全套重建并重启节点，本轮已执行。

第七轮采用同一视觉授权边界，在连续locked FOLLOW达到5s后自动Y，参数显式--follow-seconds=5；该改变仅用于测试触发，不能用跨轮不同USV曲率/入场状态作匹配A/B成功率比较。完整仿真结果待追加。
