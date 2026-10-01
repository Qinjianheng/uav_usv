# FOLLOW 接近连续性与最新视觉误差复核（2026-10-01）

本轮在实际 HEAD `77872e9`（9.30.3）上继续，未回退到文档示例中的 `057775a`。完成控制修复与软件验证，未提交或推送。用户随后明确授权启动仿真，已完成五轮；最新运行验收见 [仿真验证报告](simulation_verified_fix_20261001.md)。

结论：9月30日16:48运行的抖动有确定控制根因：速度模式与位置保持反复切换，位置锚点导致高速刹停后的回拉，FOLLOW 首条速度命令还绕过了限加速度初始化。现已修复这些软件行为。随后完整修复版本已验证持续接近、连续55.5s FOLLOW和Y后进入MINCO；末端捕获尚未成功。本文旧运行数字注明日期，当前结果以仿真验证报告为准。

## 1. 最新基线与文档事实核对

文档 A/B/C 已在当前提交中实现：FOLLOW enum、tracker 分支及 `_follow_velocity()` 存在；manager 已有一个发布周期的 LOCK→FOLLOW；Y 已接受 FOLLOW/短暂 LOCK 且 locked；writer 已在 X 后 TAKEOFF 建立；SAFE_WAIT 不可见时 yaw 和 direction 已归零。它们不是本轮重新实现的功能。历史0.57m/-0.52m、95.9%不能作为当前动态运行的统计。

原交付可查 [FOLLOW、日志与 SAFE_WAIT 报告](follow_run_logging_20260930.md)，上一轮物理磁场/初始化与视觉交付可查 [跟随失败与磁场修复报告](follow_failure_fix_20260930.md)。历史报告保留原日期和原验收状态。

## 2. 9月30日修复前运行的确定问题

输入为 `data/experiments/current/modular_intercept_20260930_164821_621968_mission_1*` 和原生 ULog `08_48_06.ulg`；[复算脚本](approach_jitter_evidence_20261001/audit.py)、[结果及输入 SHA256](approach_jitter_evidence_20261001/audit.json) 都保留。只用已记录服务器时钟锚点和 native sample time，不拟合延迟、不外推、不修改原日志。

| 指标 | 修复前已记录运行 |
|---|---:|
| 完整运行 | 74.647 s |
| FOLLOW 事件积分 | 1.134 s |
| RGB 接近事件积分 | 24.434 s |
| RGB 接近分段 | 42段，中位0.400 s |
| 原生 offboard 模式切换 | 77次，含起飞 |
| 最大内部 XY 速度设定值跳变 | 6.552 m/s / 0.096 s |
| 锁定丢失 | 17次 |
| 截获请求是否被接受 | false |

最大跳变发生在运行约29.777s：内部速度从 `[5.948,-0.789]` 跳到 `[-0.542,0.108]` m/s。三个高速→位置保持段的原生 UAV groundtruth 速度约0.95–1.05s后沿原前进方向变负，段内达到约−2.33/−2.45/−2.15m/s；离固定锚点的最大位移约3.06/3.44/2.86m。不是只有 EKF 噪声或画面抖动。

ULog 模式约10Hz记录，可能漏掉单个20Hz控制周期；CSV 分段积分24.253s与 evaluator 事件积分24.434s的时间基准不同。不能混用这些分母。

## 3. FOLLOW 恢复与本轮连续性修复

继续复用既有 `_follow_velocity()`、`follow_distance=5.0`、`follow_position_gain=0.8`，FOLLOW XYZ 仍只来自严格 fresh `/tracking/target_state`，yaw 仍由 RGB bearing 仲裁。

本轮修复：首次 fresh KF 到达和短暂 TARGET_LOCK 确认不再自动停止 RGB 接近。真正进入 locked FOLLOW 时才交给既有 FOLLOW 分支。FOLLOW 的 KF 过期但独立 RGB 仍满足0.125s、前向有效、3个唯一方向帧等条件时，可执行 RGB 接近；此时 `target_locked=false`、fresh target cache为空，不能接受 Y、不能进入 MINCO。

RGB接近沿自身heading + 新鲜RGB bearing运动，速度6*cos(bearing)，粗方向帧与居中锁定帧分别统计；粗方向不授予FOLLOW/Y锁定。RGB 接近、FOLLOW 与短时刹车保持速度模式和同一份 shaping history，不再每段从实测速率重置指令。FOLLOW/FAR_GUIDANCE 首次初始化从自身实测速度开始，避免从0直接发6.2m/s。跨安全恢复的控制 owner 交接显式清理旧速度缓存，返回跟随时重新从实际速度起步。

## 4. 完整状态机

```text
INIT → GROUND_HOLD --X accepted→ TAKEOFF → TARGET_ACQUIRE
  --视觉/KF锁定→ TARGET_LOCK（一个发布周期）
  --Y未请求→ FOLLOW --Y accepted且locked→ FAR_GUIDANCE
  --tracker接受可执行计划→ MINCO_READY → MINCO_TRACKING → TERMINAL_MINCO

FOLLOW/截获阶段 --锁定丢失→ REACQUIRE
  --重新锁定→ TARGET_LOCK
      Y未请求：FOLLOW
      Y已接受：FAR_GUIDANCE

不可见搜索超时/扫描结束 → SAFE_WAIT（yaw=0，direction=0）
新鲜RGB重新出现 → ACQUIRE/REACQUIRE → 重新锁定
终端丢失/低空不安全 → SAFE_RECOVERY → 先恢复安全高度
计划无效 → PLAN_RECOVERY → 新可执行计划或SAFE_WAIT
R → 新mission的GROUND_HOLD/INIT；显式任务Q → ABORTED
```

`BEARING_APPROACH`、`VISUAL_BRAKING` 是控制诊断 status，不是新增 MissionState enum。它们不能伪造 TARGET_LOCK/FOLLOW 状态；状态机仍依据 `search_state`、`target_locked` 和既有安全条件。

## 5. Y 的语义与本次“无反应”

Y 判断的是锁定和任务阶段，并不是必须先达到某个接近距离。用户确认本次在未跟随、未接近时按Y，summary确认 `intercept_started_at=null`；符合未锁定拒绝规则。日志无法恢复精确按键时刻，不能宣称知道该瞬间所有状态。

保留 `FOLLOW + locked`，兼容 `TARGET_LOCK + locked`；不接受未锁定 Y，不缓存提前按下的 Y。拒绝日志现在明确打印当前 phase、target_locked，并提示等 FOLLOW/锁定后重新按Y。Y被接受后仍设置 `intercept_requested`、进入 FAR_GUIDANCE；不重建 writer。

## 6. 丢失、刹车与固定搜索锚点

1–2帧 RGB 无效且 KF仍fresh时，保留已有短时 FOLLOW 规则。KF超过0.125s立即撤销3D锁定；若独立RGB有效则用RGB接近，否则撤销目标运动权限并刹车。

Y前搜索不再立即把高速时当前位置作为位置保持目标。先在速度模式中将目标 XY 速度设为0，沿现有3m/s² command shaping减速；指令速度和自身实测水平速度都≤0.1m/s后才捕获固定自身 XYZ anchor，再用有速度/加速度限制的自身位置反馈保持该锚点。锚点不会随漂移刷新。

失去目标后出现有限刹车位移是物理制动，不能当作继续使用过期目标追赶。6m/s在理想3m/s²制动下需要约2s/6m；该数值不是PX4实测停距保证。终端和已请求Y后的安全/搜索路径保留原保护。

## 7. SAFE_WAIT yaw 与 yaw ownership

基线已有不可见 SAFE_WAIT 的 yaw_rate=0、direction=0和最终 HOLD 仲裁，本轮回归继续通过。刹车可以在 SAFE_WAIT 任务阶段完成，但不会恢复无限 yaw 扫描。新鲜 RGB 可见时，按原规则退出等待并对准图像；locked FOLLOW 或 BEARING_APPROACH 为 VISION，搜索为 SEARCH，恢复/不可见 SAFE_WAIT为HOLD。XYZ锚点控制不使用目标 truth 或KF反算yaw。

## 8. 日志生命周期

基线 `_start_run_artifacts()` 在新 accepted TAKEOFF建立主CSV、visionCSV、config snapshot；`_start_intercept_evaluation()`在accepted Y请求时单独begin；正常shutdown/R/任务结束幂等finalize summary。X-only正常终止为ABORTED，保留所有阶段与失锁记录。本次新日志实际已具备四类文件，证明不需要先Y才能记录。

本轮增加 `VISUAL_BRAKING` 诊断和 summary `visual_braking_duration`；有限制动位移不计入停稳后的 `search_xy_drift_max`。字段 `bearing_approach_active`/bearing age仅表达RGB权限，不能替代真实KF age。

## 9. run timer、intercept timer与X-only测试

run elapsed从 evaluator观察到X→TAKEOFF计时，intercept elapsed从accepted Y计时；X到Y之间不消耗30s截获评价限时。既有15项 `test_run_logging.py` 覆盖X后立即建文件、X→FOLLOW→shutdown、X-only无truth仍落盘、Y只开始截获计时、R/迟到旧mission隔离、finalize幂等、评价后日志继续。

## 10. truth boundary

`/target/state`仍仅evaluator/logger；独立Gazebo UAV/USV参考仅评价与本离线脚本读取。controller/KF/BCTRA/MINCO未新增这些订阅，evaluator结果不驱动任务状态，不暂停Gazebo或冻结目标。

授权后已做实际runtime查询：/target/state仅evaluator订阅，/fmu/in/trajectory_setpoint仅trajectory_tracker_node发布。重跑时仍可按Phase A核对。

## 11. P8.1 与有效率

P8.1时间映射及KF算法本轮未修改；生产RGB-D/bearing图像QoS改为有界可靠传输，RGB-D配对收紧到1ms以排除相邻帧混用。未扩大pose_wait_timeout、重写采集戳、关闭SYNCT、添加固定延迟。P8.1相关测试包含在全量回归中。

9月30日16:48日志 `POSITION_TIMESTAMP_AFTER_HISTORY=0`，但有效观测是248/1114（22.262%），显著低于历史短近距离95.9%。拒绝直方图为DEPTH_RATIO_LOW460、IMAGE_INVALID353、DEPTH_MAD_HIGH32、DEPTH_FRAME_UNMATCHED16、RGB_FRAME_UNMATCHED4、POSITION_TIMESTAMP_BEFORE_HISTORY1。远距离超过25m深度范围、失去有效图像及混配问题需分别分析；不能把所有拒绝误诊为pose时钟失败。

有效帧采集→发布中位29.5ms、P95 82.3ms、最大130.9ms；receipt→processed中位6.09ms。17次锁丢时RGB都仍visible且lost_frames=0，KF age为129.4–134.1ms，越过125ms门限；本轮处理过期后的控制交接，没有放宽门限。

## 12. 9月30日视觉误差逐层归因及最新补充

完整输出见 [既有分析器复算](approach_jitter_evidence_20261001/visual_bias.json) 与 [精确native分解](approach_jitter_evidence_20261001/audit.json)。该旧运行247个valid+truth有限样本水平RMSE0.712458m、signed X mean−0.040857m、signed Y mean+0.123908m；历史−0.52m不应再固定补偿。

| 层 | 当前证据及限制 |
|---|---|
| RGB mask/projection | 248帧projection残差范数RMSE5.937px；expected值含PX4 pose，不独立证明像素检测错误 |
| depth/sphere center | camera空间3D残差RMSE0.748341m，大离群已进入此层 |
| optical→body FLU | body空间3D残差同为0.748341m，旋转保持范数；未发现新增轴符号/外参错误 |
| body→NED/heading | 247帧同采集时刻独立heading RMS约0.609°；精确姿态替换后水平RMSE仍0.685940m，当前大离群不由yaw主导 |
| 姿态时间贡献 | 原生同图像时刻精确分解：timing水平RMSE0.001362m、estimation0.194978m、GT-model reference0.000143m；分解闭合误差≤1.52e−14m，组件RMSE不能相加 |
| RGB/depth配对 | 有效248帧中同戳158、相差48/52ms有84、相差100ms有6；247真值样本中>1m离群10个，9个异戳 |

共同38–43.2s窗口，同戳29帧水平RMSE0.337073m，异戳17帧2.061997m；精确姿态替换后分别0.330408/2.071036m。全程同戳157个真值样本0.300556m，异戳90个1.111526m。分组仍存在不同视角/距离选择影响，共同窗口也不是同一批图像反事实。

最大的5.694m离群在约14.569s，RGB/depth差48ms、记录depth median24.809m。代码配对器会立即选择当前队列中skew≤0.1s的最近一对；仅从软件语义可确认异时配对被允许，结合离群相关性值得下一步检查。但原图未保存，不能证明该像素到底采到了船体、背景还是另一帧球面，因此本轮不宣布RGB-D像素根因闭合。

## 13. 已发现的物理错误与本轮是否修视觉生产代码

上一轮已修Gazebo magnetic ENU/FRD映射与该无偏仿真磁设备的旧校准初始化；当前原生ULog证实CAL_MAG0_ID=197388，OFF XYZ全0，EKF2_MAG_DECL仍3.405765°。当前独立yaw结果支持该修复已实际生效，不是新增固定yaw补偿。

本轮没有再修改磁参数、相机外参、球半径、坐标轴或KF Q/R。授权仿真后确认同传感器相邻帧被立即混配，已收紧RGB-D配对，并修复图像可靠传输与数值线程调度。最新几何实验2258帧原始水平RMSE0.490m，自身PX4位置误差贡献0.445m；离线同时替换位置/姿态后剩余0.080m，稳定窗口0.062m。不能把pose误差混进外参归因。详见仿真报告；没有真值在线校准、固定Y/yaw/bearing补偿。

## 14. 新测试与修改范围

新增 `test_approach_continuity.py` 16个回归，最初10项为：freshKF/LOCK确认期间保持RGB运动（2项）、过期KF独立RGB回退且不伪锁、移动时丢失平滑刹车不捕获旧anchor、停稳才建anchor且不刷新、FOLLOW首命令限加速度、RGB恢复保留上一指令、制动统计不冒充搜索漂移、恢复后从实际速度重新接管、SAFE_WAIT重锁过渡时offboard模式与setpoint一致。另6项覆盖非居中粗方向、居中阈值跨越连续性、正负视线指令、失效/迟到/重复帧和粗方向不得伪锁；均先观察失败，再实现修复。

独立审查指出SAFE_WAIT刚重锁、mission callback尚未更新时会进入NO_VALID_PLAN分支；该分支原先固定position owner，却收到新的velocity setpoint。已用真实timer fixture复现并修复为按最终command.mode发布owner，传入实际控制dt；没有改变MINCO的正常控制模式。

更新既有bearing/FOLLOW/strict visual/delivery/guidance测试，使其验证有界速度保持与撤销目标权限，保留固定anchor、freshness、truth与安全断言。没有修改BCTRA/MINCO算法、权重或capture radius。

## 15. 静态与软件验证

最终全量 **722 passed、1 skipped**，两个已知flake8插件API弃用warning；另增加同次配对、BLAS线程、可靠图像交付3项回归。`python3 -m pytest -q` 的输出见 [pytest.log](approach_jitter_evidence_20261001/pytest.log)。flake8检查source/scripts及本复算脚本；`git diff --check`检查差异。本文第2及第12节旧数字来自明确标注的9月30日运行；新软件成绩及同刻pose分解见仿真验证报告，不混用样本。

## 16. colcon构建

```bash
cd /home/qin/data/uav_usv
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  uav_usv_interfaces uav_control uav_usv_bringup
```

最终三包构建结果见 [colcon.log](approach_jitter_evidence_20261001/colcon.log)。此命令不启动仿真；native PX4源码本轮未修改。

## 17. 用户自行执行Phase A–D

### Phase A：X-only连续接近与FOLLOW

```bash
cd /home/qin/data/uav_usv
./scripts/uav_lab.sh --no-build
```

另一终端核对：

```bash
cd /home/qin/data/uav_usv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic info /target/state -v
ros2 topic info /fmu/in/trajectory_setpoint -v
ros2 topic info /simulation/impact/result -v
ros2 topic echo /mission/state
# 再开同样source的终端：
ros2 topic echo /control/diagnostic
```

先按X，不提前按Y。远处可先BEARING_APPROACH，随后ACQUIRE→LOCK→FOLLOW。观察XYZ真正平移、指令不反复速度/位置切换；锁定确认不强制刹停；KF过期立即失锁，但fresh RGB仍能接近；RGB也无效时VISUAL_BRAKING，停稳后固定搜索anchor。尽量保持FOLLOW20–30s，验证约5m距离、VISION yaw、真实运动及KF age。正常Ctrl-C结束实验节点，检查四类日志；X-only应ABORTED且intercept_started=false。

### Phase B：丢失和重捕获

在X-only FOLLOW中令目标离开FOV再重现。观察FOLLOW→REACQUIRE→LOCK→FOLLOW；失去有效RGB后平滑刹车，停稳anchor固定；不可见SAFE_WAIT yaw/direction=0、owner=HOLD。注意制动位移和停稳搜索漂移分别统计。

### Phase C：锁定FOLLOW后Y

干净会话X→稳定FOLLOW、target_locked=true→再按Y。预期Y accepted→FAR_GUIDANCE→tracker实际接受计划→MINCO_READY→MINCO_TRACKING；后续终端由现有可执行性/安全条件决定。提前按Y被拒绝后必须重新按，不能只等旧按键自动生效。核对Y前后两个timer及评价只记录的边界。

### Phase D：低动态独立几何归因

```bash
cd /home/qin/data/uav_usv
UAV_USV_EXPERIMENT_CONFIG_FILE=/home/qin/data/uav_usv/src/uav_usv_bringup/config/follow_low_dynamic_diagnostics.yaml \
  ./scripts/uav_lab.sh --no-build
```

此专用配置是+X线性0.5m/s、初始8m，启用独立几何/UAV诊断；baseline仍4m/s figure-eight。先X-only，不先跑终端MINCO。不同距离/方位/heading用该文件的实验副本改场景初值，按实际测得range/bearing分组。静态实验副本将目标速度和heave设0，不改baseline。原始图像若需验证混配像素因果，应在用户仿真时另保存双流原图及采集戳，当前CSV不能追溯像素。

```bash
python3 /home/qin/data/uav_usv/scripts/p8_visual_bias_analysis.py \
  --vision /home/qin/data/uav_usv/data/experiments/current/ACTUAL_RUN_vision.csv \
  --output /tmp/actual_run_visual_bias.json
```

`ACTUAL_RUN`替换实际新run前缀。独立嵌入heading/provenance无效时应not_evaluated；分母、拒绝直方图、同戳/异戳和稳定窗口先于全程RMSE解释。不能用USV bearing当UAV独立航向。

## 18. 已完成与尚未完成的运行验收

已完成五轮仿真、实际DDS边界查询、连续55.5s locked FOLLOW、约5m水平跟随距离、Y接受并进入MINCO_TRACKING/TERMINAL_MINCO、X-only ABORTED四类日志，以及同刻独立姿态/位置分解。最新POSITION_TIMESTAMP_AFTER_HISTORY=0，有效率94.956%。

末端目标离开前视相机后触发安全恢复，最小三维距离0.929m，最终TIMEOUT；未宣称捕获成功。后续应独立处理前视/下视生产视觉交接及自身导航精度，不放宽过期权限或capture radius。红球仅验证感知接口，不构成无标记非合作USV检测验收。

当前工作区保留修改供审阅，HEAD仍77872e9；原日志、data/videos及记忆未改。未手工编辑参数数据库，标准启动器每轮保存参数备份并按既有流程运行；自建仿真组件已清理。详细当前数据见 [仿真验证报告](simulation_verified_fix_20261001.md)。
