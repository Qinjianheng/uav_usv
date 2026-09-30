# FOLLOW、X 后日志、SAFE_WAIT 与视觉归因交付（2026-09-30）

以实际 HEAD `057775a`（`9.30.1`）为基线，开始时工作区干净。本轮就地修改，未回退、提交、推送或启动 Gazebo/PX4/飞行实验。代码与软件验证已完成；实际跟随动态、运行时话题边界和当前视觉误差根因仍需用户执行 Phase A–D。

## 1. 修改前状态机问题

核对实际代码后确认：FOLLOW enum、tracker FOLLOW 分支和 `FlightGuidanceCore._follow_velocity()` 均存在，但 manager 没有 LOCK→FOLLOW，Y 仅接受 LOCK。evaluator 原 `_start_mission()` 同时创建 writer 与开始截获评价，只在 `intercept_requested` 后调用。visibility 在计算搜索 rate 后可能只改 SAFE_WAIT 状态，保留搜索 yaw。

当前历史短运行水平 RMSE 为 0.566165690 m、signed Y mean 为 −0.519791127 m，有效 47/49（95.918%），`POSITION_TIMESTAMP_AFTER_HISTORY=0`。这些是历史记录复算，并非本轮新软件仿真结果。

## 2. FOLLOW 恢复方式

manager 在稳定锁定后保留一个发布周期 TARGET_LOCK，再按 `intercept_requested` 进入 FOLLOW 或 FAR_GUIDANCE。core 默认确认时间 0.05 s，node 按 `1/publication_rate_hz` 传入。本轮没有重写跟随控制器：仍用现有 `_follow_velocity()`，`follow_distance=5.0`、`follow_position_gain=0.8` 保持不变。

FOLLOW XYZ 使用严格 fresh `/tracking/target_state`，yaw 使用 `/perception/front/target_bearing`。测试让 KF 方位和 RGB bearing 方向相反，验证 XYZ 响应 KF、yaw 响应 RGB；拒绝过期状态后恢复搜索 anchor。未增加真值控制订阅。

## 3. 完整状态机与边界

```text
INIT --flight_ready--> GROUND_HOLD --X accepted--> TAKEOFF
  --> TARGET_ACQUIRE --视觉/KF锁定--> TARGET_LOCK（一个发布周期）
  --> FOLLOW --Y accepted且仍locked--> FAR_GUIDANCE
  --tracker接受可执行计划--> MINCO_READY --> MINCO_TRACKING --> TERMINAL_MINCO

FOLLOW/截获阶段 --确认视觉丢失--> REACQUIRE（固定XYZ anchor）
  --重新锁定--> TARGET_LOCK
      未按过Y：--> FOLLOW
      已按过Y：--> FAR_GUIDANCE

ACQUIRE/REACQUIRE --扫描完成或超时且不可见--> SAFE_WAIT（yaw=0）
  --新鲜RGB重新出现--> ACQUIRE/REACQUIRE --重新锁定--> TARGET_LOCK

既有规划恢复：MINCO链 --计划无效/耗尽--> PLAN_RECOVERY
  --新鲜可执行计划--> MINCO_READY
  --恢复超时--> SAFE_WAIT
既有安全恢复：安全决策 --> SAFE_RECOVERY --重新锁定--> TARGET_LOCK
R：新mission，GROUND_HOLD/INIT；显式Q任务命令：ABORTED。
```

标准路径是 LOCK→FOLLOW→Y。规划器成功本身不触发 MINCO，仍需 tracker 接受。CAPTURE/FAILURE enum 保留，但 evaluator 的结果不驱动这些在线状态。控制台 Q 只退出控制台，区别于显式 ROS Q 任务命令。

## 4. Y 语义

Y 正常在 `FOLLOW && target_locked` 接受，兼容短暂 `TARGET_LOCK && target_locked`。没有锁定时拒绝。SAFE_WAIT 即使几何可制导也不能在 Y 前自动进入 FAR_GUIDANCE。Y 设置既有 `intercept_requested`，不会重建 writer 或清空 X 后记录。

## 5. FOLLOW 丢失和重捕获

1–2 帧无效 RGB 只在既有 freshness 和 loss 条件允许时短暂继续；过期 KF/图像不能无限 coast。确认丢失后 REACQUIRE 固定 XYZ anchor、有限 yaw 搜索。重锁后短暂 LOCK，未请求 Y 回 FOLLOW，已请求 Y 回 FAR_GUIDANCE，不重新降级普通跟随。

## 6. SAFE_WAIT yaw 修复

scan finished 或 reacquire timeout 时，visibility 显式返回 `(SAFE_WAIT, locked=false, visible=false, yaw_rate=0, direction=0)`，并锁存扫描完成。最终 yaw 仲裁再保证不可见 SAFE_WAIT 为 `yaw_owner=HOLD`、`yawspeed=0`，即使上游意外给出 0.35 rad/s 也不扫描。新鲜可见 RGB 可重新进入 ACQUIRE/REACQUIRE；没有用 SAFE_WAIT 状态掩盖持续旋转。

## 7. 日志生命周期

`_start_run_artifacts()` 在 evaluator 收到新 mission 的 accepted TAKEOFF 时立即创建主 CSV、vision CSV、config snapshot；所有 X 后阶段均记录。phase transition 行单独标识，短暂 LOCK 不依赖定时采样碰巧命中。缺真值/缺 UAV 时仍记录阶段和可用诊断，评价量置为 unavailable/NaN。

`_start_intercept_evaluation()` 仅在同一 mission 的 Y 请求边沿调用 core.begin。`_finalize_run()` 在任务结束、新 mission/reset、正常 node shutdown 时强制冲刷 pending vision 并幂等落盘 summary。X-only 是 ABORTED，而非截获 FAILURE。旧任务的 mission/controller/planner/main prediction/shadow prediction 被隔离；同任务迟到的规划完成仍允许计数。

截获 SUCCESS/FAILURE 只发布一次并保存结果，run writer 继续记录到正常结束，便于检查后续控制；不会因评价结果暂停世界或结束在线任务。强制 SIGKILL/断电不能保证执行 shutdown/finalize。

## 8. 两个 timer 和统计含义

| 字段 | 起点和含义 |
|---|---|
| run_started_at / run_elapsed_time | evaluator 观察到新 accepted TAKEOFF 的 ROS callback time，X 后完整运行 |
| intercept_started_at / intercept_elapsed_time | evaluator 观察到 accepted Y 请求，只有此后开始 capture/sea/30 s 评价 |
| CSV time | 兼容列，现为 run_elapsed_time；旧按 Y 作零点的脚本应改用 intercept_elapsed_time |
| summary elapsed_time | 既有截获结果耗时；X-only 为0，完整运行耗时看 run_elapsed_time |
| run_metrics event_time_basis | evaluator ROS callback/sample timestamps；不是图像采集时间 |

任务事件计时有 ROS 传输/调度延迟，图像原 acquisition stamp 和 P8.1 映射不变。summary.run_metrics 记录 phase durations、takeoff/FOLLOW duration、first visible/lock、lock loss、reacquire count/完成时长/未完成时长、FOLLOW KF估计距离和同步真值评价距离分别统计、搜索期间实测 XY 最大漂移。搜索 drift 相对首个可用搜索位置样本，不是指令 anchor 误差。controller freshness 在缓存和每次采样均检查，过期诊断不重复污染 FOLLOW 统计。

## 9. X-only 日志测试

无 ROS runtime/Gazebo 初始化的 evaluator fixture 使用实际 writer/core/消息，验证：X 创建四类文件；TAKEOFF/ACQUIRE/LOCK/FOLLOW 短暂事件保留；未 Y shutdown 输出 ABORTED；60 s pre-Y 不消耗30 s截获限时；无 truth/UAV 仍有 CSV；R finalize 旧 run且不重复；迟到旧 mission 不重开日志；有效图像在缺真值超时后仍保留原戳；评价 TIMEOUT 后 run日志继续；诊断参考等待左右 bracket；旧规划/预测不进入新 run。`test_run_logging.py` 共15项通过。

## 10. Truth runtime boundary

源代码与边界回归测试确认：`/target/state` 为 evaluator/logger only；manager/tracker 无 evaluator result 控制订阅；新的 UAV model pose/helper 只有 evaluator 使用，没有向 controller/KF/BCTRA/MINCO 的 publisher。setpoint owner 仍是 trajectory_tracker。

本轮没有活动 runtime graph，实际 DDS 节点清单必须在 Phase A 核对，不能把源代码审计称为运行时已验证。evaluator 的 pause/freeze 路径未恢复。

## 11. P8.1 回归结果

`test_rgbd_target_localizer.py + test_p8_pose_time_audit.py + test_target_bearing.py`：**99 passed**。生产 localizer（含 GazeboImageClockMapper/causal timesync）、bearing、existing evaluator entity helper 与 HEAD 字节一致。没有扩大 pose_wait_timeout、receipt 当采集戳、固定 −0.21 s、关闭 SYNCT 或重写时间映射。

历史最新短运行 AFTER_HISTORY=0、有效率95.918%可复算；新软件运行时指标尚未测量。Phase D 先看 rejection histogram/分母，再解释精度，不能用历史数字保证新版本通过。

## 12. 视觉误差逐层归因

完整证据见 [分层诊断报告](/home/qin/data/uav_usv/docs/tracking/follow_bias_evidence_20260930/diagnostic_report.md) 与同目录四份 JSON。

| 层 | 当前约0.57m会话证据 | 解释限制 |
|---|---|---|
| RGB mask/projection | 49帧geometry关闭，无层残差 | 不能确认检测偏差 |
| depth/sphere camera center | 无独立层残差 | 不能确认深度/球心错误 |
| optical→body FLU | 无独立层残差 | 不能确认外参/轴映射错误 |
| body→NED / PX4姿态 | 无当前独立UAV heading | 当前根因 not_confirmed |
| 时间 | 历史最新47/49有效、AFTER_HISTORY=0 | 新版本运行待验证；不等于其他所有时钟误差均为0 |

较早 geometry 会话36个有限样本：pixel范数 RMSE28.009px、camera/body3D残差同为0.345081m。但 expected projection/camera/body 用 PX4 estimated pose，姿态误差会反映到这些层，不能据此独立证明 mask/depth/extrinsic 错误。历史 P7 static294帧 camera几何水平RMSE0.037792m、姿态贡献0.202179m，支持该旧静态工况的姿态主导，不迁移为当前−0.520m的确定解释。

新增 evaluator-only 独立 UAV quaternion：精确模型名 `x500_mono_cam_0`，native Gazebo clock映射、同采集时刻左右pose bracket、SO(3)插值；保存reference provenance/query元数据。记录 PX4/reference heading、wrapped delta_yaw、range*sin(delta_yaw)、actual lateral error、signed NED induced XY和姿态替换反事实。实体/姿态右 bracket 最多等待0.15s，仅限 evaluator；超时显式 unavailable，不外推。缺独立参考 analyzer 明确 `not_evaluated`。

## 13. 是否发现确定物理/坐标错误

**没有。** 当前数据不足以把0.566m定位到某个独立层，不能满足“当前主要产生于哪一层”的物理验收。新增诊断让下一轮获得所需证据，而非凭历史结果猜补偿。

Gazebo UAV model origin 与 PX4 local position参考点等价性未建立，字段明确 `MODEL_ORIGIN_FRAME_NOT_ESTABLISHED`，不把原始模型位置直接替换PX4 position。attitude-only counterfactual 保持PX4 position不变，标识 `PX4_POSITION_HELD_FIXED`，不能把剩余误差自动认定为检测误差。

## 14. 是否修改视觉生产代码

没有改生产 RGB-D、相机外参、目标半径/轴符号、KF Q/R、图像时间映射。没有固定Y/yaw/bearing补偿或在线truth校准。baseline YAML 未改；专用 `visual_geometry_diagnostics.yaml` 与 baseline 仅四项诊断差异：geometry=true、entity=true、UAV=true、显式UAV模型名。BCTRA/MINCO/tracking/FlightGuidance 参数和实现保持原样。

## 15. 单元测试和静态验证

按用户请求从仓库根目录运行 `python3 -m pytest -q`：**670 passed, 1 skipped, 2 warnings**；两项warning来自已有 flake8 entrypoint API弃用。裸 `python3 -m flake8 src/uav_control scripts`、`git diff --check`、启动脚本 `bash -n` 均通过。

新增 root `pytest.ini` 指定真实 package testpaths，避免重复扫描 build/install；ament style测试使用实际 source package绝对路径，使根目录与包目录调用一致。`.flake8` 将裸命令行长度对齐已有 ament99列标准，没有添加 lint忽略规则。独立审查复现并关闭四项日志/参考问题；审查原文和测试输出保存在 [验证证据目录](/home/qin/data/uav_usv/docs/tracking/follow_run_evidence_20260930)。

## 16. colcon build

```bash
cd /home/qin/data/uav_usv
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  uav_usv_interfaces uav_control uav_usv_bringup
```

最后一轮构建退出0，**3 packages finished [2min 31s]**。Python source用symlink install；未运行仿真。

## 17. 用户自行执行 Phase A–D

### Phase A：先做 X-only

```bash
cd /home/qin/data/uav_usv
./scripts/uav_lab.sh --no-build
```

另一终端先核对 runtime truth/setpoint boundary，再观察状态和诊断：

```bash
cd /home/qin/data/uav_usv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic info /target/state -v
ros2 topic info /fmu/in/trajectory_setpoint -v
ros2 topic info /simulation/impact/result -v
ros2 topic echo /mission/state
# 再开一个终端（同样source）观察：
ros2 topic echo /control/diagnostic
```

确认 truth接收者仅evaluator/明确logger，setpoint publisher仅tracker，没有result→manager/controller订阅。启动控制台按 X，预期 TAKEOFF→ACQUIRE→LOCK→FOLLOW，保持20–30 s，先不按Y。检查真实UAV XY平移、与USV相对运动、约5m跟随距离、VISION yaw、稳定lock、KF age和图像采集age。正常结束实验launch终端（Ctrl-C）或用启动控制台R清洁重启；Q只退出控制台，节点仍记录。

```bash
ls -lt /home/qin/data/uav_usv/data/experiments/current/ | head -n 16
```

确认同一run有主CSV、visionCSV、summaryJSON、config snapshot；summary intercept_started=false/outcome=ABORTED，CSV保留各阶段。正常finalization有 `Run finalized` 日志。

### Phase B：FOLLOW 丢失与重捕获

保持X-only，在用户可控条件下让目标离开FOV，再重新进入。继续观察上述两话题：FOLLOW→REACQUIRE→LOCK→FOLLOW；超时不可见时 SAFE_WAIT 的 search_direction=0、search_yaw_rate_command=0、yaw_owner=HOLD。检查实测search XY drift、loss count/reacquire duration；不能把yaw指令归零自动当成飞行器角速度瞬时归零。正常结束保留日志。

### Phase C：稳定 FOLLOW 后再 Y

R或关闭全部旧实验组件后开始干净会话，使用 Phase A启动命令。X→稳定FOLLOW→Y，预期 FAR_GUIDANCE→MINCO_READY→MINCO_TRACKING→TERMINAL_MINCO（按实际可执行计划条件）。比较Y前/后 phase与 run_elapsed_time/intercept_elapsed_time；30s限时从Y开始。评价SUCCESS/FAILURE后控制仍独立运行，结束时summary保存评价结果与run_end_reason。

### Phase D：独立几何/航向诊断

```bash
cd /home/qin/data/uav_usv
UAV_USV_EXPERIMENT_CONFIG_FILE=/home/qin/data/uav_usv/src/uav_usv_bringup/config/visual_geometry_diagnostics.yaml \
  ./scripts/uav_lab.sh --no-build
```

核对实际Gazebo UAV top-level模型名，重命名时只改实验副本中的 evaluator模型参数，不猜link、艇方向或offset。每次复用 Phase A runtime audit；先 X-only，避免复杂终端MINCO。baseline场景是4m/s figure-eight，静态/低动态归因应另复制场景配置。下面只创建临时静态场景副本，不改仓库baseline：

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
root = Path('/home/qin/data/uav_usv')
cfg = yaml.safe_load((root / 'src/uav_usv_bringup/config/visual_geometry_diagnostics.yaml').read_text())
p = cfg['moving_target']['ros__parameters']
p.update(trajectory_type='linear', horizontal_speed=0.0,
         velocity_x=0.0, velocity_y=0.0, velocity_z=0.0,
         vertical_oscillation_amplitude=0.0)
Path('/tmp/uav_usv_static_geometry.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
PY
UAV_USV_EXPERIMENT_CONFIG_FILE=/tmp/uav_usv_static_geometry.yaml \
  ./scripts/uav_lab.sh --no-build
```

每个场景用干净会话；从该临时副本调整 initial_x/initial_y 做近/中/远和画面左/中/右，记录实际 target_range/image bearing 而非把设定初值当测得距离。低动态 linear 场景可设 horizontal_speed=0.5、velocity_y=0.5（其余为0）；不同UAV heading应采用已知姿态/场景方法，保留实际独立reference，不改PX4估计器或加yawoffset。FOLLOW趋近约5m，距离分组须按实际样本分箱，过渡段单列。

```bash
python3 /home/qin/data/uav_usv/scripts/p8_visual_bias_analysis.py \
  --vision /home/qin/data/uav_usv/data/experiments/current/ACTUAL_RUN_vision.csv \
  --output /tmp/actual_run_bias.json
# 如用额外同NED/同acquisition戳独立UAV参考CSV：
python3 /home/qin/data/uav_usv/scripts/p8_visual_bias_analysis.py \
  --vision /home/qin/data/uav_usv/data/experiments/current/ACTUAL_RUN_vision.csv \
  --heading-reference /absolute/path/acquisition_aligned_uav_heading.csv \
  --output /tmp/actual_run_bias_external_heading.json
```

先看分母/valid rate/rejection histogram，再看同一批有限样本中的pixel、camera、body、NED signedXYZ、range、两套heading、range*sin(delta_yaw)与actual lateral residual。嵌入heading必须有效provenance和采集bracket。无参考时 not_evaluated，不能用USV bearing或PX4本身制造“独立参考”。若新有效率下降，先统计拒绝原因，禁止扩大P8.1等待门限。

## 18. 未解决项与交接

本轮软件层A/B/C已覆盖测试，D完成现有数据复算和可隔离诊断工具，**当前0.57m根因仍未确定**，没有生产物理修复。以下需新实验：5m FOLLOW动态性能、可见性/重捕获稳定性、SAFE_WAIT实际响应、runtime truth图、P8.1在线有效率/AFTER_HISTORY、独立UAV模型topic/name/quaternion可用性、模型与PX4位置参考点关系、不同距离/方位/航向误差分解。红球接口测试不等于无标记非合作USV检测已验证。

保留本轮未提交修改供用户审阅，HEAD仍为057775a；历史日志/data/videos/记忆文件未修改。先执行Phase A，不要直接X→Y。
