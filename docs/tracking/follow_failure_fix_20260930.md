# 2026-09-30 两次 X 后未跟随：日志归因与修复

以当前 HEAD `057775a`（9.30.1）及上轮未提交修改为基线；没有回退旧修改、提交或推送。本轮没有启动 Gazebo、PX4 SITL 或 ROS 运行图。用户两次原始 CSV/JSON/YAML/ULog 均保留。

## 结论与验证边界

两次没有平移跟随，是**有效视觉数据不足以及额外定时交付延迟**，不是 FOLLOW 控制器没有实现。第一轮重获 LOCK 后，KF 的真实 acquisition age 在下一控制周期超过 0.125 s，锁定立即被撤销。第二轮起飞结束后未取得有效锁定。根据用户后续确认，新增远距离 RGB 方位接近：不再等深度/KF 才允许向目标接近。

位置误差有很强的姿态归因证据：两次 PX4 估计 yaw 比 ULog 独立 groundtruth 高约 6–7.5°。同时发现 Gazebo 磁场/原生 PX4 磁传感器接口的确定坐标问题，已修复该物理接口。没有增加 yaw/Y 固定补偿，没有将真值用于在线校正，没有调整 EKF/KF Q/R。

**软件回归和编译已经通过；动态跟随与修复后精度尚未仿真验收。** 启动时会备份旧参数，并按零偏置模拟模型初始化已识别的模拟磁偏移；Gazebo/PX4 地磁模型差异及剩余估计误差尚待新日志核对，不宣称位置误差已经降至某个数值。

## 原始证据

| 项目 | 第一次 15:28:32 | 第二次 15:30:30 |
|---|---:|---:|
| 主 CSV 行数 | 1510 | 449 |
| 起飞结束，X 后秒数 | 9.165 | 8.758 |
| 早期最后有效深度观测，X 后秒数 | 7.405 | 7.347 |
| FOLLOW 行数 | 0 | 0 |
| 有效视觉观测/总数 | 154/653，23.58% | 58/211，27.49% |
| DEPTH_RATIO_LOW | 252 | 129 |
| IMAGE_INVALID | 234 | 22 |
| POSITION_TIMESTAMP_AFTER_HISTORY | 0 | 0 |
| 有效、同刻真值配对数 | 152 | 57 |
| 水平视觉 RMSE | 1.597 m | 1.492 m |
| 有符号 Y 均值 | +1.030 m | +1.066 m |
| acquisition→publish 中位数/P95 | 91.9/151.0 ms | 97.7/107.6 ms |
| receipt→processed 中位数/P95 | 34.9/96.3 ms | 27.8/77.8 ms |

这两次是新的误差总体，不能套用历史约 0.57 m、Y≈−0.52 m 的统计。

第一轮状态行数：TAKEOFF 184，ACQUIRE 1，REACQUIRE 848，SAFE_WAIT 417，LOCK 58，FAR_GUIDANCE 2。第二轮：TAKEOFF 176，ACQUIRE 1，REACQUIRE 272。第一轮约 X+57.615 s 首次进入 LOCK，X+57.665 s 已进入 REACQUIRE。记录中后续重复出现相同模式。日志中的 `inf` 表示严格筛选已拒绝过期状态；并不是物理采集戳突然变成无穷。

UAV 的 XY 总跨度分别为 (0.189, 0.061) m 和约厘米级漂移；实际未进入持续 FOLLOW 平移。ULog 的 XY 位置 setpoint 主要保持在固定搜索 anchor，未显示执行 FOLLOW 速度命令。用户看到“起飞、锁定、原地不跟”，与状态与命令记录一致。

CSV：`data/experiments/current/modular_intercept_20260930_152832_126022_mission_1*`、`...20260930_153030_026898_mission_1*`。

对应 ULog：

```text
/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-30/07_28_01.ulg
/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-30/07_30_10.ulg
```

`/home/qin/Projects/PX4-Autopilot` 解析到 `/data/PX4-Autopilot`；是同一源码/构建目录，不是另一份旧二进制。

## FOLLOW 交付延迟修复

旧流程在 RGB-D 配对后等待 20 Hz localizer timer，观测到达 KF 后又等待 20 Hz KF timer。两层排队额外消耗严格 freshness 的预算；LOCK 保留一个 50 ms 发布周期时，age 可从约 0.09 s 增至 0.14 s，超过 0.125 s，下一次控制直接撤销锁定，不能进入 FOLLOW。

本轮改动：

- `rgbd_target_localizer.py` 创建 ROS guard condition；配对图像到达、因果 pose/clock bracket 更新时唤醒 executor 定位。重图像计算仍在默认互斥 callback group，Gazebo Transport/DDS 回调只请求处理。原 timer 保留，用于超时与重试。
- `target_kalman_filter.py` 接受新的有效 measurement 后立即发布 projected state。周期 timer 继续发布投影；`source_stamp` 始终是原图像采集戳，内部 filter 时刻不因发布推进。
- 没有缩短 LOCK 确认周期、放宽 freshness、扩大 pose_wait_timeout、改 acquisition time、外推 pose 或改 P8.1 映射。

新增回归实际串联 KF、visibility、mission 和控制 timer，验证 LOCK→FOLLOW→非零平移，yaw 仍由 RGB bearing 控制。没有新图像且 age 超过 0.125 s 时，投影发布不能挽救陈旧数据，必须 REACQUIRE。另一回归验证缺少右侧姿态 bracket 时仍等待，pose 回调补齐 bracket 后通过 guard 定位，采集戳保持不变。

完整状态/Y/日志/SAFE_WAIT 行为保留上轮修复，见 [上轮报告](follow_run_logging_20260930.md)：X 启动 run writer，LOCK→FOLLOW；Y 才开始截获评价并进入 FAR_GUIDANCE；重捕获根据 Y 意图返回 FOLLOW/FAR；SAFE_WAIT 最终 yaw owner=HOLD、yaw_rate=0。evaluator results 不反馈给 mission/control。

## 量程与可见性限制

默认 X 同时启动 UAV 起飞和 4 m/s figure-eight USV。早期有效深度观测在 X+约 7.4 s 消失，而起飞到约 8.8–9.2 s 才完成。主记录显示目标距离随后可达约 63–88 m，超过 25 m 深度上限；FOV、深度有效比例不足也会拒绝观测。

这解释第二轮为何没有 LOCK，第一轮为何经历很长搜索。原日志没有完整原图/原深度，不能把全部 DEPTH_RATIO_LOW 唯一归为量程：精确 RGB-D 配对和有偏斜配对均发生该拒绝。本轮没有盲改配对门限、深度比例门限或摄像头量程。

交付延迟修复不会生成不可见目标的 3D 数据。新增独立 `follow_low_dynamic_diagnostics.yaml`，采用 +X 直线 0.5 m/s 并启用 evaluator geometry/UAV heading 诊断，其他控制/时间/锁定参数继承既有配置。**默认 baseline 的 4 m/s figure-eight 参数未修改。** 低速工况只用于分开验证，不能算作默认高速工况验收。默认与诊断配置均启用下面的方位接近功能。

## 新增：远距离 RGB 方位接近

用户已确认设计：远距离不需要精确位置，只需方位确保迅速接近；最高 6 m/s、约 5 m 高度、保留 0.125 s 图像 acquisition freshness。

在现有 `TARGET_ACQUIRE/REACQUIRE` 控制分支中增加 `BEARING_APPROACH` 子模式，不新增 mission enum 或消息接口。只使用 `/perception/front/target_bearing` 与 UAV 自身状态：

```text
X → TAKEOFF
  → ACQUIRE/REACQUIRE
      RGB 连续 3 个独立中心帧、age≤0.125 s
      无 fresh KF 位置
      → BEARING_APPROACH（保持高度、机头方向接近、RGB yaw 闭环）
  → fresh 3D/KF + stable lock → TARGET_LOCK → FOLLOW
  → Y → FAR_GUIDANCE → MINCO
```

- 已完成约 5 m 起飞且垂速稳定后才能前进；目标 bearing 必须在既有中心门限 ±0.15 rad 内。重复/乱序图像不能凑满 3 帧。
- desired 水平速度 6 m/s，同时受既有 guidance speed 上限限制；以当前实测速度初始化，按既有 3 m/s² 水平加速度限制上升。高度保持使用既有 altitude gain 与 sea safety guard；不下降到目标。
- RGB yaw 保持目标居中，方位接近中的 yaw owner=VISION；不由目标预测位置反算 yaw，不访问 USV 真值/路线。
- 新图像无效、偏离中心、真实采集 age 超限时即撤销前进指令，切换当前位置 anchor 的 POSITION 控制。实际制动受 PX4 动力学约束，不能把撤销指令称为瞬间物理停止。
- `SAFE_WAIT`、低高度/起飞、terminal/safe recovery、已接受 Y 的截获权限均禁止进入该子模式；该模式不报告 3D lock、不授予 Y 权限、不宣称 FAR_GUIDANCE available。
- fresh KF 出现时退出方位接近，由原锁定逻辑和 FOLLOW 接管。FOLLOW 仍只使用 fresh KF 位置，Y 仍要求当前视觉锁定。

配置：`trajectory_tracker_node.bearing_approach_enabled=true`、`bearing_approach_speed=6.0`。可设 false 回到严格原地采集策略。

日志主 CSV 增加 `bearing_approach_active`、`bearing_age_at_control`，controller status 明确记录 `BEARING_APPROACH`。后一字段是该控制决策时刻的 RGB source age；不会把它填入 KF age 或 3D observation age。summary 单列 `bearing_approach_duration`，接近运动不纳入 `search_xy_drift_max`。

新的方位控制接口仍使用红色验证目标检测器；此次不宣称已验证无标记非合作 USV 识别。

## 位置误差与磁场坐标证据

离线按 `rgb_raw_stamp` 查询 ULog `vehicle_attitude` 和 `vehicle_attitude_groundtruth`。Gazebo raw sim time 与 SITL native hrt 使用现有时钟恒等关系；没有从目标误差拟合时差。四元数使用同 reset epoch 的近邻 SLERP，无外推；位置使用 native sample 的近邻插值。

这些有效 production 帧没有保存在线四元数/body vector（geometry=false），所以使用 **ULog 姿态代理反事实**，并保留同一估计位置：

```text
body_proxy = R_ulog_estimated^-1 (vision_position - p_ulog_estimated)
counterfactual = p_ulog_estimated + R_ulog_groundtruth body_proxy - target_truth
```

| 同一组配对样本 | 第一次 n=152 | 第二次 n=57 |
|---|---:|---:|
| 原始水平 RMSE | 1.597 m | 1.492 m |
| 姿态代理反事实水平 RMSE | 0.136 m | 0.090 m |
| PX4−groundtruth yaw 中位数 | 5.94° | 7.47° |

这支持 body→NED 姿态项为主要来源；**不是精确恢复在线姿态，也不是修复后的仿真精度**。当前证据不足以逐层宣称 pixel/depth/camera/body 都已通过。新诊断配置用于补齐这些字段，剩余垂向误差（第二轮均值约 −0.183 m）不能由水平 yaw 归因消除。

仿真磁场接口有更具体的独立证据：

- 已安装 Gazebo sim8 8.15.0 默认 `use_earth_frame_ned=true`、`use_units_gauss=true`。其 legacy 地磁场 NED 分量进入 ENU 世界旋转；sensor 按 WorldPose 的逆旋转得到传感器 FLU 向量。PX4 原桥采用 `(-y,-x,z)` 兼容转换，而非当前 ENU 世界/FLU sensor 所需的 `(x,-y,-z)`。
- ULog 独立静态 groundtruth 旋转原始 `sensor_mag` 后，两次真 NED 磁偏角中位数约 **−2.574°/−2.569°**。当地 Gazebo legacy 表输出约 **+2.564°**；符号相反与坐标问题吻合。PX4 初始 `EKF2_MAG_DECL=+3.406°`，仅该物理符号错误就造成约 **5.97°** 航向不一致。
- 经过现有磁校准后的 `vehicle_magnetometer` 静态真 NED 偏角约 **−4.180°**；缓存 `CAL_MAG0_YOFF≈+0.006228 gauss` 等偏移进一步改变输入，使初始约 7.5° yaw 误差合理。此为旧日志证据，不自动等价于修复后残差。

依据：[Gazebo 8.15 磁系统源码](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.15.0/src/systems/magnetometer/Magnetometer.cc)、[Gazebo sensors8 磁传感器源码](https://raw.githubusercontent.com/gazebosim/gz-sensors/gz-sensors8/src/MagnetometerSensor.cc)。磁系统每次按地理模型更新 field，因此仅修改 ocean.sdf 的静态 magnetic_field 不能解决上述问题；没有做该无效修改。

### 已落地的物理接口修复

1. `prepare_gz_magnetometer_config.py` 复制原生 server.config，保留全部 13 个系统/渲染配置，将磁系统显式设为 `use_earth_frame_ned=false`、`use_units_gauss=true`。
2. 原生 PX4 GZBridge 在 `PX4_GZ_MAGNETOMETER_ENU=1` 时按传感器 **FLU→FRD `(x,-y,-z)`** 接入，单位仍为 gauss；未选择新模式时保留原 legacy 行为。可移植补丁和独立 C++ helper 保存于 `patches/px4/`。
3. `start_px4_ros2.sh` 在 Gazebo 和 PX4 子终端 source gz_env 后显式 export 同一 `GZ_SIM_SERVER_CONFIG_PATH`；PX4 子终端选择新 bridge 模式。启动前检查已编译 binary 的模式 marker，防止配置/桥只应用一半。R 清理创建的配置文件。
4. 在 rcS 导入参数后、replay gate 内、simulator/sensors/EKF 启动之前运行 `px4-rc.gzmag_enu`。仅当 ENU 模式开启、SIM_GZ_EN=1、槽 device ID=197388 时，将该槽 XYZOFF 初始化为零。模型只有零均值 Gaussian 噪声、无硬铁偏置；因此这按模拟模型初始化输入偏移，不是用视觉或真值估计补偿。其他设备、ID/priority/scale/rotation、EKF2_MAG_DECL/Q/R 均保持原值。
5. 启动前将存在的 `parameters.bson` / `parameters_backup.bson` 复制到工作区独立新目录 `data/experiments/px4_parameter_backups/enu_*`，不删除旧数据库。初始化日志打印命中槽及旧 XYZOFF。启动器同时检查实际运行 rootfs 中 hook 和 rcS 是否匹配；ROMFS 显式构建列表已注册脚本，tar 与 rootfs 文件均验证一致。

本机原生补丁已应用，`make px4_sitl_default` 构建成功；没有运行 gz 目标。该更改修的是仿真传感器输入坐标，未更改视觉 body→NED 数学、PX4 heading estimator、EKF 参数或在线真值边界。

本轮没有启动 PX4，所以**当前参数数据库尚未被该 hook 初始化**；下次用户启动显式 ENU 模式时才备份并初始化模拟设备偏移。Gazebo 表与 PX4 地磁模型约 0.84° 的差异仍存在。后续需用新 ULog 分开检查 raw sensor 和 calibrated magnetometer。若残差主要是 EKF/校准，则另行处理，不添加固定 yaw correction。

## 软件验证与证据文件

- `python3 -m pytest -q`：**703 passed, 1 skipped, 2 warnings**（依赖 deprecation warnings）。
- `python3 -m flake8 src/uav_control scripts`、`bash -n scripts/start_px4_ros2.sh`、`git diff --check`：通过。
- `colcon build --symlink-install --packages-select uav_usv_interfaces uav_control uav_usv_bringup`：三个包构建成功。
- 原生 `make px4_sitl_default`：成功编译 GZBridge、链接 PX4；header 与工作区 helper 完全一致，binary marker 存在。
- 配置生成检查：13 个原系统属性全部保留；磁系统 ENU/gauss 成对设置。
- 独立只读审查发现并关闭 ROMFS 未登记新 hook 的启动阻断；最终复核未发现剩余阻断。

原始统计、逐帧配对、输入 SHA256 与构建/测试输出：`follow_failure_evidence_20260930/`。主 CSV 的 `time` 是 elapsed；与 vision epoch 比较时使用原 summary 的 `run_started_at`，不推测时钟偏置。

复算第一轮（第二轮替换文件前缀和 ULog 为 07_30_10）：

```bash
cd /home/qin/data/uav_usv
python3 scripts/follow_failure_audit.py \
  --main data/experiments/current/modular_intercept_20260930_152832_126022_mission_1.csv \
  --vision data/experiments/current/modular_intercept_20260930_152832_126022_mission_1_vision.csv \
  --ulog /home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-30/07_28_01.ulg \
  --output /tmp/follow_failure_run1
```

迁移到未打补丁的 PX4 checkout 时，先 `git apply --check`，再应用 `patches/px4/gz_magnetometer_enu.patch` 与 `gz_magnetic_initialization.patch`；复制 `GzMagneticField.hpp` 至 `src/modules/simulation/gz_bridge/`、`px4-rc.gzmag_enu` 至 `ROMFS/px4fmu_common/init.d-posix/`，仅执行 `make px4_sitl_default` 编译。本机已完成，不必重复应用补丁。

## 用户下一次仿真

先关闭旧会话，再启动保持默认 4 m/s figure-eight 的诊断配置（此次由用户执行）：

```bash
cd /home/qin/data/uav_usv
UAV_USV_EXPERIMENT_CONFIG_FILE="$PWD/src/uav_usv_bringup/config/visual_geometry_diagnostics.yaml" \
  ./scripts/uav_lab.sh --no-build
```

先核对 runtime truth boundary：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv/install/setup.bash
ros2 topic info /target/state -v
ros2 topic info /fmu/in/trajectory_setpoint -v
```

`/target/state` 应仅供 evaluator/logger；Offboard setpoint publisher 应只有 trajectory_tracker。按 X，不按 Y，记录 TAKEOFF→ACQUIRE/REACQUIRE（有有效居中 RGB 时 `BEARING_APPROACH`）→LOCK→FOLLOW。接近中检查实际 XY 运动、bear age≤0.125 s、最高 6 m/s、高度约 5 m；FOLLOW 稳定后保持 20–30 s，检查约 5 m 跟随距离与 yaw_owner=VISION。确认丢失→REACQUIRE→重新 LOCK→FOLLOW 与 SAFE_WAIT yaw=0；无 Y 结束也应完整保存 ABORTED run artifacts。

若需将姿态/几何与动态接近分开验证，将上面配置文件替换为 `follow_low_dynamic_diagnostics.yaml`，运行 0.5 m/s 直线目标。稳定 FOLLOW 后才验证 Y→FAR_GUIDANCE→MINCO，不能将低速 X-only 验证算作高速截获成功。新 geometry/heading 日志应先验证 raw mag 磁偏角符号与层级残差，不能以离线反事实数值替代新实测。若 RGB 本身不可见，仍需搜索，方位接近不能生成不存在的目标方向。
