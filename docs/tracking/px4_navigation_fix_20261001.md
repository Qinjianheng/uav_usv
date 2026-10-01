# PX4 Gazebo GNSS 导航时间配置修复（2026-10-01）

## 结论与范围

根因已通过本地 PX4 源码和原始 ULog 确认：Gazebo NavSat 没有接收机延迟，GZBridge 将 sensor_gps.timestamp_sample 设为当前 timestamp；VehicleGPSPosition 在这两个时间相等时仍应用默认 SENS_GPS0_DELAY=110 ms，将当前 GNSS 位置回拨到 110 ms 以前。EKF 因此把测量融合到错误的运动时刻，输出位置主要沿速度方向偏前。

修复采用当前模拟传感器的零接收机延迟配置。没有改 EKF/KF Q/R、视觉坐标、图像采集戳、0.125 s freshness、capture radius 或 BCTRA/MINCO。Gazebo 真值仅用于离线评价和原有 evaluator。

## 因果证据

基线原始文件：data/experiments/current/modular_intercept_20261001_102705_450931_mission_1；ULog：/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-01/02_26_03.ulg。

- 原始 sensor_gps：timestamp_sample=timestamp；融合输入 vehicle_gps_position：5487 条样本全部被回拨 110 ms。
- 同一组 3256 个运动样本（真值水平速度 >3 m/s）：以原始接收时刻比较 GNSS 位置，水平 RMSE 0.061929 m；以回拨后的 timestamp_sample 比较，同刻误差变为 0.486078 m。
- EKF 原生运动段位置水平 RMSE 0.462487 m；沿运动方向的平均误差 +0.450620 m。静止段水平 RMSE 0.022525 m。错误随运动产生，不能用固定 XYZ 偏置解释。
- 本地 VehicleGPSPosition.cpp 166–170 行负责回拨；module.yaml 定义默认 110 ms；EKF2.cpp 的 UpdateGpsSample 使用已校正 timestamp_sample 进行融合。

这些是同刻离线比较，没有按目标误差拟合延迟或给控制器送入真值。

## 实现

1. patches/px4/px4-rc.gzgnss：仅 PX4_GZ_GNSS_NO_DELAY=1、SIM_GZ_EN=1 和 SIM_GZ_EN_GPS=1 时运行。识别模拟 GNSS 设备 11469068 的接收机槽；两个配置 ID 均为 0 时遵循 PX4 instance-0 fallback。仅将匹配槽 SENS_GPS*_DELAY 设为 0；未知配置或写入失败阻止启动。
2. patches/px4/gz_gnss_initialization.patch：在 simulator/sensors/EKF 启动之前调用 hook，并注册到 ROMFS。
3. scripts/start_px4_ros2.sh：启动前校验工作区与已构建 hook 一致，并启用该 Gazebo profile。原有参数 BSON 备份保留。
4. scripts/px4_navigation_audit.py：可复算的原生时间导航审计。禁止外推、拒绝非递增源时间和 >100 ms 插值区间；GNSS 两种时间比较使用共同有效样本。

本机原生 PX4 已安装并重建成功。此前 ENU 磁场补丁和其他源码改动完整保留；本次修改前的原生 rcS、CMakeLists.txt、已有 diff 和二进制 hash 位于 px4_navigation_evidence_20261001/native_before/。

该配置写入本机 SITL 的 rootfs 参数数据库，启动前备份目录见 data/experiments/px4_parameter_backups/enu_Sk9j8x/ 与 enu_1rNMDA/；未连接或修改物理飞控。离开此无延迟模拟 GNSS 模型时，应按所选传感器的实际延迟重新配置参数。

## 验证结果

所有指标均为水平 RMSE，单位 m。原生运动段以真值水平速度 >3 m/s 选样；图像同刻项采用有效配对观察。

| 指标 | 修复前 | 重启复验 1 | 重启复验 2 |
|---|---:|---:|---:|
| 原生 EKF 运动段，保留实际局部原点 | 0.462487 | 0.090872 | 0.089247 |
| 原生 EKF 运动段，离线统一原点 | 0.462496 | 0.064467 | 0.067071 |
| 图像同刻自身导航位置，全程有效配对 | 0.444953 | 0.092033 | 0.085730 |
| 图像同刻目标位置，全程有效配对 | 0.490006 | 0.154002 | 0.089607 |
| 固定 45–90 s 窗口自身导航位置 | 0.432874 | 0.089800 | 0.089385 |
| 固定 45–90 s 窗口目标位置 | 0.432025 | 0.156423 | 0.078868 |

两轮修复后融合输入所有记录都没有 110 ms 回拨。两个重启复验的 GNSS 原始噪声、EKF Q/R 与输出时间常数配置保持原值。原生运动段导航误差分别下降约 80.4% 和 80.7%。

全程、过渡 0–20 s 和固定 45–90 s 统计保存于 comparison.json。各轮传感器随机噪声及局部原点不同，不能把两次目标 RMSE 的差别当作新调参效果。图像同刻分解闭合误差均 <1.6e-14 m。

![同刻导航与目标误差曲线](/home/qin/data/uav_usv/docs/tracking/px4_navigation_evidence_20261001/navigation_comparison.png)



局部坐标原点差异必须单列。第一轮 EKF 与仿真本地参考原点差约 [0.063938, -0.012500, 0.003444] m；离线统一原点后，运动段导航水平 RMSE 为 0.064467 m。统一原点只用于审计，没有向在线控制添加原点校正。

两次复验分别持续 111.457 s、118.960 s（精确时长见 comparison.json），最长连续 locked FOLLOW 分别 34.850 s、40.783 s。仍存在严格 freshness 触发的丢锁/再捕获，不能声称全程无丢锁；导航误差修复已复现。

测试覆盖不同 simulator profile、设备槽匹配、未知接收机、参数写失败、ROMFS 调用顺序、禁止外推/时钟回退和已知运动下时间错误的方向。最终全包 735 passed、1 skipped，两个既有 flake8 入口 API 废弃警告；3 个 ROS 包 colcon build 通过；本次文件 flake8、shell 语法、原生 patch 反向检查和 git diff --check 通过。两轮测试启动的 Gazebo/PX4/DDS/experiment 进程均已退出。独立审查未发现严重或重要问题。

本任务仅验证导航修复和其对视觉位置误差的影响。X-only 仿真停止产生的 ABORTED/NODE_SHUTDOWN 不代表截获失败；没有发 Y 或声称命中成功。末段制导和真实无标记 USV 检测不属于本次验证结论。

## 复算与再次启动

在工作区运行：

```bash
python3 scripts/px4_navigation_audit.py \
  --ulog /home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-01/02_26_03.ulg \
  --output /tmp/px4_navigation_before.json
./scripts/uav_lab.sh --no-build
```

同图像时刻几何分解使用 docs/tracking/approach_jitter_evidence_20261001/audit.py：传 --base 对应 mission_1 前缀、--ulog 对应原生 ULog、--output 输出目录；本次两轮 ULog 分别是 02_49_48.ulg、02_52_25.ulg，CSV 前缀分别是 modular_intercept_20261001_105018_794586_mission_1 和 modular_intercept_20261001_105303_545916_mission_1。各输入 SHA256 保存于 comparison.json 和各几何 audit.json。新原生 checkout 需先安装既有磁场补丁，然后用 git apply --check 检查本次 gz_gnss_initialization.patch，安装 ROMFS hook 并 make px4_sitl_default；当前机器已经完成，重复应用前可用 git apply --reverse --check 确认已安装状态。


## 默认启动构建扫描修复

用户默认执行 ./scripts/uav_lab.sh 时，colcon 从工作区根目录发现了本报告证据备份中的 native_before/CMakeLists.txt，误将它识别为构建包。先前指定 --packages-select 的验证没有覆盖默认扫描路径，这是本次备份引入的问题。

现已为 docs 添加 COLCON_IGNORE，并将 scripts/build_workspace.sh 的包发现范围限定到工作区 src；所有原始备份完整保留，没有删除失败构建目录或用户文件。新增真实 colcon discovery 回归测试已确认修复前误发现 native_before、修复后仅发现四个源码包。

通过默认启动所调用的 ./scripts/build_workspace.sh 完整构建：px4_msgs、uav_usv_interfaces、uav_control、uav_usv_bringup 全部通过（4 packages finished，3.68 s）。构建日志：px4_navigation_evidence_20261001/default_workspace_build.log。新增回归测试 1 项通过，shell 语法、该测试的 flake8 和 git diff --check 通过。未为该构建问题启动仿真。
