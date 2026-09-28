# Gazebo 显示与 P5 诊断性能优化（2026-09-28 实测）

## 先回答验收问题

1. **卡顿来源**：物理仿真 RTF 的 P50 一直约 1.0，因而不是持续跑不动。两处确定的额外负担是 Gazebo 对 front/down RGB-D 的 GUI visualization，以及 P5 回调内的整帧红色检测、整帧深度解码和同步压缩。后者的单回调耗时已显著降低，但本轮**没有证明它是约 0.1 秒 RTF 尖峰的主要根因**；尖峰在优化后仍出现。当前机器使用 Intel ADL GT2/i915、Mesa 23.2.1 硬件加速，未安装 GPU 利用率监测工具。
2. **实际修改**：两台传感器仅将 `<visualize>true</visualize>` 改为 `false`，主 3D GUI 仍保留；P5 evaluation-only 采集器改为先裁剪小 ROI，再计算红掩膜和解码深度，使用容量 12 的非阻塞队列及后台压缩/写盘线程。增加采样脚本与测试。没有修改在线定位器。
3. **未做的修改**：`enable_metrics` 仍为 true；两个 `/uav/camera/{front,down}/performance_metrics` 话题运行时有 publisher、无 subscriber，全仓库也无消费者，但没有得到独立可测的关闭收益，保留其诊断价值。两个 monitor 在重处理前按 5 Hz throttle，compute P95 约 3–5 ms，未重构订阅/时钟。启动只出现一个 Gazebo server 和一个主 GUI，无 rqt/image viewer 或重复 GUI；未改 headless、shadows、光照、海面、RGB-D 质量、物理或 PX4。
4. **RTF**：三组普通场景的 P50 前后都在 0.9998 附近，未观察到持续退化；极少数低值在地面和起飞段变差、动态段改善，不能声称全面变流畅。下表列出全部低值而非只取改善场景。
5. **四路 RGB-D**：Gazebo-direct monitor 的四路输入现场约 20 Hz；ROS 多路采样器测到约 14–19 Hz，优化前已低于 20 Hz，优化后仍未达到“ROS 四路接近 20 Hz”的严格验收。单话题 `ros2 topic hz` 优化后约 front RGB 16.0、front depth 14.7、down RGB 16.7、down depth 17.2 Hz，说明不能仅归咎于四路采样器。四个 Gazebo/ROS 话题均存在，图像仍为 640×480、原 encoding、相同 intrinsics；没有为追求频率改变传感器条件。
6. **P5 尖峰**：跨会话 static5 38 秒，RTF <0.95 样本 5→6，<0.90 为 4→4；同一新版 GUI 会话连续运行旧/新采集器各 33 秒，<0.90 为 5→2，但 <0.95 均为 8，最小值 0.519→0.256。**整体尖峰减少尚未证实**。独立 100 帧同像素基准中，ROI 回调 P50/P95 为 1.170/1.214→0.026/0.108 ms；这只证明诊断回调变轻，不能替代 Gazebo RTF 结论。
7. **视觉数据**：static5 有效观测中，red pixel 中位数 515→518、depth median 5.3703→5.3667 m、depth MAD 0.0549→0.0552 m、物理相机球心误差 P50 0.0457→0.0447 m，均在原有波动范围。fx/fy 均为 269.9682、cx/cy 为 320/240；前后 ROI 档案与 JSONL 一一匹配。没有证据显示球心算法或图像几何被本轮改动。
8. **PX4/规划/控制**：相机 FOV、clip、640×480、20 Hz SDF 设定及 `always_on=true` 未变。`ocean.sdf` 与 HEAD 逐字相同；模型 SDF 除两处 `visualize` 和注释外逐字相同。控制仍 `target_state_source=simulation_truth`，视觉仍 shadow-only，MAD 门限仍 0.25 m。PX4、DDS、MINCO、BCTRA、KF、任务状态机均未改。
9. **肉眼流畅度**：取消两台传感器在 GUI 中的额外 visualization，预计减少显示端工作且保留主 3D 场景；本轮没有可靠的 GUI 帧时间或用户主观评分，**不把预期写成已观察到的改善**。现有 RTF/ROS 频率结果不足以宣称全部验收通过。

## 方法与原始数据

开始时 HEAD=`d86233a`，工作区干净。使用项目 `scripts/uav_lab.sh --no-build` 的标准 Gazebo 主窗口、PX4、DDS、ROS 启动；普通场景用原 baseline 配置，static5 用已有 `data/experiments/current/p5_20260927/static5_config.yaml`。各普通阶段采 40 s：地面 OFFBOARD hold、按 X 后起飞/FOLLOW、随后 +Y 动态段；未按 Y，没有截击或视觉闭环。static5 用相同 world、相机、目标、控制配置分别运行 38 s；额外在同一新版 GUI 会话中连续运行 HEAD 旧采集器和本轮采集器各 33 s，比较写盘路径。未运行 rqt_image_view。每次结束仅停止本轮启动且已核实 PID/命令行的 Gazebo/PX4/ROS 进程组，最终没有留下仿真进程。

`sim_render_perf_capture.py` 订阅 RTF、四路 ROS Image、两个 monitor 计算耗时和前向 TargetObservation，仅读元数据、接收时刻和 stamp；因此四路接收频率会受 Python 单线程订阅和主机负载影响。RTF 低值次数为采样点数，持续时间按相邻样本单调时钟积分；短会话之间的偶发事件不能做因果推断。首次地面 before 的观测计数因采样器初版订错 topic 而缺失，其 RTF/图像数据有效；已修正后续运行。早期 JSON 的 `process_load.cpu_percent` 曾被采样器重复读取而归零，不用于 CPU A/B；现场 `ps` 和后续同 GUI 数据用于进程负载检查。`ps` 是进程生命周期平均值，不是 GPU 帧时间。

### 普通仿真 RTF 前后

| 阶段 | 版本 | RTF min / P05 / P50 / P95 | <0.95 次 / 时长 s | <0.90 次 |
|---|---|---|---:|---:|
| 地面 | before | 0.9830 / 0.9983 / 0.9998 / 1.0010 | 0 / 0.000 | 0 |
| 地面 | after | 0.8746 / 0.9984 / 0.9998 / 1.0009 | 2 / 0.200 | 1 |
| 起飞/FOLLOW | before | 0.8215 / 0.9981 / 0.9998 / 1.0014 | 2 / 0.199 | 1 |
| 起飞/FOLLOW | after | 0.7546 / 0.9983 / 0.9998 / 1.0011 | 3 / 0.299 | 2 |
| +Y 动态 | before | 0.8146 / 0.9981 / 0.9999 / 1.0011 | 2 / 0.197 | 1 |
| +Y 动态 | after | 0.9086 / 0.9976 / 0.9998 / 1.0012 | 2 / 0.245 | 0 |

### 四路 ROS Image 到达频率（Hz）

列次序：front RGB / front depth / down RGB / down depth。采样器收到的原始 header stamp 中位间隔前后均约 0.052 s；间歇存在 0.1–0.2 s 到达/采样缺口，不能把 SDF `update_rate=20` 等同于每位 ROS 订阅者都拿到 20 Hz。

| 阶段 | before | after |
|---|---|---|
| 地面 | 16.69 / 15.97 / 19.29 / 18.97 | 17.75 / 17.09 / 19.09 / 18.69 |
| 起飞/FOLLOW | 17.05 / 15.54 / 18.83 / 18.28 | 17.58 / 16.73 / 18.78 / 17.98 |
| +Y 动态 | 17.37 / 15.93 / 18.46 / 18.04 | 17.61 / 16.87 / 18.86 / 18.26 |
| static5 + P5 | 15.36 / 16.11 / 18.52 / 18.37 | 15.07 / 14.02 / 18.20 / 16.12 |

图像 metadata 前后均为 RGB `rgb8` 640×480 step=1920、depth `32FC1` 640×480 step=2560，front/down `frame_id` 未变。SDF 水平 FOV 1.74、RGB clip 0.20–60.0 m、depth clip 0.20–25.0 m 和相机 pose 未变。项目 monitor 直接接收 Gazebo 话题，现场 `/diagnostics/{front,down}/{rgb,depth}_frame_hz` 分别约 20.08/20.00/19.97/20.09 Hz。front/down `monitor_compute_time` 普通地面 before P95 3.78/3.31 ms，static5 + P5 before P95 4.88/3.78 ms，after 5.53/3.90 ms；均未显示需要重构 monitor 的证据。

### P5 写盘与瞬时低 RTF

| 条件 | RTF min / P05 / P50 | <0.95 次 / 时长 s | <0.90 次 / 时长 s | ROI 档案/metadata |
|---|---|---:|---:|---:|
| 旧 SDF + 旧采集器 static5，跨会话 | 0.793 / 0.9976 / 0.9998 | 5 / 0.502 | 4 / 0.402 | 28/28 |
| 新 SDF + 新采集器 static5，跨会话 | 0.750 / 0.9961 / 0.9996 | 6 / 0.599 | 4 / 0.398 | 19/19 |
| 新 SDF + 旧采集器，同会话 | 0.519 / 0.9958 / 0.9996 | 8 / 0.829 | 5 / 0.529 | 24/24 |
| 新 SDF + 新采集器，同会话 | 0.256 / 0.9941 / 0.9998 | 8 / 0.802 | 2 / 0.199 | 22/22 |

同会话 A/B 仍是顺序运行，不是同步同帧；ROI 数和 ROS 观测频率也不完全相等。两次比较不能证明“同步压缩导致 RTF≈0.78”，也不能证明新 writer 已解决系统尖峰。另有一次 baseline 配置、未启用 geometry diagnostics 的旧 P5 采集，`roi_saved=0`，RTF min=0.649；这直接说明深尖峰可以在**没有 ROI 压缩**时出现，其 JSON 为 `sim_render_performance_before_capture.json`，没有混入上表 ROI 对照。仍保留异步 writer，因为它在相同像素下明确减少诊断回调阻塞，且在线链不使用它；不得将此解释为实测 GUI FPS 改善。

使用合成的 640×480 RGB8/float32、65×65 ROI，逐帧执行 100 次，旧路径为整帧红掩膜+整帧 depth 解码+同步 `np.savez_compressed`，新路径为局部裁剪/解码+有界队列提交，后台 writer 正常 flush。旧/新 ROI 像素逐元素相等；旧回调 P50/P95/max=1.170/1.214/3.164 ms，新回调=0.026/0.108/0.320 ms。新路径的后台写盘时间不计入 callback 时间；原始数字在 `sim_render_roi_callback_benchmark.json`。生产队列容量 12，满时 `put_nowait` 不阻塞并计入 `roi_writer_queue_drop`；两次新实测均 0 drop。关闭时先 drain、再等待 writer flush；文件编号只在成功入队后递增；所有本轮实际保存目录的 `roi.jsonl` 与 `.npz` 文件名集合一致。writer 不读写 ROS node 状态。

## 代码、测试与剩余问题

修改文件：`src/uav_usv_bringup/models/x500_mono_cam/model.sdf`（仅 front/down `visualize=false` 与说明注释）；`scripts/vision_p5_roi_capture.py`（ROI 局部解码、`RoiWriter`、队列 drop 计数与关闭 flush）；`src/uav_control/test/test_vision_p5_roi.py`（RGB/depth 等价、队列满、flush、编号、metadata 配对、失败清理及有界队列测试）。新增 evaluation-only `scripts/sim_render_perf_capture.py` 和本报告。未修改在线 `rgbd_target_localizer`、front/down monitor、世界、PX4、规划、控制或配置。

验证：新增 ROI 测试 8 passed；用户指定三个模块 **93 passed**（增加最后一个有界队列测试前）；最终从 `src/uav_control` 包目录加载 ROS/workspace 环境运行 **451 passed、1 skipped**，另有 2 条既有 `SelectableGroups` 弃用警告。`colcon build --packages-select uav_control uav_usv_bringup --symlink-install` 两包成功；改动文件 `flake8`、`py_compile`、`git diff --check` 通过。一次按用户示例直接覆盖 `PYTHONPATH` 导致 ROS/px4_msgs 测试收集失败，保留 ROS 路径后 93 passed；从包目录用相对 `install/setup.bash` 的首次全量调用找不到文件，改用绝对路径后首次全量 450 passed；最后增加队列上限回归后复跑为 451 passed。旧 static5 P5 采集曾在所有 CSV/ROI/JSONL 写完后 Gazebo Python 扩展退出码 134；文件核对一致，未记为正常进程退出。新采集退出码 0。

新增数据：`data/experiments/current/sim_render_performance_before.json`、`sim_render_performance_after.json` 为普通地面；同目录的 `_before_follow`/`_after_follow`、`_before_motion`/`_after_motion`、`_before_capture_static5`/`_after_capture_static5`、`_same_gui_old_collector`/`_same_gui_new_collector` JSON 是其余 A/B；`sim_render_roi_callback_benchmark.json` 为合成同像素回调基准。原始 static5 CSV、四组 ROI/JSONL 和各自捕获摘要保存于 `data/experiments/current/render_perf_20260928/{before_static5,after_static5,same_gui_old,same_gui_new}/`，没有覆盖 P5 历史实验。初始 HEAD=`d86233a`；本轮未 commit、push、reset 或 clean。

**仍未达到的验收**：四路 ROS 订阅实际收到的帧率未稳定接近 20 Hz；低 RTF 尖峰未稳定减少；没有客观 GUI frame time 或用户主观流畅度评分。下一步应独立定位 `ros_gz_image` 输出/ROS 接收缺帧与 GUI frame pacing，并在固定系统负载下做多次交错 A/B。不能通过降低传感器分辨率、更新率、shadows 或物理频率掩盖这些问题。
