# P4：RGB-D 几何鲁棒性与 PX4 仿真参考剩余归因（2026-09-27）

## 先回答四件事

1. **确认的根因**：B 组的常态 0.07–0.14 m 残差包含 `median(X depth)+radius` 的球心近似误差；极端尾部主要是球体红色掩膜内深度分布不符合单球面，既有 `minimum_depth_ratio=0.5` 不能识别。PX4 `groundtruth` 来自与评价器相同的 Gazebo `/world/default/pose/info` **模型原点**，但 PX4 桥在收到消息时用 `hrt_absolute_time()` 标记，而评价器使用消息头的仿真采样时间；这解释了两份“同名真值”具有不同时间语义，尚不能唯一证明固定延迟。地面约 0.132 rad 的 PX4 yaw 来自实际校准磁测量与 `EKF2_MAG_DECL` 的磁航向初始化，四个历史会话均能数值复现。
2. **实际修改**：仅在在线 RGB-D 定位器加入相机 X 深度 MAD 上限 0.25 m，超限发布带质量字段的无效观测 `DEPTH_MAD_HIGH`；同步 baseline YAML 和节点默认值，新增节点级测试。球心公式、相机外参、PX4 EKF 参数及控制链均未改。
3. **改善与有效率**：对历史九组全部完整有效帧做**同帧离线回放**，MAD 门控拒绝 14/1795（原完整有效帧的 0.78%；按全部 1921 行计，完整有效率 93.44%→92.71%）。−Y 全有效帧的物理相机三维 RMSE 0.576→0.197 m，最大值 6.371→0.366 m；+Y 为 0.232→0.154 m。P3 严格巡航子集 −Y 0.583→0.199 m、149→146 帧。新的 −Y shadow-only 仿真实际记录 1 次 `DEPTH_MAD_HIGH`、235/281 条有效观测（83.63%），巡航相机系 RMSE 0.159 m；此新会话与 P2 不是逐帧配对，**不能把跨会话差值当成因果改善**。在线 PX4 位姿偏差仍在，不能将 B 组改善称为全球目标定位已修复。
4. **尚未解决**：异常深度来自背景混入、球体边缘/遮挡还是约 48–52 ms RGB/Depth 采样错位的各自份额，没有逐像素原始深度可唯一确定；球心近似仍存在；PX4 桥接的实际逐消息接收间隔与地面磁场观测为何偏离直观世界磁场换算，历史日志也不能唯一确定。保留原球心公式和 PX4 参数。

## 现场、范围与误差定义

开始时 HEAD `ecf7ae9bd9212616f7cd1dce4a4783c86bee4e8c`，`git status` 干净，无 Gazebo/PX4/ROS 实验进程。本轮读取 P3 报告与 JSON、P2 原始九组 CSV、定位器/测试、Gazebo SDF、PX4 源码及 ULog。P4 新仿真按既有安全启动脚本执行，只发送 `X` 起飞/跟随命令，没有发送 `Y` 进入截击；采集结束后只停止本轮自行启动的四个仿真进程组。未覆盖 P2/P3 数据，未提交或推送。

视觉几何误差为 `center_camera − physical_expected_camera`，在相机 FLU 中统计；三维 RMSE 为 `sqrt(mean(error_3d²))`，P50/P95/最大值统计逐帧三维范数，X/Y/Z 是有符号均值。P3 的 A/B 分别是原 PX4 位姿/离线 Gazebo 模型位姿进入同一个定位器坐标变换后的 NED 结果，参照同帧 USV 实体。选择帧沿用 P3 完整有效、FOLLOW 末 15 s 与运动巡航的口径；另独立统计九组 P2 的**全部**完整有效帧。历史 CSV 未保存 `depth_min` 或逐像素深度，不能据此补造数值；`valid_depth_ratio` 由 `valid_depth_count/red_pixel_count` 精确重建。

## 深度异常与物理候选

−Y 巡航最大误差帧（图像时间 `1790166212.3823216`）相机误差 **6.371 m**，其中前向 X **+6.147 m**：`depth_median=23.491 m`、物理应得球心前向 X `17.594 m`，有效深度 **12/21**、MAD **1.338 m**。同帧物理水平投影误差仅 **−2.61 px**，RGB/Depth skew **0.052 s**；目标像素中心仍接近应有位置，前向深度明显错误。第二大帧误差 1.667 m，有效深度 10/12、MAD 1.691 m；第三大帧 1.228 m，111/111 点、MAD 0.435 m。三帧均超过球半径 0.25 m 的 MAD 物理界限。历史日志支持“深度混合或动态错位导致深度分布异常”，但没有原始逐像素深度，无法证明其中的背景、球缘遮挡和 RGB/Depth 时间错位各占多少。0.052 s skew 在正常运动帧也常见，不能单凭它归因；不能只删掉异常帧后宣称球心公式变准确。

对于半径 `R` 的单个球，其任意表面点的相机 X 在 `[C_x−R,C_x+R]`，所以干净球面 X 深度的**中位绝对偏差不大于 R**。采用 `depth_mad ≤ target_radius=0.25 m` 是几何上界，不是搜索九组日志得到的经验最优阈值。此上界能识别大深度混合；它不能识别小于阈值的系统偏差或多种距离接近的污染。保留已有最小深度比例 0.5；离线对照显示单独提高比例或有效点数会多拒绝远距小球，仍留下显著尾部。

合成 640×480、水平 FOV 1.74 rad、半径 0.25 m 的球面，逐像素用**相机 X 分量深度**计算与球的光线交点，再调用现有 `target_geometry_from_rgbd`。中心 5/7 m 的 `median(depth)+R` 三维偏差 0.066/0.073 m；7 m 左右离轴为 0.081 m。沿代表射线增加一个球半径时，离轴降为 0.066 m，但中心不变；因为中位深度来自整片可见球面而非中央射线，单纯射线修正不能消除主要偏差。已知半径的三维表面点鲁棒拟合在**无噪声合成数据**中可达到数值零误差，但历史 CSV 没有逐像素深度，无法在真实数据上评估离群点、计算开销和退化情况，所以本轮没有把拟合接入在线定位器。

## 九组全部有效帧：相机系同帧门控对照

每个误差单元为 **RMSE/P50/P95/最大值（m）**；最右列是 X/Y/Z 有符号均值（m）。箭头前后使用完全相同的历史图像及模型/实体时间参考，差别只是按新规则拒绝异常观测，**不是留下帧的估计值被修正**。有效率以各场景门控前完整有效帧为分母。

| 场景 | 保留帧；拒绝率 | 三维误差前→后 | X/Y/Z 均值前→后 |
|---|---:|---|---|
| 地面中心 | 134→134；0% | .119/.119/.119/.119 → 同前 | +.044/+.014/+.110 → 同前 |
| FOLLOW 5 m 全段 | 309→308；0.3% | .200/.094/.101/3.101 → .094/.094/.101/.143 | +.091/+.007/−.034 → +.082/+.009/−.036 |
| FOLLOW 7 m 全段 | 343→343；0% | .076/.073/.100/.122 → 同前 | +.070/+.005/+.010 → 同前 |
| 地面左侧 | 109→109；0% | .129/.129/.129/.129 → 同前 | +.046/−.047/+.111 → 同前 |
| FOLLOW 左侧全段 | 274→269；1.8% | .389/.094/.129/3.233 → .098/.094/.129/.198 | +.126/−.001/−.017 → +.078/+.002/−.019 |
| 地面右侧 | 86→86；0% | .142/.142/.142/.142 → 同前 | +.046/+.075/+.111 → 同前 |
| FOLLOW 右侧全段 | 239→237；0.8% | .183/.094/.134/2.066 → .097/.094/.130/.163 | +.093/+.006/−.029 → +.081/+.008/−.026 |
| 运动 +Y | 148→145；2.0% | .232/.127/.233/1.286 → .154/.126/.220/.497 | +.097/−.057/+.065 → +.075/−.056/+.074 |
| 运动 −Y | 153→150；2.0% | .576/.180/.316/6.371 → .197/.180/.292/.366 | +.129/+.105/+.075 → +.072/+.109/+.065 |

FOLLOW 全段包含起飞过渡，所以比 P3 稳定末 15 s 子集更容易出现异常帧。新门控在 P3 稳定中心、5 m、7 m、左右离轴子集各拒绝 **0** 帧，A/B NED 指标不变。P3 严格巡航 +Y 中，A NED RMSE 1.356→1.342 m、B 相机等价 RMSE 0.232→0.154 m，148→145 帧；−Y 中 A NED 1.212→1.051 m、B 0.583→0.199 m，149→146 帧。A 的剩余米级误差来自 PX4 位姿参考等因素，MAD 门控不能修正保留帧坐标。完整九组候选规则的保留帧数、拒绝比例、RMSE/P50/P95/最大值、X/Y/Z 均值和相机系/NED 对照保存在 `p4_20260927_geometry_gate_analysis_v2.json`，早期仅 P3 子集分析保留为 `p4_20260927_geometry_gate_analysis.json`。

| 候选规则（−Y 全段；基线 153 帧、RMSE .576、P95 .316、最大 6.371 m） | 保留帧 | RMSE/P50/P95/最大（m） | X/Y/Z 均值（m） |
|---|---:|---|---|
| 有效深度点 ≥9 | 153 | .576/.180/.316/6.371 | +.129/+.105/+.075 |
| 有效深度点 ≥16 | 121 | .221/.176/.313/1.228 | +.086/+.104/+.042 |
| MAD ≤半径 .25 m（采纳） | 150 | .197/.180/.292/.366 | +.072/+.109/+.065 |
| MAD/中位深度 ≤5% | 151 | .220/.180/.301/1.228 | +.079/+.109/+.063 |
| 有效深度比例 ≥75% | 143 | .261/.179/.312/1.667 | +.090/+.107/+.063 |
| ≥9 点且 MAD ≤.25 m | 150 | .197/.180/.292/.366 | +.072/+.109/+.065 |
| ≥16 点且 MAD ≤.25 m | 120 | .192/.175/.307/.366 | +.077/+.104/+.044 |

+Y 对应七种规则的完整数据在汇总 JSON：单独提高至 ≥16 点会从 148 帧拒绝 35 帧，RMSE 反而 0.232→0.242 m，1.286 m 的最大异常仍在；采纳的 MAD 规则只拒绝 3 帧，RMSE 0.232→0.154 m。两种方向交叉核对表明增加点数门槛主要通过大量拒绝难样本改变统计量。`depth_median` 与 `target_range` 和误差有共同距离趋势，`red_pixel_count` 随距离减少；`depth_mad` 的全局 Spearman 相关系数受距离混杂，不能用相关系数单独替代物理边界。`view_angle`、投影误差和 RGB/Depth skew 在尾部异常帧并不一致，未作为本轮在线门控。

## 新的 shadow-only 仿真复测

只复制 P2 `motion_minus_v2` 配置并添加 `maximum_depth_mad: 0.25`，其他项通过 `diff` 确认为一致。采集器仅读取 `/perception/front/target_observation`、PX4、Gazebo 实体和目标真值，不发布控制。独立采集 35 s：`data/experiments/current/p4_20260927_sim/vision_static_capture_20260927_160937_825404.csv` 及同名 JSON，共 281 条观测，235 条有效；无效原因 `DEPTH_RATIO_LOW` 31、`IMAGE_INVALID` 14、**`DEPTH_MAD_HIGH` 1**。被新规则拒绝的帧有 180/180 个有效深度点、MAD 0.390 m，说明仅依赖点数或深度比例会放行。巡航有效帧 200 条，相机系 RMSE/P50/P95/最大 **0.159/0.131/0.266/0.414 m**，相对同帧实体的在线 NED 三维 RMSE/P50/P95/最大 **0.968/0.513/2.197/2.573 m**。新会话只证明在线拒绝路径生效且保持控制边界；历史同帧回放才是前后归因依据。按这 281 条实际输出推算，若仅移除该一条新门控拒绝且其他流程不变，有效率会从 235/281（83.63%）增至 236/281（83.99%，差 0.36 个百分点）；这是假设性上界，**没有运行独立的同会话关门控 A/B 实验**。

## PX4 `groundtruth` 源码归因与剩余时间问题

历史 ULog `ver_sw=de8158101c96ad6b04170dc91f087148104c58eb`，与本机 PX4 源码 HEAD 一致，当前仿真通过 `gz_x500_mono_cam` 的 `GZBridge`，不是 Gazebo Classic 的 MAVLink 插件。`GZBridge::subscribePoseInfo` 订阅 `/world/default/pose/info`；`poseInfoCallback` 按 `_model_name` 选**模型条目**，直接将 `pose.position` 变为 `(Y,X,−Z)` 发布 `vehicle_local_position_groundtruth`，完整四元数由 `rotateQuaternion` 转成 NED/FRD 发布 `vehicle_attitude_groundtruth`。没有额外 base_link/IMU/COM 平移；速度由**同一模型位置序列**差分 `(position − _position_prev)/dt` 计算，`dt` 为两次回调时间差、限在 `[0.001,0.1] s`，因此位置和速度源于同一次 `pose/info` 状态，但速度是回调时差的有限差分，不是 Gazebo 消息中的独立速度字段。`timestamp` 与 `timestamp_sample` 都赋值为 `hrt_absolute_time()` **回调时刻**，两类 groundtruth 历史 ULog 中全量相等；源码没有读取 `msg.header.stamp` 作为真值采样时刻。 两轮巡航 ULog 的 groundtruth 更新间隔中位数/P95 均为 20 ms，最大 36/40 ms；Gazebo 传输、桥接订阅回调及独立评价器订阅各有自己的队列，历史 ULog 未记录每条消息头时间与入队/出队时间，不能直接测出各队列延迟。`GZBridge::clockCallback` 又用 Gazebo `Clock.sim` 更新 PX4 单调时钟。P2/P3 评价器则用 `pose/info` 头部仿真时间与 Gazebo Clock 的 sim/system 双时间映射查询图像时刻。

故同一物理位姿内容 `p(s)` 被两条链标以不同时间：评价器使用消息头采样时间 `s`；PX4 groundtruth 使用桥接收到消息时的 PX4 时钟 `τ`。在模型原点无差别的前提下，同“查询时间”位置差可写为 `T[p(s_eval(t)) − p(s_bridge(t))]`，T 是 ENU→NED，速度恒定近似为 `v_UAV·(s_eval−s_bridge)`。P3 历史 +Y/−Y 的模型−PX4 groundtruth Y 均值 **−0.139/+0.215 m**，除以各自 UAV 真值速度 **+4.733/−4.113 m/s**得到约 **−29/−52 ms 的等效样本相位**；两方向不相等，且旧巡航 CSV 没有桥接收到的原始 `pose/info` 头时间，**不能据此认定可配置的固定延迟**。模型位置坐标来源已由源码确认相同，先前“model origin 或 base_link 参考点差异”这一候选被排除；真实队列与时钟映射偏差的份额仍未测出，不修改在线时间映射。

若继续实验，最小数据集应同步记录每条 `pose/info` 的头部 sim 时间、模型位置和 PX4 桥接回调 `hrt` 时间，并使用安全 Offboard 的**静止→加速→+Y 恒速→减速→静止→反向加速→−Y 恒速→减速→静止**完整 UAV 轨迹。先在 +Y 的非匀速段拟合一个时间模型及独立固定原点，再在 −Y 段保持参数不变验证；所有插值必须在真实样本历史内。现有 P2 两段几乎全是恒速巡航，无法将常量位置项与时间项可辨识地分开，故本轮未为了“凑”0.5 m 做延迟拟合或修改 PX4。

## PX4 地面 yaw 与磁航向

海洋世界 `/home/qin/data/uav_usv/src/uav_usv_bringup/worlds/ocean.sdf` 配置磁场 `(6e−6,2.3e−5,−4.2e−5) T`，世界球面坐标约北纬 47.397971°、东经 8.546164°。x500 `base_link` 磁传感器 100 Hz；`GZBridge::magnetometerCallback` 有当前 Gazebo 版本左手坐标/单位兼容转换 `(-y,-x,z)`。四会话 ULog 的实际参数均为 `EKF2_MAG_TYPE=0`、`EKF2_DECL_TYPE=3`、`EKF2_MAG_DECL=3.405765°`、`SIM_GZ_EN_MAG=1`，校准偏置 `CAL_MAG0_X/Y/ZOFF≈−0.00149/+0.00623/+0.00192 G`；参数详情见审计 JSON。

各会话仿真真值开始后 2–12 s，校准后的磁测量中位数约 `(0.21763,−0.01591,0.42738) G`。地面航向关系 `atan2(−m_y,m_x)+EKF2_MAG_DECL` 得 **0.13238–0.13243 rad**；与四会话实际 EKF yaw 的差分别 **+0.00038、−0.00074、−0.00057、+0.00109 rad**，而同窗 Gazebo/PX4 groundtruth yaw 约 0。地面约 −0.13 rad 的 groundtruth−EKF yaw 因此已由实际磁测量与偏角参数定量解释。ULog 中 `cs_yaw_align`、`cs_mag_hdg` 在约 1 s 时启用；起飞后约 3 s，`cs_mag_aligned_in_flight` 变 1、`cs_mag_hdg` 转 0、`cs_mag_3d` 转 1，伴随 yaw 差逐渐靠近 0。磁融合 aid source 的 `fused` 在审查窗口为 1，磁创新量及模式时间见 JSON。为何当前 Gazebo 磁输出的机体水平分量与世界字段的简单坐标换算不一致、三轴融合何以取得不同的稳定 yaw，尚无足够的逐消息传感器/姿态配对与 Gazebo 插件内部依据；不更改世界磁场、校准或 EKF 磁融合参数，更不关闭融合来降低视觉 RMSE。

## 实际文件、验证和边界

| 文件 | 函数或内容 | 原因及影响 |
|---|---|---|
| `src/uav_control/uav_control/perception/rgbd_target_localizer.py` | `RgbdTargetLocalizer.__init__` 新增 `maximum_depth_mad=0.25`；`localize` 在几何提取后拒绝超限帧；`publish_invalid_observation` 对该拒绝保留深度质量字段 | 唯一在线行为改变是少量深度分布异常帧变为无效观测，沿原无效观测安全路径发布；不改球心坐标、KF、控制或外参 |
| `src/uav_usv_bringup/config/baseline.yaml` | `rgbd_target_localizer.ros__parameters.maximum_depth_mad: 0.25` | 与节点默认值及当前 0.25 m 红球半径同步 |
| `src/uav_control/test/test_rgbd_target_localizer.py` | 新增高 MAD 帧节点级拒绝与质量字段测试；检查 baseline 参数 | 先复现原实现错误（测试失败），再验证 `DEPTH_MAD_HIGH`、NaN 位置、零置信度和不发布有效目标位置 |
| `scripts/vision_p4_geometry_gate_analysis.py` | `metrics`、`ned_metrics`、`gate`、`main` 等 | 九场景全部有效帧与 P3 严格子集的相机系/NED、保留率和候选门控对照 |
| `scripts/vision_p4_synthetic_sphere.py` | `render`、`evaluate`、`main` | 验证 X 深度球面光线交点、射线修正和已知半径拟合的数学效果 |
| `scripts/vision_p4_px4_mag_audit.py` | `audit`、`attitude_yaw`、`main` | 四会话磁测量、参数、航向和融合创新的可复现审计 |
| `docs/vision/vision_rgbd_geometry_px4_reference_p4_20260927.md` | 本报告 | 归因、前后对照、证据缺口与安全边界 |

新增数据：`data/experiments/current/p4_20260927_geometry_gate_analysis.json`（首版 P3 子集）、`p4_20260927_geometry_gate_analysis_v2.json`（全部帧与 P3 子集）、`p4_20260927_synthetic_sphere.json`、`p4_20260927_px4_mag_audit.json`、`p4_20260927_motion_minus_mad025_config.yaml`、`p4_20260927_sim/vision_static_capture_20260927_160937_825404.csv` 及其同名 `.json`；PX4 原始 ULog `/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-27/08_09_01.ulg`；ROS launch 日志 `/tmp/uav_usv_ros_logs/2026-09-27-16-09-07-813732-qin-GLO-FX6-19106/launch.log`。三个离线脚本均可用 `--output` 指定新路径重跑，默认输出已存在时排他写入会报错，防止覆盖。所有输出为独立新文件，历史 P2/P3 原始数据未修改。

本轮测试：新节点测试先失败后通过；`src/uav_control` 全量 **442 passed, 1 skipped**，2 条已有 `SelectableGroups` 弃用警告；受影响文件和三份新脚本 `flake8` 通过；`colcon build --packages-select uav_control uav_usv_bringup --symlink-install` 两包通过。新增脚本及受影响模块 `py_compile`、`git diff --check` 和结果 JSON 完整性均在报告完成后复核通过；三份离线脚本另以独立 `/tmp` 路径重跑，结果与所交付 JSON 逐字节一致；最终 Git 状态仅含本轮列出的修改/新文件。新仿真只有 shadow-only 视觉验证；规划控制继续使用仿真真值，未修改 KF、BCTRA、MINCO、PX4 控制器、任务状态机、海面安全或捕获判据。未执行 Git 提交或推送。
