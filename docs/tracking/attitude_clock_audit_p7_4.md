# P7.4：姿态估计与时钟误差离线归因

2026-09-29；审计基线为当前本地 master `1201381`，与本地 `github/master` 引用一致，未联网 fetch。本轮没有启动仿真，没有修改 PX4、相机变换、KF Q/R，没有在线使用 Gazebo truth 校正 PX4。分析脚本是纯离线工具。

**第一主因是 PX4 实际航向估计偏差经目标相对位置的旋转放大，时间对齐是次要项。没有发现能通过修正 camera→body→NED、四元数顺序或插值来消除该误差的明确 bug。** “真实估计偏差”指相同物理时刻的 PX4 输出与模型姿态仍不同，不代表已经证明偏差源头是哪一个 EKF 传感器/参数，也不代表它在工程上不可改善。

用户随后明确确认“将主预测也切到 KF，允许改变控制链”。因此主 BCTRA 已改用 KF 位置/速度；此前的 shadow-only 约束被这一后续确认覆盖到主预测分支。tracker 的直接目标状态、FOLLOW 历史和评价仍接真值，不宣称整个系统已迁移成视觉闭环。

## 可重复证据

- [分析脚本](../../scripts/p7_attitude_clock_analysis.py)
- [汇总 JSON](../../data/experiments/current/p7_attitude_clock_20260929/summary.json)
- [强追赶逐帧数据](../../data/experiments/current/p7_attitude_clock_20260929/plus_y_transition_frames.jsonl)
- [P7.3 原始评价定义](kf_current_b_dynamic_baseline_p7_3.md)

```bash
cd /home/qin/data/uav_usv
python3 scripts/p7_attitude_clock_analysis.py
```

读取原 `analysis_validated/paired_frames.jsonl` 的相同帧，不重选更好样本：static 294、+Y 稳态 226、−Y 237、+Y transition 136 帧全部保留，无新增拒绝。前 10 s 沿用 `phase_comparison_validated.json` 的“首个有效配对帧为零点”，+Y 为 70 帧。SLERP 采用两侧各不超过 50 ms 的 bracket，拒绝外推和跨姿态 reset；姿态与时间查询之间也检查 reset。

本机 PX4 `GZBridge::clockCallback()` 将 HRT 单调时钟同步到 Gazebo sim time。`poseInfoCallback()` 的 groundtruth 使用回调 HRT 而非 pose header，因此不能只因“同 ULog”便认定同物理时刻。本轮进一步验证：在**原 RGB sim stamp** 查询 ULog attitude groundtruth，与采集的图像时刻 Gazebo 模型姿态相比，作用于同一目标杆臂后的残差 RMSE 最大仅 0.000052 m（+Y 前 10 s，最大单帧 0.000408 m）。这支持在这些日志中直接用 sim stamp 对齐姿态；不是假设所有 PX4/真实硬件都适用此关系。

在线选取的 PX4 姿态与 ULog 在 ROS→ULog 自校准时刻的姿态，杆臂残差 RMSE 最大 0.000914 m。ROS→ULog offset 沿用 P7.3 的“同一 EKF 位置流自校准”，没有用目标定位误差拟合延迟。

设 `b` 为相机测量经安装外参后得到的 body FRD 向量（含相机平移），`Ro` 为在线姿态，`Re(ti)` 为原图像时刻的 PX4 EKF 姿态，`Rg(ti)` 为同一时刻的 ULog groundtruth，`Rm(ti)` 为图像模型姿态。精确分解：

```
(Ro − Rm)b = (Ro − Re)b + (Re − Rg)b + (Rg − Rm)b
              取时差项       真实估计项      参考残差
```

这是向量恒等式，不是 RMSE 相加。逐帧闭合最大约 4.2e−17 m，重建原 P7.3 姿态贡献最大差约 5.7e−14 m。

| 场景 | 原 Current-B RMSE m | 真实姿态估计项 m | 姿态取时差项 m | 仅对齐姿态时间后的反事实 RMSE m | 完美姿态反事实 RMSE m |
|---|---:|---:|---:|---:|---:|
| Static 30 s | 0.197 | 0.202 | 0.000034 | 0.197 | 0.045 |
| +Y 稳态 30 s | 0.121 | 0.157 | 0.00202 | 0.121 | 0.090 |
| −Y 含过渡 30 s | 0.160 | 0.180 | 0.00704 | 0.160 | 0.124 |
| +Y 强追赶前 10 s | 1.118 | 1.029 | 0.02941 | 1.110 | 0.311 |

反事实只改变作用于**同一测量向量**的姿态，不重算 RGB/depth、平台平移或目标实体时刻，也不是在线修复后的实测结果。`0.311 m` 表明强追赶仍有相机几何/异步观测的尾部问题，不能把稳态 `0.04–0.12 m` 推广到每一帧。修正姿态也不能保证每一帧误差下降，因为原有误差向量可能偶然抵消。

## 为何强追赶、远距离放大

一阶近似为 `δp ≈ −R[b]× δθ`（右乘、body 误差角），因此误差取决于距离、姿态误差轴与视线夹角；绕视线的旋转一阶影响小。航向误差主要按水平杆臂放大，不应只用三维斜距直接乘 yaw。

静态世界轴误差旋转向量均值约 `(−0.046, −0.046, +2.266)°`，明显以 yaw 为主；+Y 稳态 yaw 约 `−1.924°`，−Y 约 `+1.813°`。+Y 强追赶前 10 s 姿态角误差 RMSE 3.33°，P95 5.96°，最大 6.05°。它随阶段/场次变化甚至反号，固定 yaw 无法普适消除。水平距离 20 m、3° 的误差量级就是 1.05 m；即使只有 5 m，2.3° 也约 0.20 m。

时间误差一阶项约为 `R(ω×b)δt`，所以不仅要看 30–50 ms，更要看对应帧角速度和杆臂。强追赶同时存在姿态变化、相对速度和 RGB/depth skew；但本轮实际重采样表明它不是姿态米级误差的主因。平台平移与目标数学状态/实体的时间差是另外的误差项，本次“姿态时间对齐”反事实没有消除它们。

## 软件审计结论与未证明之处

1. `VehicleAttitude.q` 是 Hamilton `(w,x,y,z)`、body FRD→local NED；矩阵实现按该定义正向旋转，没有取逆或二次 ENU/NED 旋转。camera FLU 安装俯仰和平移先作用，再转换成 FRD，最后转 NED。独立 Gazebo 姿态反事实和现有几何测试支持当前实现。
2. PX4 位置/姿态都优先用 `timestamp_sample`；uXRCE 已转换到 ROS 时间时不再次叠加接收偏移。姿态使用归一化、符号连续、最短弧 SLERP，不是欧拉角线性插值。本机 EKF 发布的是 output predictor 的当前姿态，不应再硬减 EKF fusion horizon。
3. 图像按 Gazebo server sim/system anchor 映射，PX4 按 DDS 时钟映射，两条映射动态下存在约几十毫秒相对差。真正统一时基要使用有语义的时钟配对/同步状态，不能由目标 RMSE 扫描反推出固定 delay。现有日志足以量化其影响，但还不足以给通用在线映射打补丁。
4. KF 接收图像时间位置、先 predict 再 update，发布时只外推副本到当前 `stamp`。本轮没有发现 dt 或重复变换根因，KF 也无法自行识别并消掉慢变公共航向偏差。
5. 日志参数为 `EKF2_MAG_TYPE=0`、`EKF2_EV_CTRL=0`、`EKF2_GPS_CTRL=7`；状态中未启用 external-vision yaw 或 GNSS yaw。应优先审计航向观测、磁力计校准/创新、收敛状态及输出预测器，而不是继续改目标 KF。日志存在非零持久化 `CAL_MAG0_*OFF`，本机 Gazebo 磁力计桥还含旧版本单位/坐标兼容逻辑；它们是待验证项，**不能仅据配置就定性为 bug 或直接清零**。本轮没有证明某一个磁场配置/校准值是唯一根因。

PX4 关于航向观测与 EKF 的官方说明：[Using PX4's Navigation Filter](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf)。运行版本语义以上述本机源码/日志为准。

## 最合理的工程路线

**先改善源头航向可观测性，再表达剩余不确定度；时间同步与 RGB/depth 同步作为独立问题验证。** 不需要重设计整个系统。

1. 用现有 ULog 的 mag/heading innovation、校准参数、yaw estimator 和状态协方差，对齐检查航向误差发生/恢复时段；用传感器和估计器本身的证据判定磁场、标定、初始化或弱可观测性问题。真值只用于离线评分。工程上可用校准可靠的磁航向、双天线 GNSS 航向或独立视觉惯性航向来源降低误差；这些属于观测质量改善，不是给相机加经验 bias。无法仅靠当前相机对一个未知运动目标的测量，无条件分辨“目标横移”和“自机 yaw 偏差”。
2. 用真实时间对应关系统一 PX4/图像采样时刻；RGB/depth 尽量采用同触发/同仿真更新。不能只把 pairing 门限收紧后宣称精度改善，必须同时报告有效率，且同一帧集合对比。需要滚动快门、外参或动态去畸变时分别验证。
3. 协方差先实现**离线传播与一致性检查**，再接入现有 `TargetObservation.covariance` 接口。本轮遵照要求不改 Q/R，不把诊断数据用于在线修正。当前视觉已有 `base_std + range_scale×range` 的各向同性模型；它不是完整的姿态不确定度传播。

令 `p = p_u + R b`，右乘 body 小角度误差的雅可比 `Jθ = −R[b]×`。对相机测量、平台位置/姿态、外参、时钟组成的联合扰动 `ξ`，通用传播为：

```
Σp = J Σξ Jᵀ
```

若暂作独立近似：

```
Σp ≈ Σu + R Σb Rᵀ + Jθ Σθ Jθᵀ + Jt σt² Jtᵀ
```

这里 `Σb` 包括经外参旋转的 RGB-D 测量和外参不确定度；姿态取时误差的 `Jt` 包含 `R(ω×b)`，平台位置取时还包含平台速度，RGB/depth 异步的相对运动项应另建雅可比。共用时钟/导航状态时需要保留交叉协方差，不能把相关项重复相加。评价若使用 PX4 平台位置作锚，则与物理世界坐标评价的 `Σu` 含义不同。

姿态偏差跨帧相关，简单扩大每帧独立 R 会使滤波“承认不准”，但不能消除偏差；长期仍可能过度自信。更完整的方法是带有可观测性约束的姿态 nuisance state/相关测量模型，或分开保存随机协方差与公共姿态不确定度预算。绝不能只靠目标轨迹拟合在线 yaw bias，再当作独立校准。

BCTRA 应从 KF 的状态协方差传播 `Σpred ≈ F ΣKF Fᵀ + Σmodel`，结合航向公共误差和预测时域。当前 `prediction_to_message()` 写入 `(0.02 + 0.10×horizon)² I`，没有继承 KF 的 6×6 协方差，不能把它当作校准后的可靠置信区间。此次只切输入，不伪造姿态协方差来“补齐”该接口。

相对坐标跟踪适合减少对绝对世界航向的依赖，但 body/camera 是旋转坐标系，动力学需要 `−ω×r`、自机运动和相机杆臂；不能直接把当前 local-NED CV-KF 的输入改成 body vector。可先离线比较 body/短时局部稳定坐标下的相对位置与接近速度。最终若仍输出世界 NED，航向不确定度仍需传播，不能凭换坐标消失。

预期改善边界：只校正姿态取时，+Y 强追赶约从 1.118 到 1.110 m，收益很小；若真实航向误差显著下降，才可能削减约 1 m 的强追赶项和约 0.2 m 的静态项。当前完美姿态反事实残差是 0.311/0.045 m，而非已实现的性能承诺。协方差传播主要改善可信度、门控与风险判断，不保证 RMSE 下降；KF→BCTRA 输入切换本身也不是定位精度修复。

## 实际代码改动与输入边界

- `baseline.yaml` 主 `target_predictor_node.target_state_source` 从 `simulation_truth` 改为 `tracking`；节点无配置默认值同步为 `tracking`。
- 订阅 `/tracking/target_state` 的 position 和 velocity，输出 `/planning/target_prediction` 供现有 planner 使用；不从真值补 KF 速度，不再加一次图像年龄外推。
- 原 `/planning/shadow_target_prediction` 仍是隔离的诊断副本。它与主预测现在使用相同输入，并非 truth/KF A/B。
- `trajectory_tracker_node.target_state_topic=/target/state` 和 evaluator 真值保留。legacy `trajectory_impact_sim` 的内部预测未迁移；当前默认启动为 modular。不能把这次变化描述为全系统移除了真值。
- 保留参数名 `enable_shadow_perception` 兼容现有启动命令，但已更新说明：当前主预测依赖这条 KF 链，关闭它会使预测无输入/过期，不会自动回退真值。
- 新增离线脚本与回归测试，不修改在线几何、滤波算法、PX4 参数或控制安全限制。

验证结果：

- 在 `src/uav_control` 包目录执行 `python3 -m pytest -q --tb=short`：**482 passed、1 skipped**，另有 2 个 setuptools/entrypoint 弃用警告。首次从工作区根目录执行时，`test_flake8` 和 `test_pep257` 扫描到 build/install 生成物而失败；修正执行目录后两者通过。
- 最终相关 20 项测试通过；新增脚本/测试 flake8、pydocstyle 和 `git diff --check` 通过。
- `colcon build --packages-select uav_control uav_usv_bringup`：两包通过。
- 复跑离线归因，893 个原验证帧全部保留，数值如上。
- [KF→BCTRA 离线消息边界回放](../../data/experiments/current/p7_attitude_clock_20260929/prediction_replay.json)：四段合计 6498 个有效 KF 状态，输出 source 均为 tracking，预测零时刻位置逐条等于 KF 输入。此项检查输入边界，不证明预测精度或闭环稳定性改善。
- 未启动仿真、未提交或推送。真实定位误差仍未通过在线航向质量改善消除。
