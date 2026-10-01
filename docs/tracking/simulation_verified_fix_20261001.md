# 接近、FOLLOW 与 Y 制导：修改及仿真验证（2026-10-01）

本轮以 HEAD `77872e9` 为基线。用户随后授权自主选择方案并启动仿真，实际完成五轮 Gazebo/PX4 验证。已恢复连续接近、稳定 FOLLOW 和 Y 后进入 MINCO；末端截获尚未成功，不能把“进入制导”写成“完成截获”。全部修改保留在工作区，未提交或推送。

原始实验均在 `data/experiments/current/`，原生 ULog 在 PX4 `rootfs/log/2026-10-01/`；原日志不覆盖。输入哈希、分阶段指标见 [simulation_verification.json](approach_jitter_evidence_20261001/simulation_verification.json)。完整状态机、日志生命周期、Phase A–D 命令及早期误差证据见 [18项报告](approach_jitter_fix_20261001.md)。

## 1. 确认的问题与修改

| 确认的问题 | 本轮修改 | 证据 |
|---|---|---|
| RGB 接近与搜索切换速度/位置模式，捕获高速时的搜索锚点后回拉 | Y 前保持速度控制，先平滑制动，指令及实测水平速度均小于0.1m/s后才固定锚点；保持指令历史 | 旧运行77次模式切换、6.55m/s跳变及真实反向运动；最新X-only原生模式切换0次，无高速保持回拉 |
| “画面居中”被错误用作远距离运动门限 | 3个新鲜独立RGB方向帧即可接近；沿 UAV heading + image bearing 运动，速度为 `6*cos(bearing)`，沿用限加速度 | 居中门限只授予3D锁定/FOLLOW，偏角0.149→0.151rad不再切入刹车；过期RGB仍撤销运动权限 |
| 同一RGB-D传感器被允许配对相邻帧 | localizer 的 maximum_rgb_depth_skew 从0.1s收紧到0.001s，三份实验YAML保持一致 | 有效观测原含48/52/100ms混配，最新2259帧全部同采集戳；不改变采集戳、时钟映射或因果等待 |
| KF的小矩阵BLAS线程争抢CPU | modular launch 在所有子进程启动前设置 OPENBLAS/OMP/MKL/BLIS 线程数为1 | 原KF约290%CPU，五个计算线程各约59%；3×6求解10000次约1.274s，单线程约0.144s；算法/Q/R未改 |
| 图像BEST_EFFORT传输丢帧，导致125ms门限反复失锁 | RGB-D和bearing控制入口改为 RELIABLE + KEEP_LAST(8) | 同场12s RGB/深度：BEST_EFFORT队列1或8均166/157帧，RELIABLE队列8均226/226帧，见[qos_capture.log](approach_jitter_evidence_20261001/qos_capture.log) |

PX4 pose/timesync 订阅仍用原 BEST_EFFORT(1)。可靠图像传输仍保留原采集戳、有限队列、重复/逆序拒绝与过期拒绝。控制仍保留0.125s freshness；没有扩大pose等待、修改P8.1映射、添加固定位置/yaw补偿或在线真值校准。

首次FOLLOW/安全恢复接管从自身实际速度初始化，短时刹车不计作停稳搜索漂移；重锁过渡分支按最终command.mode发布offboard owner。提前未锁定Y仍被拒绝，日志现在明确给出拒绝阶段和重按提示。

## 2. 仿真结果

目标维持 baseline 的4m/s八字运动、起飞约5m高度。下表是顺序排障的不同运行，样本、时长和模拟GNSS噪声不同；不能当作匹配样本的准确率因果试验。

| run前缀（均20261001） | 软件阶段 | 最长连续locked FOLLOW | 结果 |
|---|---|---:|---|
| 100810_841726 | 视线接近和制动修复 | 0.876s | 仍反复失锁 |
| 101252_688541 | 再加同次RGB-D配对 | 0.771s | 交付延迟降低，仍丢帧 |
| 101828_604389 | 再加单线程数值运算 | 2.647s | 仍受BEST_EFFORT图像丢帧影响 |
| 102220_716360 | 完整修复，稳定FOLLOW约20s后Y | 19.960s（CSV严格组合条件） | Y被接受，进入MINCO_TRACKING及TERMINAL_MINCO；末端失败 |
| 102705_450931 | 完整修复，X-only并开启几何诊断 | **55.503s** | 总FOLLOW103.770s；正常关闭生成ABORTED四类日志 |

第四轮driver以连续20s locked FOLLOW作为Y发送条件；CSV严格要求 phase/status/locked 同时满足，因状态消息与诊断发布交界记录约19.96s。

第五轮完整运行119.665s，45–90s窗口有900个periodic样本：实际水平距离中位4.552m、P95 4.666m、范围4.312–4.699m；锁定比例99.889%。事件统计仍有4次锁定丢失，包含起飞/初始接近，不能声称整场零失锁。该窗口有一次KF age=126.908ms越界，按原规则撤销锁定；没有放宽门限掩盖它。

![实际水平距离与locked FOLLOW](/home/qin/data/uav_usv/docs/tracking/approach_jitter_evidence_20261001/follow_verification.png)

第五轮有效视觉2259/2379（94.956%），拒绝为IMAGE_INVALID19、DEPTH_RATIO_LOW101；POSITION_TIMESTAMP_AFTER_HISTORY=0。同采集戳2259/2259。有效观测采集→发布中位35.071ms、P95 47.279ms，最大170.057ms；少量迟到帧不能授予控制时效。

## 3. 位置误差的同刻分解

专用几何实验仍采用4m/s八字目标，启用已有 evaluator-only UAV/USV参考。2258个有效同图像时刻样本，用原生ULog acquisition-time姿态/位置插值，拒绝无右括号样本，无外推、无经验lag拟合。Gazebo UAV参考与native groundtruth的水平差RMSE约1.976mm，支持本运行的model-origin NED参考对应。

离线恒等式：

```text
目标NED误差 = UAV位置估计误差
            + UAV姿态引入的目标旋转误差
            + 同刻完美pose替换后的剩余几何/参考误差
```

| 项目 | 全程2258帧水平RMSE | 解释 |
|---|---:|---|
| 在线原始目标位置对评价truth | 0.490006m | signed X=-0.033180m、Y=-0.076040m |
| native PX4同刻位置估计对native groundtruth | **0.444953m** | 剩余全局误差的主要来源 |
| 在线位置历史对native同刻EKF位置（时间交付贡献） | 0.001968m | 不能把导航估计误差误诊为时间映射错误 |
| 精确同刻姿态估计贡献 | 0.145788m | 不是固定yaw偏置 |
| 姿态时间贡献 | 0.001525m | 不支持再改P8.1时间链 |
| 离线同时替换自身位置和姿态后 | **0.079542m** | 20–90s同批1388帧为0.061533m；不是在线输出或新控制成绩 |

RMSE不能直接相加。位置的时间交付与估计贡献之和为在线位置对同刻groundtruth的误差（水平RMSE0.444781m）。逐帧完整分解闭合误差小于1e-15m，见 [native audit](approach_jitter_evidence_20261001/run102705/audit.json) 和 [paired frames](approach_jitter_evidence_20261001/run102705/paired_frames.jsonl)。独立heading RMS约0.952°，见 [既有bias分析器](approach_jitter_evidence_20261001/run102705_visual_bias.json)。

此前camera/body残差会包含自身pose误差：本轮camera-space范数RMSE0.589m、body-FLU也是0.589m，不能直接据此断言相机外参错了。分解加入自身位置后，剩余仅约0.080m，故没有再修改外参、球半径或轴符号。

生产修复消除了RGB/depth相邻帧混配。修复后约0.49m的全局NED差不能继续靠视觉固定补偿解决。PX4 GZBridge的NavSatCallback确实注入模拟位置/速度噪声，EKF2_GPS_P_NOISE=0.5；这支持继续单独研究自身导航误差，但尚未证明其全部滤波原因。本轮没有关闭GNSS噪声、乱调EKF/KF Q/R或把真值接入控制。

## 4. Y后制导与未解决项

第四轮运行约33.638s接受Y；33.732s MINCO_READY、33.768s MINCO_TRACKING，36.685s TERMINAL_MINCO。11次规划完成，8次成功、3次PLAN_STALE_ON_ARRIVAL按规则拒绝。说明此前“Y无反应/不进入制导”的问题已通过运行验证。

末端目标离开前视相机后，约37.436s进入SAFE_RECOVERY，之后SAFE_WAIT；最小三维距离0.929m，最终评价TIMEOUT，没有捕获成功。按既定本轮范围，没有重构BCTRA/MINCO、放宽capture radius或延用过期目标继续下压。

后续末端工作应先做前视/下视覆盖与视觉控制交接；现有下视流主要是诊断，单独把诊断selector的allow_down_fallback打开不能构成生产定位闭环。自身导航精度也应独立评估，不能用目标truth校准。

## 5. 边界、软件验证与交付状态

实际runtime查询确认 `/target/state` 仅有 evaluator 订阅，`/fmu/in/trajectory_setpoint` 仅有 trajectory_tracker_node 发布。独立几何参考只进入 evaluator/logger；controller/KF/BCTRA/MINCO没有读取真值。红球仍只是感知接口验证，未验证无标记USV检测。

- 全量包测试：**722 passed，1 skipped**；两个已知flake8插件API弃用warning。新增19个回归均经历预期失败再修复，其中连续接近16个、同次配对/线程环境/图像可靠传输3个。
- source、scripts、launch与audit脚本flake8通过，`git diff --check`通过。
- 三包colcon构建通过（2.38s）。[pytest.log](approach_jitter_evidence_20261001/pytest.log)、[colcon.log](approach_jitter_evidence_20261001/colcon.log)、[flake8.log](approach_jitter_evidence_20261001/flake8.log)。
- 独立代码审查未发现重要问题，另复跑相关116项通过。
- 自建Gazebo/PX4/DDS/experiment组件已按现场核验的PID清理，无仿真runtime残留；正常启动器每轮保存参数备份。未手工覆盖原日志、用户视频或记忆，未改native PX4源码。
- 清理曾因未现场核验PID被自动审批拒绝；只读确认命令、进程组和启动时间后，带所属会话校验的清理获准完成，无待批准操作。

重新启动即加载当前修复：

```bash
cd /home/qin/data/uav_usv
./scripts/uav_lab.sh --no-build
```

先X，FOLLOW稳定且locked后再Y；未锁定时提前按Y仍会拒绝，必须重新按。完整Phase A–D命令保留在18项报告中。
