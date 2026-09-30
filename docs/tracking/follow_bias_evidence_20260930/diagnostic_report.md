# 本轮视觉偏差分层诊断（2026-09-30）

本轮没有启动 Gazebo，没有修改视觉生产定位代码、P8.1 时间映射、KF Q/R、相机外参或固定位置/航向补偿。新增的是 evaluator-only UAV 原始模型姿态采集、专用诊断 YAML 和离线逐层统计。最新约 0.57 m 误差的现有 CSV 缺少 geometry 和独立 UAV heading，当前不能确定它产生于哪一层。

## 当前可复算证据

| 数据 | 有效帧/总帧 | 水平 RMSE | signed Y mean | geometry/heading |
|---|---:|---:|---:|---|
| 20260929_213155 最新短运行 | 47/49（95.918%） | 0.566165690 m | -0.519791127 m | 所有49帧 geometry关闭；无独立heading |
| 20260929_205024 较早运行 | 115/174（66.092%） | 1.117407234 m | -0.020504999 m | 6帧开启geometry，但有效帧没有可用逐层残差 |
| 20260923_153822 历史geometry运行 | 37/52（71.154%） | 0.249098681 m | -0.240788444 m | 36帧有限pixel/camera/body残差；无独立heading |

最新短运行的 `POSITION_TIMESTAMP_AFTER_HISTORY` 为 0。以上为不同会话、不同工况和不同阶段的历史记录；没有合并为当前控制闭环精度结论，也不能据此声称新版本有效率已通过仿真。

历史20260923_153822的分层统计：pixel projection二维范数 RMSE 28.009133 px（36帧）；camera center三维范数 RMSE 0.345081452 m；body-FLU范数 RMSE 0.345081452 m；vision-to-entity三维范数 RMSE 0.351762027 m（37帧）；entity-to-truth三维范数 RMSE 0.096638170 m（37帧）。所有层样本数明确记录在 JSON 中。**projection/camera/body 的 expected 值使用 PX4 estimated pose，不能用它们独立证明 mask、深度或外参有错误。** camera/body残差范数相等体现同一刚性旋转的耦合，不能当成两份独立证据。

此外窄范围复算现有 P7 static `analysis_final/paired_frames.jsonl` 的294个配对帧：camera geometry horizontal RMSE 0.037791684 m；PX4 attitude contribution horizontal RMSE 0.202178620 m；raw operational horizontal RMSE 0.194836758 m；entity-minus-math-target水平误差为0。这支持**该历史静态工况**中姿态贡献较大的既有发现。这里没有重做原始时间链和 source bracket 审计，不把这一结论迁移为当前 -0.5198 m bias 已确认的根因，也没有用这些字段构造当前 UAV heading reference。

可复算文件：`latest_short_run.json`、`historical_20260929_205024.json`、`historical_20260923_153822.json`、`historical_p7_static_decomposition.json`。前三份由当前 `scripts/p8_visual_bias_analysis.py` 直接读取原始 CSV 生成。输入绝对路径保存在各 JSON 的 `inputs` 字段。

## 新诊断边界与数据来源

`evaluation/uav_heading_diagnostics.py` 不创建 publisher，不给控制/KF/预测/规划发送消息。仅 evaluator 可选接收 Gazebo `/pose/info` 中**精确匹配** `gazebo_uav_entity_name` 的模型；默认 `x500_mono_cam_0`，不做前缀或link猜测。不存在/重命名时报告 reference unavailable，不能自动换成艇方向。

原始数据保存 Gazebo model ENU xyz 和 FLU→ENU 的真实 wxyz quaternion。复用既有 `GazeboImageClockMapper`、`TimedPoseHistory` 和 `interpolate_model_pose`：native sample stamp 先由可信 Gazebo clock双边锚点映射，有限0.15 s clock等待只用于 evaluator reference；采集戳不取receipt、不外推。历史姿态在 acquisition measurement_stamp 处做 SO(3) interpolation，并记录查询自身的左右sim/ROS源戳、fraction、interval和EXACT/INTERPOLATED状态。clock reset清空历史与待映射队列。映射未来锚点不足时仅pending；超时不保存其参考，不扩大线上P8.1等待门限。

Gazebo model FLU→ENU姿态用 `gazebo_model_rotation_ned_frd` 转为 FRD→NED。PX4 heading取图像时刻已插值的 PX4 quaternion 的yaw，独立 reference_heading取 UAV model quaternion 转换后的yaw。delta_yaw是wrapped PX4-reference差，记录 `range*sin(delta_yaw)`。actual_lateral_error将同刻视觉-truth XY误差投影至独立 UAV reference heading 的横向轴；signed NED induced X/Y以 estimated relative vector 与其逆yaw旋转之差计算，均是诊断，不做在线修正。

**Gazebo模型原点与PX4 local position原点/rigid body参考点是否完全等价尚未验证。** 原始模型NED位置可记录，但 `uav_position_reference_status=MODEL_ORIGIN_FRAME_NOT_ESTABLISHED`，不将其直接替代 PX4 position，也不提供错误的全独立B/C pose counterfactual。新增 attitude-only D counterfactual 保持测量camera vector、相机外参和PX4 position固定，仅替换为独立完整 UAV quaternion；状态为 `PX4_POSITION_HELD_FIXED`。它能显示姿态替换后的残差，仍不能证明剩余残差纯属检测/外参。

CSV 中 heading必须明确 `heading_diagnostics_status=VALID`、`heading_reference_source=gazebo_uav_model_pose`、geometry/reference flags开启、query自身采集戳一致且有效左右bracket覆盖采集时刻，离线分析器才读取嵌入reference。也支持原先 `--heading-reference` 输入，由调用者提供同NED frame、同acquisition戳的独立UAV heading；最多1e-6 s数字容差，不插值、不拟合延迟。PX4自身quaternion、truth艇方向或误差列不能产生独立reference。缺少参考时明确 `not_evaluated` / `insufficient_same_time_reference`。

## 用户自行执行 Phase D

专用配置是 `src/uav_usv_bringup/config/visual_geometry_diagnostics.yaml`。它复制baseline，只开启 RGBD geometry、evaluator entity、evaluator UAV reference并声明精确模型名，在线参数不改变。配置隔离测试比较 parsed YAML，保证四项差异之外与baseline相同。

```bash
cd /home/qin/data/uav_usv
UAV_USV_EXPERIMENT_CONFIG_FILE=/home/qin/data/uav_usv/src/uav_usv_bringup/config/visual_geometry_diagnostics.yaml \
  ./scripts/uav_lab.sh --no-build
```

先核对实际模型名与该配置一致，再做 X-only静态/低动态 FOLLOW，保持20–30 s后正常结束，确认主CSV、visionCSV、summaryJSON、config snapshot存在。每个工况用干净新会话：近/中/远距离、画面左/中/右、不同UAV heading；不要同时改变相机/KF/时间参数或先进入复杂终端MINCO。runtime truth audit应再次确认 `/target/state`只有evaluator/explicitlogger接收，reference仅在evaluator，不增加PX4 setpoint publisher。

离线分析新会话（实际文件名替换占位符）：

```bash
python3 scripts/p8_visual_bias_analysis.py \
  --vision data/experiments/current/ACTUAL_RUN_vision.csv \
  --output /tmp/actual_run_bias.json
```

若独立heading来自额外ULog/参考CSV：

```bash
python3 scripts/p8_visual_bias_analysis.py \
  --vision data/experiments/current/ACTUAL_RUN_vision.csv \
  --heading-reference /absolute/path/acquisition_aligned_uav_heading.csv \
  --output /tmp/actual_run_bias.json
```

按层检查pixel、camera、body、vision-to-entity、entity-to-truth，并同时看valid denominator/rejection histogram和各层有限sample count。比较同一批有真实heading reference的样本中 yaw proxy、signed NED induced error 与实际XY误差的相关性和残差RMSE；不能把不同样本集合的RMSE变化当成姿态解释率。只有独立参考/时间对齐支持时才进一步查物理模型、相机外参、轴符号和sphere中心。若主要指向PX4 EKF heading，本轮只记录结论，不加offset。

## 软件验证与未验证项

本子任务32项测试通过（analyzer20、UAV tracker/helper/config isolation12）；owned files的裸 `python3 -m flake8` 默认79列检查及ament_pep257通过。覆盖 acquisition双边pose查询、quaternion插值、clock等待、显式超时、clock reset、精确模型名、无参考/geometry关闭、ENU/FLU→NED/FRD、position frame隔离、有限JSON、嵌入reference provenance与bracket拒绝、逐层统计、专用YAML隔离。

未启动Gazebo；新增reference的实际topic availability、实际model name、完整新软件视觉有效率及P8.1在线rejection count待用户运行验证。当前没有发现证据充分、可在本轮修复的真实物理/坐标错误，故不改视觉生产代码。红球接口实验也不能宣称已验证无标记非合作艇检测。
