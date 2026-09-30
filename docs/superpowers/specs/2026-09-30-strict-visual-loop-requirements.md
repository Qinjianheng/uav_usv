## 总目标

将当前系统升级为严格的非合作视觉闭环：

```text
RGB-D
  ↓
视觉目标检测/定位
  ↓
KF
  ↓
BCTRA
  ↓
MINCO
  ↓
Trajectory Tracker
  ↓
PX4
  ↓
UAV运动
  ↓
RGB-D重新观测
```

在线控制、状态判断、搜索、预测、规划、跟踪均不得读取 Gazebo USV truth。

`/target/state` 只允许进入 evaluator、日志和明确标记的 offline/shadow diagnostics。

同时新增目标搜索与重捕获：

```text
TAKEOFF
   ↓
TARGET_ACQUIRE
   ↓
TARGET_LOCK
   ↓
FAR_GUIDANCE
   ↓
MINCO_ENTRY
   ↓
MINCO_TRACKING
   ↓
TERMINAL_MINCO
```

若非终端阶段目标丢失：

```text
TRACKING
   ↓
TARGET_LOST
   ↓
REACQUIRE
   ↓
TARGET_LOCK
   ↓
恢复原流程
```

如果从未看到过目标，则采用有界搜索扫描，不读取目标真实方向。

---

## 一、先做全仓库 truth audit，不立即修改

检查整个：

```text
src/
scripts/
config/
launch/
```

搜索：

```bash
grep -RIn \
  --exclude-dir=build \
  --exclude-dir=install \
  --exclude-dir=log \
  --exclude-dir=.git \
  -e "/target/state" \
  -e "truth_topic" \
  -e "simulation_truth_topic" \
  -e "target_state_topic" \
  -e "gazebo_entity" \
  src scripts config
```

对每一个 truth 消费者分类：

```text
A. evaluator / logger
B. shadow diagnostics
C. 在线 perception
D. 在线 predictor/planner
E. tracker/controller
F. mission/state transition
G. safety logic
```

要求：

```text
A/B 可以保留；
C/D/E/F/G 全部禁止读取目标 truth。
```

特别检查：

```text
trajectory_tracker_node:
    target_state_topic: /target/state
```

不要简单把 topic 名改成 `/tracking/target_state`。

先读取 tracker 中该 target state 的全部使用点，明确它分别用于：

```text
FAR_GUIDANCE
endpoint mismatch
plan replacement
terminal handover
yaw command
target distance
loss判断
```

然后按实际语义替换。

---

## 二、P8.2：移除在线目标 truth

### 2.1 主预测链

保持：

```text
/perception/front/target_observation
        ↓
target_kalman_filter
        ↓
/tracking/target_state
        ↓
target_predictor_node
        ↓
/planning/target_prediction
```

`target_predictor_node` 保持：

```yaml
target_state_source: tracking
tracking_topic: /tracking/target_state
```

在线 BCTRA 禁止 fallback 到 `/target/state`。

如果视觉/KF失效：

```text
不得切 truth
```

只能进入：

```text
STALE
TARGET_LOST
SAFE_WAIT
REACQUIRE
```

### 2.2 Tracker

检查 tracker 对 target state 的真实需求。

原则：

```text
当前目标位置/速度
    → /tracking/target_state

未来目标位置/速度
    → 当前有效 /planning/target_prediction

MINCO terminal endpoint
    → trajectory/planner 消息自身携带的信息
```

不要让 tracker 为了验证一条视觉规划轨迹再读取 Gazebo truth。

`TARGET_ENDPOINT_MISMATCH` 必须改成：

```text
planned endpoint
vs
当前 KF/BCTRA 可获得的目标估计/预测
```

或者如果该检查本质已经在 planner 中完成，则删除 tracker 中重复的 truth-based endpoint gate。

### 2.3 Evaluator

保持：

```text
/target/state
```

但必须明确：

```text
truth_role = evaluation_only
```

truth 可以用于：

```text
成功判定
minimum distance
RMSE
prediction error
offline statistics
```

绝不能反向影响：

```text
SEARCH
LOCK
KF
BCTRA
MINCO
Tracker
PX4 command
```

---

## 三、增加 TARGET_ACQUIRE / REACQUIRE 状态机

不要把搜索逻辑塞进 MINCO。

搜索是 mission/tracker 层的独立行为。

推荐状态：

```text
GROUND_HOLD
TAKEOFF
TARGET_ACQUIRE
TARGET_LOCK
FAR_GUIDANCE
MINCO_ENTRY
MINCO_TRACKING
TERMINAL_MINCO
REACQUIRE
SAFE_RECOVERY
```

### 3.1 起飞后行为

保持现有安全起飞逻辑。

起飞过程中：

```text
位置控制继续负责升高；
不要为了找目标产生大水平运动；
允许有限 yaw 转动。
```

达到：

```text
takeoff_horizontal_full_height
```

或新的：

```text
target_search_enable_height
```

以后允许进入搜索。

不要在刚离地几十厘米时快速旋转。

---

## 四、实现“朝目标消失方向搜索”

新增一个非常轻量的 `TargetVisibilityState`，保存：

```text
last_valid_observation_stamp
last_valid_image_bearing
last_valid_bearing_sign
last_lock_stamp
consecutive_valid_frames
consecutive_lost_frames
```

### 4.1 目标可见时

从图像目标中心计算水平 bearing：

```text
normalized_x =
    (target_u - camera_cx) / camera_fx

bearing =
    atan(normalized_x)
```

约定并通过相机坐标定义确认符号：

```text
bearing > 0
bearing < 0
```

分别对应机头应向哪一侧 yaw。

必须用单元测试验证左右方向，禁止凭经验猜符号。

### 4.2 目标消失瞬间

如果此前已经看到目标：

```text
SEARCH_DIRECTION =
sign(last_valid_image_bearing)
```

然后：

```text
保持当前位置/安全高度
+
缓慢向目标最后消失方向 yaw
```

例如新增参数：

```yaml
target_search_yaw_rate: 0.35
target_search_max_yaw_rate: 0.6
```

不要一开始就用现有 `max_observation_yaw_rate=1.0` 全速扫描。

### 4.3 不要只按照最后方向无限旋转

实现有界的分阶段搜索：

```text
阶段1：
沿 last_seen_direction 搜索 +Δψ

如果仍未发现：
回到中心附近

阶段2：
反方向搜索 -2Δψ

阶段3：
扩大为 +3Δψ

最终：
bounded sweep / 360° search
```

例如：

```text
last direction:
0 → +30°

not found:
+30 → -60°

not found:
-60 → +90°

...

最大可进入完整360°扫描
```

不要读取 `/target/state` 来决定转哪边。

---

## 五、从未看到过目标时

这是与“丢失目标”不同的情况。

如果起飞完成后：

```text
has_ever_locked_target == false
```

没有“消失方向”可以用。

因此进入：

```text
TARGET_ACQUIRE_INITIAL
```

保持：

```text
XY hold
安全高度 hold
```

执行固定方向、低速 yaw sweep。

例如：

```yaml
initial_search_yaw_rate: 0.25
```

完整扫描最多一圈后继续循环，或进入 SAFE_WAIT。

严禁：

```text
通过 Gazebo USV pose 直接转向目标
```

---

## 六、TARGET_LOCK 不要单帧触发

不能：

```text
看到一帧 → 立即 MINCO
```

定义 lock gate。

至少要求：

```text
连续 N 帧有效 observation
```

建议初始参数：

```yaml
target_lock_min_frames: 3
target_lock_max_age: 0.15
target_lock_max_bearing: 0.15
```

锁定条件应包括：

```text
observation valid
measurement age 合格
连续帧满足
目标处于相机中央附近
KF 已初始化
KF state freshness 合格
```

达到后：

```text
TARGET_ACQUIRE
      ↓
TARGET_LOCK
      ↓
FAR_GUIDANCE
```

---

## 七、锁定后使用图像闭环让机头持续朝向目标

这里不要使用目标 truth heading。

增加视觉 yaw servo：

```text
image bearing error
        ↓
bounded proportional yaw controller
        ↓
yaw / yaw-rate reference
        ↓
PX4
```

例如：

```text
yaw_rate_cmd =
clamp(
    K_yaw * image_bearing,
    -max_observation_yaw_rate,
    +max_observation_yaw_rate
)
```

不要在第一版加入复杂 PID。

建议：

```yaml
vision_yaw_gain: 1.0
max_observation_yaw_rate: 保留现值
target_center_deadband_rad: 0.03
```

小误差进入 deadband：

```text
|bearing| < deadband
→ yaw rate = 0
```

避免机头抖动。

---

## 八、搜索控制与正常追踪控制的权限必须明确

定义 yaw ownership。

### SEARCH / REACQUIRE

```text
search controller owns yaw
trajectory/controller owns XYZ hold
```

### TARGET_LOCK / FAR_GUIDANCE / MINCO

```text
vision bearing controller owns yaw
trajectory tracker owns XYZ
```

不要让：

```text
trajectory tracker yaw
+
search yaw
+
vision yaw
```

三个地方同时发布互相竞争的命令。

代码中只能存在一个最终 yaw command arbiter。

---

## 九、目标暂时丢失后的行为

不要“一丢一帧立即搜索”。

新增：

```yaml
target_loss_frames: 3
target_reacquire_timeout: 2.0
```

短暂掉帧：

```text
1–2 frames
→ 保持当前预测/轨迹
```

确认丢失：

```text
连续 >= target_loss_frames
```

再进入 REACQUIRE。

---

## 十、不同飞行阶段的丢失策略必须不同

### FAR_GUIDANCE / PREPARATION

允许：

```text
hold XYZ
+
REACQUIRE yaw search
```

找到后：

```text
重新初始化/更新 KF
↓
BCTRA
↓
重新规划
```

### MINCO_ENTRY / MINCO_TRACKING

短暂丢失：

```text
允许在严格 stale limit 内使用已有 KF/BCTRA
```

超过：

```text
maximum state age
```

则：

```text
停止接受新 terminal plan
→ SAFE_WAIT / REACQUIRE
```

### TERMINAL_MINCO

这里不能简单悬停旋转，因为可能已经很靠近海面。

目标长时间丢失时：

```text
停止继续下降
→ sea-safe recovery/climb
→ 达到 recovery_clearance
→ REACQUIRE
```

搜索优先级必须低于海面安全。

---

## 十一、P8.3：统一状态新鲜度

当前重点检查：

```text
observation age
KF state age
prediction age
planner source age
trajectory age
```

统一定义 acquisition-time freshness。

禁止用：

```text
receipt time
```

伪装成 measurement time。

保持刚完成的 P8.1：

```text
PX4 timestamp
+ causal timesync offset
→ raw PX4 sim timestamp
→ Gazebo clock mapper
```

不要重新改 P8.1。

增加 diagnostics：

```text
target_visible
target_locked
search_state
search_direction
last_valid_observation_age
last_valid_image_bearing
search_yaw_rate_command
consecutive_valid_frames
consecutive_lost_frames
kf_state_age
prediction_age
```

---

## 十二、P8.5：处理当前约 0.5 m 水平系统偏差

最新日志已经表现为：

```text
raw horizontal RMSE ≈ 0.57 m
Y mean error ≈ -0.52 m
```

不要：

```text
固定 yaw correction
固定 XY correction
用 truth 在线标定
```

第一阶段只做诊断。

记录：

```text
image bearing
PX4 heading
PX4 attitude
target range
vision horizontal error（仅 evaluator offline）
```

分析：

```text
horizontal error
vs
range × heading/yaw error
```

目标是确认：

```text
PX4 yaw estimation bias
```

是否仍是主要来源。

搜索/视觉 yaw servo 解决的是：

```text
目标保持在 FOV 中
```

它不能被当成 EKF yaw bias 的数学修正。

---

## 十三、搜索功能对视觉检测器的要求

当前仍是红球 detector。

搜索逻辑不要直接依赖：

```text
Gazebo sphere pose
```

它只允许读取实际图像检测结果。

最好允许 detector 即使：

```text
depth invalid
```

但 RGB 仍检测到红目标时输出：

```text
bearing_valid = true
position_valid = false
```

因为搜索/机头对准只需要二维 image bearing，不一定要求完整 RGB-D 3D localization。

如果现有 `TargetObservation` 不适合表达，优先增加一个轻量 topic，例如：

```text
/perception/front/target_bearing
```

内容至少：

```text
stamp
valid
bearing
confidence
```

不要为了搜索复制整个 3D localizer。

以后 YOLO 替换红球 detector 后：

```text
YOLO bbox
→ target bearing
```

其余 search controller 可以原样复用。

---

## 十四、参数配置

所有搜索相关参数放进 `baseline.yaml`，不要硬编码。

建议新增：

```yaml
target_search_enable_height: 1.5

initial_search_yaw_rate: 0.25
reacquire_yaw_rate: 0.35
maximum_search_yaw_rate: 0.6

target_lock_min_frames: 3
target_loss_frames: 3

target_lock_max_age: 0.15
target_lock_max_bearing: 0.15
target_center_deadband_rad: 0.03

vision_yaw_gain: 1.0

target_reacquire_timeout: 2.0
search_initial_arc: 0.52
search_arc_increment: 0.52

terminal_loss_recovery_timeout: 0.3
```

这些是安全初值，不要为一次实验自动调参。

---

## 十五、必须增加的测试

至少完成以下测试：

```text
1. 起飞后目标一直可见
   TAKEOFF → TARGET_LOCK → FAR_GUIDANCE

2. 起飞过程中目标从图像右边消失
   → 向正确方向搜索
   → 找回
   → TARGET_LOCK

3. 从左边消失
   → 搜索方向相反

4. 从未看到目标
   → initial sweep
   → 禁止使用 truth

5. 单帧丢失
   → 不进入 REACQUIRE

6. 连续 target_loss_frames 丢失
   → REACQUIRE

7. 重新出现但只有一帧
   → 不 lock

8. 连续 N 帧且 bearing 合格
   → TARGET_LOCK

9. 搜索过程中 XYZ 保持

10. TARGET_LOCK 后视觉 bearing 控制 yaw

11. MINCO tracking 短暂丢失
    → bounded stale continuation

12. terminal 长时间丢失
    → 停止下降
    → safe recovery
    → reacquire

13. `/target/state` publisher 存在
    但在线节点全部不订阅

14. evaluator 删除/关闭时
    在线任务仍能正常运行

15. timesync reset
    不破坏搜索和 pose history

16. search yaw 与 tracker yaw 不产生双发布冲突
```

---

## 十六、验证顺序

不要一次直接跑复杂完整实验。

Phase A：

```text
静态 UAV
目标从视野左右移出
验证 search direction
```

Phase B：

```text
起飞
不启动 intercept
验证 ACQUIRE → LOCK
```

Phase C：

```text
起飞过程中人为让 USV 移出 FOV
验证自主 reacquire
```

Phase D：

```text
完全去 truth
完整闭环
```

Phase E：

运行不同非合作运动：

```text
直线
figure-eight
急转
S-turn
加减速
随机 bounded turn-rate
```

---

## 十七、最终验收条件

架构验收：

```text
/target/state
    → evaluator/logger only
```

运行时验收：

```text
目标初始不可见 → 能自主搜索
目标从左/右丢失 → 能沿合理方向寻找
重新出现 → 能稳定锁定
锁定后 → 机头持续视觉对准
视觉/KF → BCTRA → MINCO → PX4 全链闭环
无 truth fallback
```

时间验收：

```text
POSITION_TIMESTAMP_AFTER_HISTORY ≈ 0
vision valid rate ≥ 当前约95%的水平
```

安全验收：

```text
搜索期间不产生明显 XY 漂移
terminal loss 不继续盲目下降
无 SEA_CONTACT
```

非合作性验收：

控制器不知道：

```text
USV真实位置
USV真实速度
figure-eight参数
未来轨迹
```

只能获得：

```text
RGB-D / image bearing
PX4 EKF UAV pose
自身历史估计
KF target state
BCTRA prediction
```

---

## 十八、明确禁止的修改

本阶段不要：

```text
调 KF Q/R 去掩盖位置 bias
修改 MINCO 权重解决搜索问题
扩大 pose_wait_timeout
修改 P8.1 已验证时间链
固定补偿 yaw bias
读取 truth 决定搜索方向
读取 USV trajectory_type
读取 USV future trajectory
增加“视觉失败就用truth”的 fallback
```

完成后输出一份报告：

```text
1. truth audit结果
2. 删除/替换的所有 truth dependency
3. 新状态机图
4. yaw ownership
5. 搜索算法
6. terminal loss处理
7. 新参数
8. 新测试与结果
9. 完整闭环实验结果
10. 仍然存在的问题
```