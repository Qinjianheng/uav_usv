# UAV-USV模块化预测、MINCO规划与控制架构设计

日期：2026-09-15
状态：待用户书面复核后进入实施计划

## 1. 背景与问题

当前 `trajectory_impact_sim.py` 同时承担PX4 Offboard、任务状态机、目标预测、三维可达性、MINCO规划、轨迹跟踪、海面安全、成功判定和CSV记录。节点超过5500行，预测、规划和控制共享大量可变状态，难以独立测试，也使异步规划的输入时间与轨迹执行时间发生混淆。

2026-09-15 15:19实验提供了以下证据：

- 截击30.05秒后以 `TIMEOUT` 结束；
- 150次规划提交中只有1次成功；
- 成功规划耗时487.6毫秒，动态不可行任务的典型耗时约475毫秒；
- 4米/秒目标在487.6毫秒内移动约1.95米，超过0.5米端点偏移限制；
- 唯一成功轨迹在下一个20 Hz控制周期被判为 `TARGET_SHIFT`；
- 随后27.6秒处于 `SAFE_WAIT`，没有轨迹保持周期；
- 110次规划失败被归类为 `SEA_CLEARANCE`；
- 目标实际垂向运动是幅值0.15米的有界正弦运动，但现有预测按 `z + vz * t` 线性外推。

因此，仅把优化放入同一节点内的Python线程不能解决问题。新架构必须同时隔离实时控制、显式传递时间戳、减少在线搜索量，并把预测不确定性与规划失败原因分开记录。

## 2. 目标

1. 将目标预测、截击规划、轨迹跟踪、任务管理和真值评价拆成职责明确的ROS 2节点。
2. 保持控制回调20 Hz稳定运行，不在控制进程中执行SciPy优化或候选轨迹搜索。
3. 让每条预测和轨迹携带输入参考时间、生成时间和有效截止时间，拒绝到达即过期的规划结果。
4. 保留当前水平BCTRA结构，但使用不读取未来真值的有界垂向预测。
5. 将实时MINCO候选数从约120至220条降到不超过6条。
6. 保留速度、加速度、海面净空和0.25米三维捕获半径硬约束。
7. 保持仿真真值、感知估计和评价真值之间的边界可审计。
8. 允许后续只替换规划后端为C++，不改变上层ROS接口。

## 3. 非目标

- 本轮不把相机/KF直接接入稳定控制闭环；
- 不改变USV 4米/秒运动、8字轨迹和0.15米升沉幅值；
- 不改变0.25米成功判定半径；
- 不放宽UAV速度、加速度或海面安全硬限制；
- 不实现完整GCOPTER障碍物走廊、推力、姿态和角速度约束；
- 不删除现有 `trajectory_impact_sim.py` 或历史实验数据；
- 不以一次成功实验声明成功率得到验证。

## 4. 推荐架构

采用分阶段模块化ROS进程。第一阶段保留Python算法库，但预测器和规划器必须作为独立进程运行；若快速规划后端在固定回放集上的P95仍超过80毫秒，则只把规划节点后端迁移到C++。

```text
/target/* simulation truth ------> target_predictor_node
/tracking/target_state ----------> target_predictor_node（后续闭环）
                                      |
                                      v
                         /planning/target_prediction
                                      |
PX4 VehicleLocalPosition ------------+----> intercept_planner_node
mission state ------------------------+              |
                                                     v
                                      /planning/intercept_trajectory
                                      /planning/status
                                                     |
PX4 VehicleLocalPosition ----------------------------+----> trajectory_tracker_node
mission state ---------------------------------------+              |
                                                                    v
                                                         PX4 Offboard setpoint

/target/* truth + UAV truth + planning/status ----> intercept_evaluator_node
keyboard X/Y/R + health/status --------------------> mission_manager_node
```

这些节点作为独立可执行程序由launch启动，不使用ROS composable同进程部署，避免Python GIL重新形成控制与规划耦合。

## 5. 节点职责

### 5.1 `target_predictor_node`

职责：根据带时间戳的目标状态生成0至3秒预测轨迹和不确定性包络，默认采样间隔为0.1秒。

输入：

- 当前基线：`/target/position`、`/target/velocity`；
- 后续闭环：`/tracking/target_state`；
- 参数 `target_state_source` 必须取 `simulation_truth` 或 `tracking`，并写入输出消息。

输出：

- `/planning/target_prediction`；
- `/planning/prediction_status`。

限制：

- 预测节点不读取UAV状态、任务成功标志或未来目标真值；
- 水平面保留有界BCTRA；
- 垂向采用基于历史位置、速度和加速度的有界预测，预测均值和包络不得超过可配置的物理升沉范围；
- 当前仿真真值输入必须在消息中明确标记为 `simulation_truth`，不得伪装成传感器输出。

运行频率：20 Hz。预测计算P95目标低于2毫秒。

### 5.2 `intercept_planner_node`

职责：读取一致时间基准下的UAV状态和目标预测，执行三维可达性、接触几何和MINCO轨迹规划。

输入：

- `/planning/target_prediction`；
- `/fmu/out/vehicle_local_position`；
- `/mission/state`。

输出：

- `/planning/intercept_trajectory`，仅在轨迹通过硬约束时发布；
- `/planning/status`，每次规划都发布成功或明确失败原因。

规划器只保存最新输入。新的状态到达时覆盖尚未开始处理的旧状态，不建立FIFO规划队列。正在运行的求解不能被安全取消时，结果返回后按原始输入时间检查时效，过期结果直接丢弃。

第一阶段运行频率设为5 Hz。达到性能门槛后允许提高到10 Hz，但频率不能高于求解器稳定完成能力。

### 5.3 `trajectory_tracker_node`

职责：以20 Hz采样有效轨迹，加入位置反馈，执行命令限幅和最终海面Safety Guard，并发布PX4 Offboard设定值。

输入：

- `/planning/intercept_trajectory`；
- `/planning/status`；
- `/fmu/out/vehicle_local_position`；
- `/mission/state`；
- 与预测器相同来源的当前目标状态，仅用于轨迹端点时效复核和安全等待速度匹配；来源切换必须由同一个launch参数同时配置，不能由控制节点单独选择真值。

输出：

- `/fmu/in/offboard_control_mode`；
- `/fmu/in/trajectory_setpoint`；
- `/control/reference`；
- `/control/status`。

控制节点不得导入SciPy，不得调用目标预测器或MINCO优化器。Safety Guard必须位于所有速度和加速度限幅之后。

### 5.4 `mission_manager_node`

职责：处理X/Y/R/Q命令、PX4准备状态和任务阶段转换，不执行预测、优化或PX4控制律。

状态机：

```text
INIT -> GROUND_HOLD -> TAKEOFF -> FOLLOW -> FAR_GUIDANCE
     -> MINCO_READY -> MINCO_TRACKING -> TERMINAL_MINCO -> CAPTURE
                                      -> PLAN_RECOVERY -> SAFE_WAIT
任意飞行状态 -> ABORTED/FAILURE
```

只有通过当前状态复核的新鲜轨迹才能触发 `MINCO_READY`。仅收到求解器的历史成功结果不能锁存MINCO。进入 `MINCO_TRACKING` 后丢失轨迹时，控制器先执行仍有效的旧轨迹；旧轨迹无效后进入 `PLAN_RECOVERY` 或 `SAFE_WAIT`，不得切换到高速终端追逐。

### 5.5 `intercept_evaluator_node`

职责：使用仿真真值计算成功、触海、最小距离、相对速度、运动约束、预测误差、规划性能和运行工件。

输入：

- `/target/*`真值；
- UAV仿真/PX4状态；
- prediction、planning、control和mission状态话题。

输出：

- `/simulation/impact/result`；
- `/simulation/impact/hit`；
- CSV、`*_summary.json`、`*_config.yaml`。

评价节点不得向预测、规划或控制节点发布会改变控制行为的数据。这样可以从架构上阻止评价真值泄漏到未来感知闭环。

## 6. ROS接口

在 `uav_usv_interfaces` 中新增以下消息。

### 6.1 `PredictedTargetPoint.msg`

```text
builtin_interfaces/Duration time_from_start
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64[9] position_covariance
```

### 6.2 `TargetPrediction.msg`

```text
builtin_interfaces/Time source_stamp
builtin_interfaces/Time generated_stamp
uint64 prediction_id
string frame_id
string source
string model
builtin_interfaces/Time valid_until
PredictedTargetPoint[] points
bool valid
string invalid_reason
float64 compute_time
```

### 6.3 `PolynomialSegment.msg`

```text
builtin_interfaces/Duration duration
float64[18] coefficients
```

系数按 `x(c0..c5), y(c0..c5), z(c0..c5)` 排列。轨迹采样约定使用每段局部时间。

### 6.4 `InterceptTrajectory.msg`

```text
builtin_interfaces/Time uav_state_stamp
builtin_interfaces/Time prediction_source_stamp
builtin_interfaces/Time generated_stamp
builtin_interfaces/Time valid_until
uint64 plan_id
uint64 prediction_id
uint64 mission_id
string target_state_source
string frame_id
string planner_type
PolynomialSegment[] segments
geometry_msgs/Point target_contact
geometry_msgs/Vector3 terminal_velocity
float64 duration
float64 closing_speed
float64 max_horizontal_speed
float64 max_vertical_speed
float64 max_horizontal_acceleration
float64 max_vertical_acceleration
```

### 6.5 `PlannerStatus.msg`

```text
builtin_interfaces/Time uav_state_stamp
builtin_interfaces/Time prediction_source_stamp
builtin_interfaces/Time generated_stamp
uint64 request_id
uint64 prediction_id
uint64 mission_id
bool success
string reason
string detail
float64 compute_time
float64 prediction_age
float64 horizontal_min_time
float64 vertical_min_time
float64 sea_safe_min_time
float64 search_min_time
float64 search_max_time
uint32 candidates_checked
```

### 6.6 `MissionState.msg`

```text
builtin_interfaces/Time stamp
uint8 state
string state_name
uint64 mission_id
bool intercept_requested
bool completed
```

状态常量定义在消息文件中：`INIT=0`、`GROUND_HOLD=1`、`TAKEOFF=2`、`FOLLOW=3`、`FAR_GUIDANCE=4`、`MINCO_READY=5`、`MINCO_TRACKING=6`、`TERMINAL_MINCO=7`、`PLAN_RECOVERY=8`、`SAFE_WAIT=9`、`CAPTURE=10`、`FAILURE=11`、`ABORTED=12`。节点不得依赖自由文本比较完成状态转换。

## 7. 时间和有效性规则

ROS仿真时间用于状态对应和轨迹年龄；单调墙钟用于计算耗时和求解期限。两者不得混用。

每条规划结果必须保留：

- `uav_state_stamp`：规划所用UAV快照的ROS时间；
- `prediction_source_stamp`：规划所用预测输入的原始目标状态ROS时间；
- `generated_stamp`：求解完成的ROS时间；
- `compute_time`：单调墙钟测得的计算时间；
- `valid_until`：轨迹最晚接受或执行时刻。

轨迹接收后按 `max(now - uav_state_stamp, now - prediction_source_stamp)` 计算主年龄，同时分别检查两项输入均未过期，不能用较新的输入掩盖另一项陈旧输入。满足以下全部条件才可执行：

1. 轨迹年龄不超过125毫秒；
2. 当前UAV位置与轨迹传播后起点误差不超过0.30米；
3. 当前UAV速度误差不超过0.50米/秒；
4. 当前目标预测端点与轨迹接触点偏差不超过0.50米；
5. 剩余轨迹时间不少于0.20秒；
6. 海面制动余量为正；
7. `mission_id`、`prediction_id` 和坐标系一致。

125毫秒来自4米/秒目标和0.5米端点容差。后续若目标速度或容差变化，配置检查应保证 `maximum_plan_age <= endpoint_tolerance / target_speed_bound`。

## 8. 目标预测设计

水平预测保留当前有界BCTRA：估计转率、转率加速度和纵向加速度，并对加速度随时域衰减。0.5、1.0和2.0秒到期误差继续与真值配对，仅用于评价。

垂向预测不得直接无限延长当前 `vz`。第一阶段采用有界常加速度模型：

1. 从最近0.5至1.0秒目标状态历史估计垂向速度和加速度；
2. 对速度、加速度和预测位置应用物理包络；
3. 包络来自公开配置或在线观测范围，不读取未来轨迹相位；
4. 输出均值和位置不确定区间；
5. 规划器在预测均值不满足接触几何、但不确定区间仍与捕获可行区相交时，返回 `PREDICTION_UNCERTAIN`，而不是直接记为真实 `SEA_CLEARANCE`。

仿真基线允许使用已知的最大升沉幅值作为物理上限，但不能使用正弦相位或未来真值生成完美预测。正式感知闭环中应由USV尺寸、水线先验和在线观测更新该范围。

## 9. 快速MINCO规划设计

三段MINCO-T3和硬约束检查保留，但取消大规模笛卡尔积搜索。

每次实时规划最多评估6条候选：

- 时域候选：`t_required + margin`、上一条成功时域、动态上限，共2至3个去重值；
- 闭合速度：期望值和保守值，共1至2个；
- 曲率权重：默认使用上一条成功值，不再组成独立四值搜索维度。

实时主路径：

1. 计算水平、垂向和海面最短时间；
2. 检查预测接触几何；
3. 闭式生成三段MINCO初值；
4. 密集检查速度、加速度和净空；
5. 首个可行候选立即发布。

L-BFGS-B不再运行于每次实时规划。它作为1至2 Hz可选改良任务，仅在已有可行轨迹时运行，并受80毫秒墙钟预算约束。改良结果仍需通过时间和当前状态复核；迟到结果直接丢弃。

如果Python快速主路径在固定回放集上的P95超过80毫秒，则将 `intercept_planner_node` 的MINCO生成、约束检查和可选优化迁移到独立C++包。消息接口、状态机和测试标准保持不变。

## 10. QoS与并发

- UAV状态和目标预测：KeepLast(1)，BestEffort，短lifespan；
- 轨迹和规划状态：KeepLast(1)，Reliable，Volatile；
- 任务状态和最终结果：Reliable；
- CSV和终端日志不在控制回调内执行批量统计。

规划节点的订阅回调只更新最新快照。求解线程一次只运行一个任务；任务完成后直接读取最新快照决定是否开始下一次规划，不排队补算历史状态。

## 11. 配置所有权

仍使用一个 `baseline.yaml`，但参数按节点命名空间划分：

- `target_predictor_node`：预测模型、历史窗口、物理包络；
- `intercept_planner_node`：可达性、候选数、MINCO和求解预算；
- `trajectory_tracker_node`：反馈增益、命令限制和Safety Guard；
- `mission_manager_node`：状态转换和超时；
- `intercept_evaluator_node`：捕获半径、结果判定和日志目录。

同一物理限制不得由多个节点各自维护不同默认值。共享限制放入统一YAML锚点不可被ROS参数解析可靠支持，因此构建前测试必须比较相关参数值，并在启动时由每个节点输出实际值。

## 12. 错误处理

预测失败时，规划节点发布 `PREDICTION_STALE` 或具体无效原因，不生成轨迹。

规划失败必须区分：

- `STATE_STALE`；
- `PREDICTION_STALE`；
- `PREDICTION_UNCERTAIN`；
- `HORIZON_INSUFFICIENT`；
- `CAPTURE_GEOMETRY`；
- `DYNAMIC_LIMIT_HORIZONTAL`；
- `DYNAMIC_LIMIT_VERTICAL`；
- `MINCO_CONSTRUCTION_FAIL`；
- `OPTIMIZATION_FAIL`；
- `DEADLINE_EXCEEDED`；
- `PLAN_STALE_ON_ARRIVAL`；
- `SAFETY_REJECTED`。

轨迹跟踪节点不得根据一次规划失败立即改变任务状态。它持续执行仍有效的当前轨迹；轨迹失效后发布 `NO_VALID_PLAN`，由任务节点进入恢复或安全等待。

## 13. 测试与验收

### 13.1 单元测试

- 水平BCTRA和有界垂向预测；
- 正弦升沉历史在不知道未来相位时不产生无限垂向外推；
- 接触球与海面净空几何；
- 三维可达性和失败原因；
- 多项式消息序列化后轨迹采样一致；
- 轨迹年龄、UAV状态偏差和端点偏差校验；
- Safety Guard四态转换。

### 13.2 节点集成测试

- 延迟注入超过125毫秒的轨迹必须被拒绝，且不得触发 `MINCO_READY`；
- 规划节点忙碌期间控制节点保持20 Hz；
- 新状态覆盖等待中的旧状态，不形成任务积压；
- 预测节点停止后规划器输出 `PREDICTION_STALE`；
- 重启或R命令后旧 `mission_id` 轨迹不能被执行；
- 评价真值话题不能成为预测或控制节点输入。

### 13.3 性能门槛

- 预测节点计算P95小于2毫秒；
- Python快速MINCO主路径P95小于80毫秒，最大值小于100毫秒，为ROS传输和控制接收预留至少25毫秒；
- 轨迹跟踪回调P95小于10毫秒；
- 20 Hz控制周期超过75毫秒的比例小于1%；
- 规划任务队列长度始终不超过1；
- 可达状态回放集上，实时主路径可行轨迹生成率不低于80%。

这些指标只证明软件时效和可行性，不等同于Gazebo截击成功率。

### 13.4 仿真回归

1. 一次完整Gazebo回归，检查节点时序、命令、海面安全和运行工件；
2. 若发生 `SEA_CONTACT`，导出最后2秒垂向数据；
3. 固定配置运行10次，分别记录结果、最小距离、预测误差、规划成功率、失败原因和周期时延；
4. 只有完成10次后才能报告该固定配置下的观察成功比例。

## 14. 迁移顺序与回滚

1. 保留提交 `2e6c416` 作为对照基线；
2. 新增消息和纯数据转换测试；
3. 新增 `target_predictor_node`，使用历史CSV回放验证；
4. 新增快速 `intercept_planner_node`，使用冻结状态集做性能测试；
5. 新增 `trajectory_tracker_node`，进行延迟注入集成测试；
6. 新增 `mission_manager_node` 和 `intercept_evaluator_node`；
7. 新建模块化launch，与旧launch并存；
8. 模块化管线完成一次Gazebo回归后，再把实验脚本默认入口切换到新launch；
9. 保留旧入口作为短期回滚路径，直到10次固定配置实验完成。

迁移期间不得删除旧节点、历史CSV或 `data/videos/`。每一阶段独立提交，并运行聚焦测试、完整ROS构建和适当的节点集成测试。

## 15. 完成判据

设计实施完成必须同时满足：

- 预测、规划和轨迹跟踪运行在不同ROS进程；
- 控制节点不导入SciPy或执行MINCO搜索；
- 所有轨迹按原始输入时间计算年龄；
- 陈旧规划结果不能锁存MINCO状态；
- 垂向预测不再把有界升沉无限线性外推；
- 实时规划最多评估6条候选；
- 规划失败原因能够定位到输入、接触几何、水平/垂向动力学、求解器、时限或安全拒绝；
- 单元测试、ROS构建和延迟注入集成测试通过；
- 一次Gazebo回归和10次固定条件实验结果被明确区分并记录。
