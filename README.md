# UAV-USV interception workspace

这是无人机—无人艇感知、跟踪、预测、规划与控制实验的 ROS 2 Humble 工作空间。

## 快速使用

执行一键启动脚本：

```bash
cd /home/qin/data/uav_usv
./scripts/uav_lab.sh
```

脚本会在需要时自动启动 QGroundControl，然后编译工作空间并依次启动 Gazebo 海面世界、PX4 SITL、Micro XRCE-DDS Agent、移动目标和当前基准控制器。海面是无碰撞的视觉平面，原点处保留独立起飞平台。若 QGroundControl 不在默认位置，可先设置 `QGC_APPIMAGE`。命令台中：

- `X`：起飞并进入跟随模式；
- `Y`：进入截击模式；
- `Q`：退出命令台，已经打开的实验终端保持运行。

移动目标在节点启动后保持初始位置并发布零速度；收到X时与无人机起飞同步开始运动。起飞阶段采用Z轴位置控制和XY轴速度控制，无人机在爬升时同步跟随USV，不再原地等待爬升完成。

跟随模式保存USV已经走过的轨迹，并跟踪航迹弧长后方20米的历史位置，而不是当前航向切线后方的位置。这样8字转弯时跟随参考点仍以接近USV的速度沿原轨迹运动，不会横向快速甩动。

截击模式以20 Hz滚动重规划。规划器从连续目标速度观测估计转向率；远离目标时直接闭合当前USV位置并保持距海面1米的安全待机高度，进入可达域后在1–2秒范围内联合搜索末端时间和闭合速度，以五次多项式同时约束末端位置、速度与加速度，并执行轨迹上前视0.75秒的速度状态。候选轨迹只有在全时域满足速度、加速度、捕获半径和海面安全约束时才会执行，因此不再直接追逐6–8秒后的远期单点，也不会在水平截击不可达时提前触海。观测不足或目标速度过低时自动退回恒速度模型。CSV记录预测模型、转向率、完整轨迹可行性和轨迹约束峰值。

UAV在起飞、跟随、截击和结果悬停阶段都按 `atan2(target_y-uav_y, target_x-uav_x)` 计算期望偏航，并以默认1.5 rad/s的最大偏航速率连续转向USV。该观测偏航将作为后续相机和激光雷达共同视场约束的基础。

## 前视ToF试验

一键启动现在使用一台固定在机头的前视RGB-D相机模拟ToF设备。它同时输出对齐的彩色图像和逐像素浮点深度：RGB暂时通过红色目标球验证视场，深度图在目标掩膜内取有效像素中值作为目标表面距离。相机水平视场角为1.74 rad（约99.7度），640×480图像对应垂直视场角约83.3度，更新率20 Hz，有效深度范围0.2–25米。这些数值是针对当前8字航迹的仿真工程参数，并非Gao等（2024）论文中的相机标定值。

Gazebo RGB-D输出属于理想化几何深度，尚未模拟真实ToF在强日照、海面镜面反射、多径和低反射区域中的深度空洞。当前结果只能验证接口、视场和规划几何，不能替代真实海面传感器试验。

相机图像和诊断话题如下：

- `/camera/front/image_raw`：前视RGB图像；
- `/camera/front/depth/image_raw`：与RGB对齐的 `32FC1` 米制深度图；
- `/camera/front/camera_info`：由视场角生成的针孔模型参数；
- `/perception/usv_visible`：RGB画面是否看到当前红色目标球；
- `/perception/usv_tof_valid`、`/perception/usv_range`：目标区域是否有可靠深度以及深度中值；
- `/perception/usv_depth_valid_ratio`：目标掩膜内有效深度比例；
- `/perception/usv_visibility_rate`、`/perception/usv_tof_valid_rate`：最近5秒RGB可见率和ToF有效率。

当前阶段只验证“单个前视ToF能否稳定获取USV”。截击控制、卡尔曼滤波和成功/失败判定仍使用 `/target/*` 真值，不读取ToF结果。可用以下命令检查：

```bash
ros2 topic echo /perception/usv_visible
ros2 topic echo /perception/usv_tof_valid
ros2 topic echo /perception/usv_range
ros2 topic echo /perception/usv_visibility_rate
ros2 topic echo /perception/usv_tof_valid_rate
ros2 topic hz /camera/front/image_raw
ros2 topic hz /camera/front/depth/image_raw
```

先用完整起飞、跟随和截击实验观察最近5秒可见率与ToF有效率。若前视固定相机在近距离俯视阶段持续丢失目标，再增加第二台下视ToF，并参考论文的末端姿态条件切换；暂不因个别帧丢失直接增加硬件。后续取消全局信息时，红球颜色检测必须替换为非合作目标检测/分割；真值只保留在评价链路中，不能继续作为跟踪或规划输入。

只编译工作空间：

```bash
./scripts/build_workspace.sh
```

## 同步到 GitHub

修改完成后，用一句话说明本次变化并同步当前分支：

```bash
./scripts/sync_github.sh "改进有限时域截击与目标偏航控制"
```

脚本会显示待提交文件，确认后执行暂存、提交并推送到 `github` 远程仓库。提交正文自动记录文件数量、增删行统计和最多20个文件的状态。使用 `-n` 只预览，使用 `-y` 跳过确认；使用 `--auto` 时无需填写说明，脚本会根据时间和文件数量自动生成提交标题并直接同步：

```bash
./scripts/sync_github.sh -n "本次修改说明"
./scripts/sync_github.sh -y "本次修改说明"
./scripts/sync_github.sh --auto
```

## 目录

```text
uav_usv/
├── src/
│   ├── px4_msgs/             PX4 ROS 2 消息
│   ├── uav_control/          当前可运行节点与后续算法模块
│   ├── uav_usv_interfaces/   项目统一消息接口
│   └── uav_usv_bringup/      launch与实验参数
├── data/
│   ├── experiments/          CSV实验结果
│   ├── bags/                 rosbag数据
│   ├── datasets/             感知数据集
│   └── models/               检测或强化学习模型
├── docs/                     架构与迁移说明
└── scripts/                  编译和一键启动脚本
```

当前稳定基准保留为 `uav_control/trajectory_impact_sim.py`。后续算法放入 `perception`、`tracking`、`guidance`、`controllers` 和 `mission` 子目录，不再继续扩大单一节点。

## 截击结果

截击成功或失败后，节点在 `/simulation/impact/result` 发布 `uav_usv_interfaces/InterceptResult`。默认成功条件是在Y指令后的30秒内进入0.25米三维捕获半径；若无人机未先进入捕获半径就接触海面（NED `z >= 0`），结果为 `FAILURE / SEA_CONTACT`。两种结果都会冻结目标并暂停Gazebo。结果同时记录真实最小距离、水平距离、垂直误差、相对速度、闭合速度和运动约束峰值。CSV最终行的 `outcome` 与 `failure_reason` 给出本次实验结论。

终端轨迹使用与PX4指令一致的有效运动上限进行可行性检查；`terminal_min_closing_speed` 允许在期望3米/秒末端闭合速度不可达时逐级放宽，但不低于默认0.5米/秒。实际飞行硬限制仍由 `max_actual_horizontal_acceleration` 和 `max_actual_vertical_acceleration` 独立设置，持续超过实际硬限制才返回 `CONSTRAINT_VIOLATION`。

水平速度指令默认保留0.6米/秒的静态余量，因此8米/秒实际硬限制对应7.4米/秒指令上限。当PX4实际速度超过该软上限时，速度反馈调节器会进一步降低沿当前运动方向的指令，消除速度环滞后造成的持续超调；实际8米/秒硬限制和持续0.25秒的失败判定保持不变。

Gazebo控制还会在5米/平方秒实际水平加速度硬限制下保留0.5米/平方秒响应余量。截击规划与发送给PX4的有效加速度上限均为4.5米/平方秒，以吸收速度控制器的跟踪超调；跟随阶段仍使用2.5米/平方秒限制。

## 目标状态估计

统一launch同时启动恒速度卡尔曼滤波节点。节点使用 `/target/position` 估计目标位置和速度，在 `/tracking/target_state` 发布 `uav_usv_interfaces/TargetState`，并在 `/tracking/predicted_position` 发布默认0.5秒后的预测位置。当前截击控制仍使用真值状态输入，但规划模型已经支持转弯目标；滤波输出预留给后续传感器闭环，在误差验证完成后再切换控制输入。
