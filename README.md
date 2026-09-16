# UAV-USV interception workspace

这是无人机—无人艇感知、跟踪、预测、规划与控制实验的 ROS 2 Humble 工作空间。

项目目标、当前链路、主要参数和已知边界的简要说明见 [`docs/project_introduction.md`](docs/project_introduction.md)。

## 快速使用

执行一键启动脚本：

```bash
cd /home/qin/data/uav_usv
./scripts/uav_lab.sh
```

脚本会在需要时自动启动 QGroundControl，然后编译工作空间并依次启动 Gazebo 海面世界、PX4 SITL、Micro XRCE-DDS Agent、移动目标和当前基准控制器。启动过程中会把PX4的上升、下降速度限制同步为4米/秒；ROS侧仍以独立的起飞限制保持平缓起飞，并按 `terminal_dive_angle` 生成相对移动捕获点的45度几何下滑参考。实际轨迹仍受速度、加速度、视场和海面净空约束；这里不会强制机体俯仰45度。海面是无碰撞的视觉平面，原点处保留独立起飞平台。若 QGroundControl 不在默认位置，可先设置 `QGC_APPIMAGE`。命令台中：

- `X`：同时启动无人机起飞和无人艇运动，并进入跟随模式；
- `Y`：进入截击模式；
- `R`：安全停止本轮Gazebo、PX4、DDS和ROS实验进程，然后跳过编译并自动启动一轮全新仿真；
- `Q`：退出命令台，已经打开的实验终端保持运行。

移动目标在节点启动后保持初始位置并发布零速度。控制器先在地面连续发送保持点，自动进入Offboard但保持未解锁；一键脚本只有收到 `/simulation/impact/flight_ready=true` 才开放命令输入。收到X后PX4才解锁，无人机立即改变高度设定值，同时USV开始运动。起飞阶段采用Z轴位置控制和XY轴速度控制，无人机在爬升时同步跟随USV，不再把Offboard预发送和解锁延迟算入目标先跑时间。

跟随模式保存USV已经走过的轨迹，并跟踪航迹弧长后方5米的历史位置，而不是当前航向切线后方的位置。5米水平间距在5米高度时的斜距约7.1米，为25米ToF上限保留足够余量；单纯提高高度会增加斜距，不能解决远距离丢失。这样8字转弯时跟随参考点仍以接近USV的速度沿原轨迹运动，不会横向快速甩动。

截击控制保持20 Hz，目标预测保持20 Hz，MINCO以5 Hz触发并在单工作线程中运行；100 Hz完成轮询会在求解结束后及时发布最新结果。目标预测采用有界自适应CTRA：由连续速度观测在线估计转率、转率加速度和纵向加速度，并让加速度随预测时域衰减；它不读取目标预设航线，数据不足时自动退回恒速度模型。远距离 `FAR_GUIDANCE` 只负责进入三维可达域，45度下滑角仅作为初始几何偏好。规划器显式计算水平、垂向和海面制动最短时间，以其中最大值构造动态搜索区间，当前最大规划时域为4秒。首次获得稳定可行轨迹后锁定绝对 `contact_stamp`，后续重规划优先按 `remaining_t_go` 倒计时；剩余时间不超过1秒或三维距离不超过2米时进入 `TERMINAL_MINCO`，让tracker执行最后接触段。单次重规划失败时只短时执行仍有效的旧轨迹，旧轨迹超龄、目标三维偏移过大、剩余时间不足或海面余量不足时立即废弃并进入安全等待，不再切换到高速终端追逐。最终垂向指令在全部限幅之后还要经过独立海面Safety Guard。

UAV在起飞、跟随、截击和结果悬停阶段都按 `atan2(target_y-uav_y, target_x-uav_x)` 计算期望偏航，并以默认1.0 rad/s的最大偏航速率连续转向USV。该观测偏航将作为后续相机和激光雷达共同视场约束的基础。

当前MINCO后端参考[Wang等的GCOPTER](https://arxiv.org/abs/2103.00190)增加时空变形：在快速离散初值上，使用平滑映射保证中间点位于几何变形域、每段时间为正且总时域不变，并将水平/垂直速度、加速度与海面净空超限写成三次时间积分惩罚，再用L-BFGS-B调整分段时间和USV曲率引导点。优化后仍要通过硬上限复核；这是面向无障碍动态截击的轻量实现，不包含完整GCOPTER的障碍物安全飞行走廊和全部多旋翼平坦性约束。

移动目标使用ROS时钟实测回调间隔推进，延迟回调内部按标称周期子步积分，避免固定50毫秒步长与控制器实测时间轴不一致所造成的目标跳步和伪预测误差。

## 双ToF试验

一键启动现在使用两台刚性固定的RGB-D相机模拟ToF设备。前视相机位于机头前方并向下俯视12度，负责起飞、跟随和接近；下视相机光轴竖直向下，负责水平距离很小时的末端覆盖。两台相机都输出对齐的彩色图像和逐像素浮点深度：RGB暂时通过红色目标球验证视场，深度图在目标掩膜内取有效像素中值作为目标表面距离。水平视场角为1.74 rad（约99.7度），640×480图像对应垂直视场角约83.3度，更新率20 Hz，有效深度范围0.2–25米。这些数值是针对当前8字航迹的仿真工程参数，并非Gao等（2024）论文中的相机标定值。PX4模型初始朝向USV所在的NED北向，起飞后控制器继续按目标方位更新偏航。

海面模型的几何体是500×500米平面，并非球体；画面中的球面感主要来自宽视场透视和高光。此前红球中心位于海平面，天然有一半被不透明海面遮挡。现在只把临时可视化球心抬高0.25米，使其下表面对应USV真值参考点；控制、滤波、轨迹和成功判定使用的 `/target/*` 状态不变，下视相机也保留作为诊断备份。

Gazebo RGB-D输出属于理想化几何深度，尚未模拟真实ToF在强日照、海面镜面反射、多径和低反射区域中的深度空洞。当前结果只能验证接口、视场和规划几何，不能替代真实海面传感器试验。

相机图像和诊断话题如下：

- `/camera/front/image_raw`：前视RGB图像；
- `/camera/front/depth/image_raw`：与RGB对齐的 `32FC1` 米制深度图；
- `/camera/front/camera_info`：由视场角生成的针孔模型参数；
- `/camera/down/image_raw`、`/camera/down/depth/image_raw`、`/camera/down/camera_info`：下视相机对应数据；
- `/perception/front/*`、`/perception/down/*`：每台相机独立的可见性、ToF距离、有效率和真值视场诊断；
- `/perception/active_camera`：当前主诊断源。本轮单前视验证将 `allow_down_fallback` 设为 `false`，因此前视丢失时报告 `none`，不会用下视结果掩盖问题；下视独立话题仍持续记录。将参数改为 `true` 后恢复“前视丢失3帧切下视、前视恢复10帧切回”的迟滞备份；
- `/perception/usv_visible`：RGB画面是否看到当前红色目标球；
- `/perception/usv_red_pixel_count`：当前分析帧中的强红像素数，便于判断远距离小目标阈值；
- `/perception/usv_tof_valid`、`/perception/usv_range`：目标区域是否有可靠深度以及深度中值；
- `/perception/usv_depth_valid_ratio`：目标掩膜内有效深度比例；
- `/perception/front/target_observation`：前视RGB-D恢复的带置信度USV位置，坐标系为PX4本地NED；
- `/perception/front/target_position`：供卡尔曼滤波器使用的有效相机位置观测；
- `/perception/usv_visibility_rate`、`/perception/usv_tof_valid_rate`：最近5秒RGB可见率和ToF有效率。
- `/perception/camera_stream_alive`、`/perception/camera_frame_count`：分析流是否新鲜以及持续递增的帧计数，用于确认相机流没有冻结；
- `/perception/camera_frame_change`：相邻分析帧的归一化内容变化量；
- `/perception/target_truth_in_fov`、`/perception/target_horizontal_angle`、`/perception/target_vertical_angle`：真值评价得到的目标视场状态和水平/垂直角（弧度），只用于排障和评价。

当前阶段验证双ToF能否在完整航迹中互补覆盖USV。截击控制和成功/失败判定仍使用 `/target/*` 真值；前视RGB-D位置已经进入卡尔曼滤波影子链路，但不会改变控制指令。选择器只影响诊断输出。可用以下命令检查：

```bash
ros2 topic echo /perception/usv_visible
ros2 topic echo /perception/usv_tof_valid
ros2 topic echo /perception/usv_range
ros2 topic echo /perception/usv_visibility_rate
ros2 topic echo /perception/usv_tof_valid_rate
ros2 topic hz /camera/front/image_raw
ros2 topic hz /camera/front/depth/image_raw
ros2 topic hz /camera/down/image_raw
ros2 topic echo /perception/active_camera
ros2 topic echo /perception/camera_frame_change
ros2 topic echo /perception/camera_stream_alive
ros2 topic echo /perception/camera_frame_count
ros2 topic echo /perception/target_truth_in_fov
ros2 topic echo /perception/front/target_observation
ros2 topic echo /tracking/target_state
```

查看实时画面时应运行 `ros2 run rqt_image_view rqt_image_view`，跟随/接近阶段选择 `/camera/front/image_raw`，末端阶段同时观察 `/camera/down/image_raw`。图像由独立的 `ros_gz_image` 桥接器直接以传感器频率发布；RGB-D定位以10 Hz运行，前/下视诊断分析以5 Hz运行，图像QoS为BEST_EFFORT、KEEP_LAST、depth=1。可用 `enable_shadow_perception:=false` 关闭图像桥、定位、KF和相机诊断，进行同场景A/B测试；这一参数不会关闭20 Hz真值预测、控制或评价。海面和天空纹理近似均匀，若目标在视场外，飞机只做平移时画面可能肉眼近似不变；此时应结合每台相机的帧变化量、Gazebo real-time factor和真值视场话题判断。

先用完整起飞、跟随和截击实验比较前视、下视最近5秒可见率、ToF有效率和切换时刻。后续取消全局信息时，红球颜色检测必须替换为非合作目标检测/分割；真值只保留在评价链路中，不能继续作为跟踪或规划输入。

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

截击成功或失败后，节点在 `/simulation/impact/result` 发布 `uav_usv_interfaces/InterceptResult`。Evaluator以0.50米三维球作为实验成功判据，planner则瞄准0.35米内部区域；两个半径分别记录在config和summary中。若无人机未先进入捕获球就接触海面（NED `z >= 0`），结果为 `FAILURE / SEA_CONTACT`。SUCCESS、SEA_CONTACT和TIMEOUT都会触发独立Gazebo暂停请求（失败时短时重试），MissionManager同时进入终态并让tracker清除旧MINCO，只保留海面上方的安全保持点。`/simulation/impact/hit` 只表示SUCCESS，不表示是否已经发生终止事件。结果还记录真实最小距离、相对速度、规划/跟踪拒绝原因、接触时间倒计时及按目标距离分桶的运行性能。

终端轨迹使用与PX4指令一致的有效运动上限进行可行性检查；末端闭合速度、动态时域、有效垂向制动能力以及其他实验参数均以 [`baseline.yaml`](src/uav_usv_bringup/config/baseline.yaml) 和每次运行生成的 `*_config.yaml` 为准，不在本文重复维护数值。实际飞行硬限制仍由 `max_actual_horizontal_acceleration` 和 `max_actual_vertical_acceleration` 独立设置，持续超过实际硬限制才返回 `CONSTRAINT_VIOLATION`。

水平速度指令默认保留0.6米/秒的静态余量，因此8米/秒实际硬限制对应7.4米/秒指令上限。当PX4实际速度超过该软上限时，速度反馈调节器会进一步降低沿当前运动方向的指令，消除速度环滞后造成的持续超调；实际8米/秒硬限制和持续0.25秒的失败判定保持不变。

Gazebo控制还会在5米/平方秒实际水平加速度硬限制下保留0.5米/平方秒响应余量。截击规划与发送给PX4的有效加速度上限均为4.5米/平方秒，以吸收速度控制器的跟踪超调；跟随阶段仍使用2.5米/平方秒限制。

## 目标状态估计

统一launch同时启动RGB-D定位和恒速度卡尔曼滤波节点。定位器对仿真红球做颜色分割，在掩膜内关联深度，利用相机内参、安装外参和PX4姿态把目标恢复到本地NED；有效观测发布到 `/perception/front/target_position`。卡尔曼滤波器据此估计位置和速度，在 `/tracking/target_state` 发布 `uav_usv_interfaces/TargetState`，并在 `/tracking/predicted_position` 发布默认0.5秒后的预测位置。当前截击控制仍使用真值状态输入，滤波输出处于影子验证阶段。

实验CSV会同步记录相机位置、卡尔曼位置/速度及其相对真值误差。制导预测器和卡尔曼滤波器分别生成0.5、1.0和2.0秒预测；相应时域到期后，日志把历史预测与当时USV真值配对并记录误差。完成实验后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv/install/setup.bash
ros2 run uav_control usv_estimation_analysis \
  /home/qin/data/uav_usv/data/experiments/current/<实验日志.csv>
```

报告给出相机/KF有效率，以及当前定位、0.5秒、1秒和2秒预测的均值、RMSE、中位数、P95和最大误差。红色分割只用于验证当前仿真小球；面对现实中的无标记非合作USV，必须把颜色掩膜替换为外观检测或实例分割模型。
