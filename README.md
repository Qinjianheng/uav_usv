# UAV-USV interception workspace

这是无人机—无人艇感知、跟踪、预测、规划与控制实验的 ROS 2 Humble 工作空间。

## 快速使用

执行一键启动脚本：

```bash
cd /home/qin/data/uav_usv
./scripts/uav_lab.sh
```

脚本会在需要时自动启动 QGroundControl，然后编译工作空间并依次启动 Gazebo 海面世界、PX4 SITL、Micro XRCE-DDS Agent、移动目标和当前基准控制器。海面是无碰撞的视觉平面，原点处保留独立起飞平台。若 QGroundControl 不在默认位置，可先设置 `QGC_APPIMAGE`。命令台中：

- `X`：同时启动无人机起飞和无人艇运动，并进入跟随模式；
- `Y`：进入截击模式；
- `Q`：退出命令台，已经打开的实验终端保持运行。

移动目标在节点启动后保持初始位置并发布零速度。控制器先在地面连续发送保持点，自动进入Offboard并解锁；一键脚本只有收到 `/simulation/impact/flight_ready=true` 才开放命令输入。收到X后，无人机立即改变高度设定值，同时USV才开始运动。起飞阶段采用Z轴位置控制和XY轴速度控制，无人机在爬升时同步跟随USV，不再把Offboard预发送和解锁延迟算入目标先跑时间。

跟随模式保存USV已经走过的轨迹，并跟踪航迹弧长后方15米的历史位置，而不是当前航向切线后方的位置。15米水平间距在5米高度时的斜距约15.8米，为25米ToF上限保留足够余量；单纯提高高度会增加斜距，不能解决远距离丢失。这样8字转弯时跟随参考点仍以接近USV的速度沿原轨迹运动，不会横向快速甩动。

截击模式以20 Hz滚动重规划。规划器从连续目标速度观测估计转向率；距离USV超过12米时保持5米巡航高度并持续水平闭合，12至3米之间连续下降到约1米，近距离再按前视相机最大48.7度目标俯角继续降低高度。终端多项式暂时不可行时，追逐制动点是0.25米实际捕获邻域，而不是原来的3米终端规划边界，因此不会在3米外停住等待下降。进入可达域后在1–2秒范围内联合搜索末端时间和闭合速度，以五次多项式同时约束末端位置、速度与加速度，并执行轨迹上前视0.75秒的速度状态。候选轨迹只有在全时域满足速度、加速度、捕获半径和海面安全约束时才会执行。观测不足或目标速度过低时自动退回恒速度模型。CSV新增 `guidance_altitude_reference` 和 `guidance_closing_speed`，并继续记录预测模型、转向率、完整轨迹可行性和轨迹约束峰值。

UAV在起飞、跟随、截击和结果悬停阶段都按 `atan2(target_y-uav_y, target_x-uav_x)` 计算期望偏航，并以默认1.5 rad/s的最大偏航速率连续转向USV。该观测偏航将作为后续相机和激光雷达共同视场约束的基础。

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
- `/perception/usv_visibility_rate`、`/perception/usv_tof_valid_rate`：最近5秒RGB可见率和ToF有效率。
- `/perception/camera_stream_alive`、`/perception/camera_frame_count`：分析流是否新鲜以及持续递增的帧计数，用于确认相机流没有冻结；
- `/perception/camera_frame_change`：相邻分析帧的归一化内容变化量；
- `/perception/target_truth_in_fov`、`/perception/target_horizontal_angle`、`/perception/target_vertical_angle`：真值评价得到的目标视场状态和水平/垂直角（弧度），只用于排障和评价。

当前阶段验证双ToF能否在完整航迹中互补覆盖USV。截击控制、卡尔曼滤波和成功/失败判定仍使用 `/target/*` 真值，不读取ToF结果；选择器只影响诊断输出。可用以下命令检查：

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
```

查看实时画面时应运行 `ros2 run rqt_image_view rqt_image_view`，跟随/接近阶段选择 `/camera/front/image_raw`，末端阶段同时观察 `/camera/down/image_raw`。图像由独立的 `ros_gz_image` 桥接器直接以传感器频率发布；红球和深度分析以10 Hz限频运行，不再阻塞RQt刷新。海面和天空纹理近似均匀，若目标在视场外，飞机只做平移时画面可能肉眼近似不变；此时应结合每台相机的帧变化量和真值视场话题判断。

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

截击成功或失败后，节点在 `/simulation/impact/result` 发布 `uav_usv_interfaces/InterceptResult`。默认成功条件是在Y指令后的30秒内进入0.25米三维捕获半径；若无人机未先进入捕获半径就接触海面（NED `z >= 0`），结果为 `FAILURE / SEA_CONTACT`。两种结果都会冻结目标并暂停Gazebo。结果同时记录真实最小距离、水平距离、垂直误差、相对速度、闭合速度和运动约束峰值。CSV最终行的 `outcome` 与 `failure_reason` 给出本次实验结论。

终端轨迹使用与PX4指令一致的有效运动上限进行可行性检查；`terminal_min_closing_speed` 允许在期望3米/秒末端闭合速度不可达时逐级放宽，但不低于默认0.5米/秒。实际飞行硬限制仍由 `max_actual_horizontal_acceleration` 和 `max_actual_vertical_acceleration` 独立设置，持续超过实际硬限制才返回 `CONSTRAINT_VIOLATION`。

水平速度指令默认保留0.6米/秒的静态余量，因此8米/秒实际硬限制对应7.4米/秒指令上限。当PX4实际速度超过该软上限时，速度反馈调节器会进一步降低沿当前运动方向的指令，消除速度环滞后造成的持续超调；实际8米/秒硬限制和持续0.25秒的失败判定保持不变。

Gazebo控制还会在5米/平方秒实际水平加速度硬限制下保留0.5米/平方秒响应余量。截击规划与发送给PX4的有效加速度上限均为4.5米/平方秒，以吸收速度控制器的跟踪超调；跟随阶段仍使用2.5米/平方秒限制。

## 目标状态估计

统一launch同时启动恒速度卡尔曼滤波节点。节点使用 `/target/position` 估计目标位置和速度，在 `/tracking/target_state` 发布 `uav_usv_interfaces/TargetState`，并在 `/tracking/predicted_position` 发布默认0.5秒后的预测位置。当前截击控制仍使用真值状态输入，但规划模型已经支持转弯目标；滤波输出预留给后续传感器闭环，在误差验证完成后再切换控制输入。
