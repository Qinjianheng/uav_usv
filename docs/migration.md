# 迁移记录

- 旧工作空间：`/home/qin/data/px4_ros2_ws`
- 新工作空间：`/home/qin/data/uav_usv`
- PX4源码仍位于：`/home/qin/Projects/PX4-Autopilot`
- 历史CSV迁移到：`data/experiments/baseline_legacy`
- `predictive_intercept_v1.py` 与 `pure_pursuit_backup.py` 移入 `uav_control/archive`
- 未迁移旧工作空间的 `build`、`install`、`log` 和Python缓存

旧工作空间不会自动删除，可在新工作空间完成编译和运行验证后再决定是否保留。

