# RGB acquisition time mapped to the ROS/system clock by server clock anchors.
# Zero means mapping unavailable; receipt time is never substituted.
builtin_interfaces/Time stamp
# Original RGB sensor acquisition stamp in the Gazebo simulation clock domain.
builtin_interfaces/Time raw_stamp
bool valid
# atan((target_u - camera_cx) / camera_fx), positive image-right / PX4 yaw.
float64 bearing
# Unit RGB ray rotated by the fixed camera mount into body FRD XY.
# atan2(body_right, body_forward); positive clockwise about body down.
# No depth, UAV attitude, target position, or truth is used.
float64 body_bearing
# Red detector support score in [0, 1]; zero on invalid readings.
float64 confidence
