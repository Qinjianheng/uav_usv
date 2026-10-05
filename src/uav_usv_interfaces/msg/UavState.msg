# Physical PX4 sample time, causally mapped with Gazebo sim/system anchors.
# Receipt/publication time is never substituted for this stamp.
builtin_interfaces/Time stamp
string frame_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 heading
bool valid
# Original DDS-translated timestamp_sample, preserved for clock audits.
uint64 px4_timestamp_sample
# Recovered native simulation sample time before DDS offset conversion.
uint64 native_timestamp_sample
