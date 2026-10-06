"""Read-only terminal freeze evidence from native Gazebo and ROS target state."""
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from uav_usv_interfaces.msg import TargetState
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_stats_pb2 import WorldStatistics

rclpy.init()
node = Node('terminal_freeze_evidence')
target, stats, poses = [], [], []
node.create_subscription(TargetState, '/target/state', lambda m: target.append({
    'receipt': time.monotonic(), 'position': [m.position.x, m.position.y, m.position.z],
    'velocity': [m.velocity.x, m.velocity.y, m.velocity.z]}), 10)
gz = GzNode()
gz.subscribe(WorldStatistics, '/world/default/stats', lambda m: stats.append({
    'receipt': time.monotonic(), 'paused': m.paused,
    'sim_time': m.sim_time.sec + m.sim_time.nsec * 1e-9, 'iterations': m.iterations}))

def pose_callback(message):
    models = {p.name: [p.position.x, p.position.y, p.position.z,
                      p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
              for p in message.pose
              if p.name == 'usv_target' or p.name.startswith('x500_mono_cam')}
    if models:
        poses.append({'receipt': time.monotonic(),
                      'sim_stamp': message.header.stamp.sec
                      + message.header.stamp.nsec * 1e-9, 'models': models})

gz.subscribe(Pose_V, '/world/default/pose/info', pose_callback)
start = time.monotonic()
pause_at = None
while time.monotonic() - start < 50:
    rclpy.spin_once(node, timeout_sec=.05)
    if stats and stats[-1]['paused']:
        if pause_at is None:
            pause_at = time.monotonic()
        if time.monotonic() - pause_at > 7:
            break
steady = lambda values: [x for x in values if pause_at is not None
                         and x['receipt'] >= pause_at + .5]
target, stats, poses = steady(target), steady(stats), steady(poses)
report = {'role': 'READ_ONLY_TERMINAL_FREEZE_EVIDENCE', 'pause_seen': pause_at is not None,
          'target_samples': target, 'world_stats_samples': stats,
          'native_pose_snapshots': [poses[0], poses[-1]] if poses else []}
report['target_position_frozen'] = bool(target) and all(
    x['position'] == target[0]['position'] for x in target)
report['target_velocity_zero'] = bool(target) and all(
    x['velocity'] == [0., 0., 0.] for x in target)
report['world_clock_frozen'] = bool(stats) and all(
    x['paused'] and x['sim_time'] == stats[0]['sim_time'] for x in stats)
report['native_uav_target_poses_frozen'] = bool(poses) and all(
    x['models'] == poses[0]['models'] for x in poses)
Path(sys.argv[1]).write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: v for k, v in report.items() if not k.endswith('samples')}, indent=2))
print('target/world stats/native pose counts:', len(target), len(stats), len(poses))
node.destroy_node()
rclpy.shutdown()
