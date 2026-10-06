import array
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String, Bool
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from uav_usv_interfaces.msg import (
    ControllerDiagnostic, PlannerDiagnostic, MissionState, TargetPrediction,
    InterceptTrajectory, TargetState, TargetBearing, TargetObservation,
    InterceptResult, UavState,
)

def plain(v):
    if isinstance(v, dict):
        return {k: plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, array.array)):
        return [plain(x) for x in v]
    if hasattr(v, 'tolist'):
        return v.tolist()
    return v

class Recorder(Node):
    def __init__(self):
        super().__init__('y_intercept_evidence_recorder')
        self.output = Path(sys.argv[1]).open('w', buffering=1024 * 1024)
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.subs = []
        self.last = {}
        self.mission = None
        self.control = None
        self.follow_since = None
        self.required_follow = float(next((arg.split('=', 1)[1] for arg in sys.argv
                                           if arg.startswith('--follow-seconds=')), '20'))
        self.y_time = None
        self.result_time = None
        self.started = time.monotonic()
        self.auto_x = '--auto-x' in sys.argv
        self.require_settled_follow = '--settled-follow' in sys.argv
        self.navigation = self.kf = None
        self.follow_metrics = {}
        self.flight_ready = False
        self.x_sent = False
        self.x_ready_since = None
        self.x_pulses = 0
        self.pub = self.create_publisher(String, '/simulation/impact/command', 10)
        topics = [
            ('mission', MissionState, '/mission/state'),
            ('controller', ControllerDiagnostic, '/control/diagnostic'),
            ('planner', PlannerDiagnostic, '/planning/diagnostic'),
            ('prediction', TargetPrediction, '/planning/target_prediction'),
            ('trajectory', InterceptTrajectory, '/planning/intercept_trajectory'),
            ('kf', TargetState, '/tracking/target_state'),
            ('truth_evaluation_only', TargetState, '/target/state'),
            ('navigation', UavState, '/navigation/uav_state'),
            ('pose', VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1'),
            ('reference', TrajectorySetpoint, '/control/reference'),
            ('bearing', TargetBearing, '/perception/front/target_bearing'),
            ('observation', TargetObservation, '/perception/front/target_observation'),
            ('result', InterceptResult, '/simulation/impact/result'),
        ]
        for name, typ, topic in topics:
            self.subs.append(self.create_subscription(
                typ, topic, lambda msg, name=name: self.receive(name, msg), self.qos))
        self.timer = self.create_timer(0.5, self.tick)
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs.append(self.create_subscription(
            Bool, '/simulation/impact/flight_ready',
            lambda msg: setattr(self, 'flight_ready', msg.data), ready_qos))
        self.command_timer = self.create_timer(0.1, self.auto_start)

    def auto_start(self):
        ready = (self.flight_ready and self.mission is not None
                 and self.mission.state_name == 'GROUND_HOLD'
                 and self.pub.get_subscription_count() >= 2)
        if not ready:
            self.x_ready_since = None
        elif self.x_ready_since is None:
            self.x_ready_since = time.monotonic()
        if (self.auto_x and not self.x_sent and ready
                and time.monotonic() - self.x_ready_since >= 10):
            self.x_sent = True
            self.x_pulses = 3
        if self.x_pulses:
            self.pub.publish(String(data='X'))
            self.x_pulses -= 1
            self.write('driver', {'command': 'X', 'remaining_pulses': self.x_pulses})
            print('X SENT', self.x_pulses, flush=True)

    def write(self, topic, message):
        self.output.write(json.dumps({'topic': topic,
                         'receipt': self.get_clock().now().nanoseconds * 1e-9,
                         'message': message}, separators=(',', ':')) + '\n')

    def receive(self, name, msg):
        now = time.monotonic()
        if name == 'pose' and now - self.last.get(name, 0) < .045:
            return
        self.last[name] = now
        self.write(name, plain(message_to_ordereddict(msg)))
        if name == 'mission':
            if self.mission is None or msg.state_name != self.mission.state_name:
                print('PHASE', msg.state_name, 'Y elapsed', None if self.y_time is None else round(now-self.y_time, 3), flush=True)
            self.mission = msg
        elif name == 'controller':
            self.control = msg
        elif name == 'navigation':
            self.navigation = msg
        elif name == 'kf':
            self.kf = msg
        elif name == 'planner' and self.y_time is not None:
            print('PLAN', msg.plan_id, msg.prediction_sequence_id,
                  msg.failure_detail, msg.rejection_stage, msg.rejection_detail,
                  'age', round(msg.input_age_at_start, 4), round(msg.input_age_at_finish, 4),
                  round(msg.input_age_at_publish, 4), 't_go', round(msg.selected_t_go, 3), flush=True)
        elif name == 'result' and self.y_time is not None:
            print('RESULT', plain(message_to_ordereddict(msg)), flush=True)
            self.result_time = now

    def settled_follow(self):
        """Test-only Y trigger from fresh estimated states, never target truth."""
        uav, target = self.navigation, self.kf
        if uav is None or target is None or not uav.valid or not target.valid:
            return False
        if uav.frame_id != 'local_ned' or target.frame_id != uav.frame_id:
            return False
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        for stamp in (uav.stamp, target.stamp, target.source_stamp):
            age = ros_now - (stamp.sec + stamp.nanosec * 1e-9)
            if stamp.sec <= 0 or not 0 <= age <= .125:
                return False
        speed = math.hypot(target.velocity.x, target.velocity.y)
        if speed < .1:
            return False
        desired_x = target.position.x - 5. * target.velocity.x / speed
        desired_y = target.position.y - 5. * target.velocity.y / speed
        metrics = {
            'follow_position_error': math.hypot(uav.position.x-desired_x,
                                               uav.position.y-desired_y),
            'horizontal_distance': math.hypot(uav.position.x-target.position.x,
                                              uav.position.y-target.position.y),
            'relative_horizontal_speed': math.hypot(uav.velocity.x-target.velocity.x,
                                                    uav.velocity.y-target.velocity.y),
            'height': -uav.position.z,
            'vertical_speed': uav.velocity.z,
        }
        self.follow_metrics = metrics
        return (all(math.isfinite(v) for v in metrics.values())
                and metrics['follow_position_error'] <= 1.
                and metrics['relative_horizontal_speed'] <= 1.
                and abs(metrics['height']-5.) <= .35
                and abs(metrics['vertical_speed']) <= .35)

    def tick(self):
        now = time.monotonic()
        ready = (self.mission is not None and self.mission.state_name == 'FOLLOW'
                 and self.control is not None and self.control.target_visible
                 and self.control.target_locked
                 and now-self.last.get('controller', 0) < .2)
        if self.require_settled_follow:
            ready = ready and self.settled_follow()
        if self.y_time is None:
            if ready and self.follow_since is None:
                self.follow_since = now
            elif not ready:
                self.follow_since = None
            if self.follow_since is not None and now-self.follow_since >= self.required_follow:
                msg = String(data='Y')
                self.pub.publish(msg)
                self.y_time = now
                self.write('driver', {'command': 'Y', 'continuous_locked_follow_seconds': now-self.follow_since,
                                      'settled_follow_required': self.require_settled_follow,
                                      'entry_metrics': self.follow_metrics})
                print('Y SENT after stable FOLLOW', flush=True)
        self.output.flush()
        if (self.result_time is not None and now-self.result_time > 2
                or self.y_time is not None and now-self.y_time > 45
                or now-self.started > 240):
            self.output.close()
            raise SystemExit(0)

rclpy.init()
node = Recorder()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, SystemExit):
    pass
finally:
    if not node.output.closed:
        node.output.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
