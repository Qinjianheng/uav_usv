"""ROS wrapper for the command-driven recoverable mission state machine."""

from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool, String
from uav_usv_interfaces.msg import ControllerDiagnostic, InterceptResult
from uav_usv_interfaces.msg import MissionState, PlannerDiagnostic

from .mission_manager import MissionManagerCore


def _seconds_to_time(value):
    value = max(float(value), 0.0)
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def mission_state_message(core, stamp_seconds):
    """Build the stable numeric state contract used by all other nodes."""
    message = MissionState()
    message.stamp = _seconds_to_time(stamp_seconds)
    message.mission_id = int(core.mission_id)
    message.state = int(core.phase)
    message.state_name = core.phase.name
    message.intercept_requested = bool(core.intercept_requested)
    message.completed = bool(core.completed)
    return message


class MissionManagerNode(Node):
    """Own X/Y/R/Q transitions without prediction, planning, or control."""

    def __init__(self):
        super().__init__('mission_manager_node')
        self.declare_parameter('publication_rate_hz', 20.0)
        self.declare_parameter('maximum_tracker_age', 0.125)
        self.declare_parameter('minimum_plan_remaining_time', 0.20)
        self.declare_parameter('plan_recovery_timeout', 0.50)
        publication_rate = float(
            self.get_parameter('publication_rate_hz').value
        )
        if publication_rate <= 0.0:
            raise ValueError('publication_rate_hz must be positive')
        self.core = MissionManagerCore(
            maximum_tracker_age=self.get_parameter(
                'maximum_tracker_age'
            ).value,
            minimum_plan_remaining_time=self.get_parameter(
                'minimum_plan_remaining_time'
            ).value,
            plan_recovery_timeout=self.get_parameter(
                'plan_recovery_timeout'
            ).value,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.command_sub = self.create_subscription(
            String,
            '/simulation/impact/command',
            self.command_callback,
            event_qos,
        )
        self.flight_ready_sub = self.create_subscription(
            Bool,
            '/simulation/impact/flight_ready',
            self.flight_ready_callback,
            state_qos,
        )
        self.takeoff_complete_sub = self.create_subscription(
            Bool,
            '/control/takeoff_complete',
            self.takeoff_complete_callback,
            event_qos,
        )
        self.far_guidance_sub = self.create_subscription(
            Bool,
            '/control/far_guidance_available',
            self.far_guidance_callback,
            event_qos,
        )
        self.controller_sub = self.create_subscription(
            ControllerDiagnostic,
            '/control/diagnostic',
            self.controller_callback,
            event_qos,
        )
        self.planner_sub = self.create_subscription(
            PlannerDiagnostic,
            '/planning/diagnostic',
            self.planner_callback,
            event_qos,
        )
        self.result_sub = self.create_subscription(
            InterceptResult,
            '/simulation/impact/result',
            self.result_callback,
            state_qos,
        )
        self.state_pub = self.create_publisher(
            MissionState,
            '/mission/state',
            state_qos,
        )
        self.timer = self.create_timer(
            1.0 / publication_rate,
            self.timer_callback,
        )
        self.far_guidance_available = False
        self.get_logger().info(
            'Mission manager ready | tracker age<='
            f'{self.core.maximum_tracker_age:.3f} s | recovery timeout='
            f'{self.core.plan_recovery_timeout:.2f} s'
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self):
        self.state_pub.publish(mission_state_message(self.core, self._now()))

    def command_callback(self, message):
        command = message.data.strip().upper()
        accepted = self.core.handle_command(command, self._now())
        if accepted:
            self.get_logger().info(
                f'{command} accepted | mission={self.core.mission_id} | '
                f'state={self.core.phase.name}'
            )
            self._publish()
        else:
            self.get_logger().warn(
                f'{command!r} rejected in {self.core.phase.name}'
            )

    def flight_ready_callback(self, message):
        if self.core.set_flight_ready(bool(message.data), self._now()):
            self._publish()

    def takeoff_complete_callback(self, message):
        if message.data and self.core.mark_takeoff_complete(self._now()):
            self._publish()

    def far_guidance_callback(self, message):
        self.far_guidance_available = bool(message.data)

    def controller_callback(self, message):
        accepted = self.core.observe_tracker(
            mission_id=message.mission_id,
            plan_id=message.plan_id,
            status=message.status,
            source_age=message.source_age,
            remaining_time=message.remaining_time,
            now=self._now(),
        )
        if accepted:
            self._publish()

    def planner_callback(self, message):
        self.core.observe_planner(
            success=(
                message.result == PlannerDiagnostic.RESULT_SUCCESS
                and message.failure_reason == PlannerDiagnostic.NONE
            ),
            plan_id=message.plan_id,
            now=self._now(),
        )

    def result_callback(self, message):
        if not message.outcome:
            return
        if message.mission_id not in (0, self.core.mission_id):
            return
        if message.success:
            self.core.mark_capture(self._now())
        else:
            self.core.mark_failure(self._now())
        self._publish()

    def timer_callback(self):
        self._publish()
        self.core.tick(
            self._now(),
            far_guidance_available=self.far_guidance_available,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
