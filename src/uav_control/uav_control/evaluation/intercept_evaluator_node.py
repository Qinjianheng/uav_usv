"""ROS node for truth-only evaluation, event metrics, and run artifacts."""

import math
import statistics

from builtin_interfaces.msg import Time
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from uav_usv_interfaces.msg import ControllerDiagnostic, InterceptResult
from uav_usv_interfaces.msg import MissionState, PlannerDiagnostic
from uav_usv_interfaces.msg import TargetPrediction, TargetState

from uav_control.tracking.prediction_error_tracker import PredictionErrorTracker

from .intercept_evaluator import ExperimentArtifactWriter
from .intercept_evaluator import InterceptEvaluatorCore, KinematicState
from .intercept_evaluator import PlannerEventAccumulator


PREDICTION_HORIZONS = (0.5, 1.0, 2.0)


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _seconds_to_time(value):
    value = max(float(value), 0.0)
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def _finite_tuple(values, label):
    values = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{label} contains non-finite values')
    return values


def truth_from_message(message):
    """Convert an explicitly truth-topic TargetState for evaluation only."""
    if not message.valid:
        raise ValueError('simulation truth state is invalid')
    return KinematicState(
        position=_finite_tuple((
            message.position.x,
            message.position.y,
            message.position.z,
        ), 'target position'),
        velocity=_finite_tuple((
            message.velocity.x,
            message.velocity.y,
            message.velocity.z,
        ), 'target velocity'),
    )


def uav_from_message(message):
    """Convert PX4 local state in unchanged NED coordinates."""
    return KinematicState(
        position=_finite_tuple((message.x, message.y, message.z), 'UAV position'),
        velocity=_finite_tuple((message.vx, message.vy, message.vz), 'UAV velocity'),
    )


def result_to_message(result, stamp_seconds, radius):
    """Serialize one terminal truth-evaluation result."""
    message = InterceptResult()
    message.stamp = _seconds_to_time(stamp_seconds)
    message.mission_id = int(result.mission_id)
    message.success = bool(result.success)
    message.outcome = str(result.outcome)
    message.reason = str(result.reason)
    message.detail = 'determined by simulation truth in evaluator only'
    message.elapsed_time = float(result.elapsed_time)
    message.capture_radius = float(radius)
    message.minimum_distance = float(result.minimum_distance)
    message.horizontal_distance = float(result.horizontal_distance)
    message.vertical_error = float(result.vertical_error)
    message.relative_speed = float(result.relative_speed)
    message.closing_speed = float(result.closing_speed)
    message.max_horizontal_speed = float(result.maximum_horizontal_speed)
    message.max_vertical_speed = float(result.maximum_vertical_speed)
    message.max_horizontal_acceleration = float(
        result.maximum_horizontal_acceleration
    )
    message.max_vertical_acceleration = float(
        result.maximum_vertical_acceleration
    )
    return message


def _prediction_at(message, horizon):
    samples = list(message.samples)
    if not samples:
        raise ValueError('prediction has no samples')
    horizon = float(horizon)
    if horizon <= _duration_seconds(samples[0].relative_time):
        point = samples[0].position
        return float(point.x), float(point.y), float(point.z)
    for left, right in zip(samples, samples[1:]):
        left_time = _duration_seconds(left.relative_time)
        right_time = _duration_seconds(right.relative_time)
        if horizon <= right_time:
            scale = (horizon - left_time) / max(right_time - left_time, 1e-9)
            return tuple(
                float(getattr(left.position, axis))
                + scale * (
                    float(getattr(right.position, axis))
                    - float(getattr(left.position, axis))
                )
                for axis in ('x', 'y', 'z')
            )
    point = samples[-1].position
    return float(point.x), float(point.y), float(point.z)


class InterceptEvaluatorNode(Node):
    """Observe truth and diagnostics without feeding any control input."""

    def __init__(self):
        super().__init__('intercept_evaluator_node')
        self.declare_parameter('evaluation_rate_hz', 20.0)
        self.declare_parameter('capture_radius', 0.25)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('enable_sea_contact_failure', True)
        self.declare_parameter('maximum_duration', 30.0)
        self.declare_parameter(
            'log_directory',
            'data/experiments/current',
        )
        self.declare_parameter('truth_topic', '/target/state')
        rate = float(self.get_parameter('evaluation_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('evaluation_rate_hz must be positive')
        self.capture_radius = float(
            self.get_parameter('capture_radius').value
        )
        self.evaluator = InterceptEvaluatorCore(
            capture_radius=self.capture_radius,
            sea_surface_z=self.get_parameter('sea_surface_z').value,
            enable_sea_contact_failure=self.get_parameter(
                'enable_sea_contact_failure'
            ).value,
            maximum_duration=self.get_parameter('maximum_duration').value,
        )
        self.log_directory = str(
            self.get_parameter('log_directory').value
        )
        self.truth_topic = str(self.get_parameter('truth_topic').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        result_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.uav_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.uav_callback,
            sensor_qos,
        )
        self.truth_sub = self.create_subscription(
            TargetState,
            self.truth_topic,
            self.truth_callback,
            sensor_qos,
        )
        self.filtered_target_sub = self.create_subscription(
            TargetState,
            '/tracking/target_state',
            self.filtered_target_callback,
            sensor_qos,
        )
        self.prediction_sub = self.create_subscription(
            TargetPrediction,
            '/planning/target_prediction',
            self.prediction_callback,
            sensor_qos,
        )
        self.planner_sub = self.create_subscription(
            PlannerDiagnostic,
            '/planning/diagnostic',
            self.planner_callback,
            event_qos,
        )
        self.controller_sub = self.create_subscription(
            ControllerDiagnostic,
            '/control/diagnostic',
            self.controller_callback,
            event_qos,
        )
        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_callback,
            result_qos,
        )
        self.result_pub = self.create_publisher(
            InterceptResult,
            '/simulation/impact/result',
            result_qos,
        )
        self.hit_pub = self.create_publisher(
            Bool,
            '/simulation/impact/hit',
            result_qos,
        )
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.latest_uav = None
        self.latest_truth = None
        self.latest_mission = None
        self.latest_controller = None
        self.latest_tracker_rejection_reason = ''
        self.event_metrics = PlannerEventAccumulator()
        self.prediction_tracker = PredictionErrorTracker(PREDICTION_HORIZONS)
        self.prediction_sequences = set()
        self.prediction_errors = {
            (model, horizon): []
            for model in ('guidance', 'kf')
            for horizon in PREDICTION_HORIZONS
        }
        self.latest_prediction_error = {}
        self.controller_compute_times = []
        self.writer = None
        self.result_published = False
        self.get_logger().info(
            'Truth-only evaluator ready | truth='
            f'{self.truth_topic} | capture={self.capture_radius:.2f} m | '
            f'output={self.log_directory}'
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def uav_callback(self, message):
        try:
            self.latest_uav = uav_from_message(message)
        except ValueError:
            self.latest_uav = None

    def truth_callback(self, message):
        try:
            self.latest_truth = truth_from_message(message)
        except ValueError:
            self.latest_truth = None
            return
        truth_stamp = _stamp_seconds(message.stamp) or self._now()
        for model in ('guidance', 'kf'):
            for horizon in PREDICTION_HORIZONS:
                evaluation = self.prediction_tracker.evaluate(
                    model,
                    horizon,
                    truth_stamp,
                    self.latest_truth.position,
                )
                if evaluation is not None:
                    self.prediction_errors[(model, horizon)].append(
                        evaluation.error
                    )
                    self.latest_prediction_error[(model, horizon)] = (
                        evaluation.error
                    )

    def filtered_target_callback(self, message):
        if not message.valid:
            return
        stamp = _stamp_seconds(message.stamp) or self._now()
        try:
            position = _finite_tuple((
                message.position.x,
                message.position.y,
                message.position.z,
            ), 'filtered target position')
            velocity = _finite_tuple((
                message.velocity.x,
                message.velocity.y,
                message.velocity.z,
            ), 'filtered target velocity')
        except ValueError:
            return
        predictions = {
            horizon: tuple(
                value + rate * horizon
                for value, rate in zip(position, velocity)
            )
            for horizon in PREDICTION_HORIZONS
        }
        self.prediction_tracker.add('kf', stamp, predictions)

    def prediction_callback(self, message):
        key = int(message.mission_id), int(message.sequence_id)
        if not message.valid or key in self.prediction_sequences:
            return
        self.prediction_sequences.add(key)
        try:
            predictions = {
                horizon: _prediction_at(message, horizon)
                for horizon in PREDICTION_HORIZONS
                if horizon <= float(message.prediction_horizon) + 1e-9
            }
        except (TypeError, ValueError):
            return
        self.prediction_tracker.add(
            'guidance',
            _stamp_seconds(message.source_stamp),
            predictions,
        )

    @staticmethod
    def _failure_name(message):
        names = {
            value: name
            for name, value in PlannerDiagnostic.__dict__.items()
            if name.isupper() and isinstance(value, int)
        }
        return names.get(int(message.failure_reason), str(message.failure_reason))

    def planner_callback(self, message):
        self.event_metrics.observe_planner(
            mission_id=message.mission_id,
            plan_id=message.plan_id,
            success=message.result == PlannerDiagnostic.RESULT_SUCCESS,
            failure_reason=self._failure_name(message),
            compute_time=message.compute_time,
            generation_time=message.generation_time,
            completion_stamp=_stamp_seconds(message.generated_stamp),
            input_age_at_publish=message.input_age_at_publish,
            completion_to_publish_delay=(
                message.completion_to_publish_delay
            ),
        )

    def controller_callback(self, message):
        self.latest_controller = message
        self.event_metrics.observe_controller(
            message.mission_id,
            message.plan_id,
            message.status,
            message.rejection_reason,
        )
        if message.status == 'PLAN_REJECTED':
            self.latest_tracker_rejection_reason = (
                str(message.rejection_reason)
            )
        self.controller_compute_times.append(
            max(float(message.callback_compute_time), 0.0)
        )

    def _config_snapshot(self):
        return {
            'capture_radius': self.capture_radius,
            'sea_surface_z': self.evaluator.sea_surface_z,
            'enable_sea_contact_failure': (
                self.evaluator.enable_sea_contact_failure
            ),
            'maximum_duration': self.evaluator.maximum_duration,
            'truth_topic': self.truth_topic,
            'truth_role': 'evaluation_only',
        }

    def _start_mission(self, mission_id, now):
        self.evaluator.begin(mission_id, now)
        self.event_metrics = PlannerEventAccumulator()
        self.prediction_tracker.reset()
        self.prediction_sequences.clear()
        for values in self.prediction_errors.values():
            values.clear()
        self.latest_prediction_error.clear()
        self.controller_compute_times.clear()
        self.latest_tracker_rejection_reason = ''
        self.writer = ExperimentArtifactWriter(
            self.log_directory,
            mission_id,
            self._config_snapshot(),
        )
        self.result_published = False

    def mission_callback(self, message):
        self.latest_mission = message
        now = self._now()
        if (
            message.intercept_requested
            and (
                self.evaluator.started_at is None
                or int(message.mission_id) != self.evaluator.mission_id
            )
        ):
            self._start_mission(int(message.mission_id), now)

    def _sample_row(self, now):
        metrics = self.evaluator.instantaneous_metrics(
            self.latest_uav,
            self.latest_truth,
        )
        controller = self.latest_controller
        return {
            'time': now - self.evaluator.started_at,
            'mission_id': self.evaluator.mission_id,
            'phase': (
                self.latest_mission.state_name if self.latest_mission else ''
            ),
            'uav_x': self.latest_uav.position[0],
            'uav_y': self.latest_uav.position[1],
            'uav_z': self.latest_uav.position[2],
            'uav_vx': self.latest_uav.velocity[0],
            'uav_vy': self.latest_uav.velocity[1],
            'uav_vz': self.latest_uav.velocity[2],
            'target_x': self.latest_truth.position[0],
            'target_y': self.latest_truth.position[1],
            'target_z': self.latest_truth.position[2],
            'target_vx': self.latest_truth.velocity[0],
            'target_vy': self.latest_truth.velocity[1],
            'target_vz': self.latest_truth.velocity[2],
            'distance': metrics[0],
            'horizontal_distance': metrics[1],
            'vertical_error': metrics[2],
            'relative_speed': metrics[3],
            'closing_speed': metrics[4],
            'controller_status': controller.status if controller else '',
            'tracker_rejection_reason': (
                self.latest_tracker_rejection_reason
                if controller and controller.status == 'PLAN_REJECTED'
                else ''
            ),
            'plan_id': controller.plan_id if controller else 0,
            'plan_prediction_sequence_id': (
                controller.prediction_sequence_id if controller else 0
            ),
            'latest_prediction_sequence_id': (
                self.latest_prediction.sequence_id
                if self.latest_prediction else 0
            ),
            'planner_source_age_at_publish': (
                self.latest_planner_diagnostic.input_age_at_publish
                if self.latest_planner_diagnostic else 0.0
            ),
            'planner_completion_to_publish_delay': (
                self.latest_planner_diagnostic.completion_to_publish_delay
                if self.latest_planner_diagnostic else 0.0
            ),
            'plan_source_age': controller.source_age if controller else 0.0,
            'sea_safety_state': (
                controller.safety_state if controller else ''
            ),
            'sea_safety_margin': (
                controller.safety_margin if controller else 0.0
            ),
            'prediction_0p5_error': self.latest_prediction_error.get(
                ('guidance', 0.5),
                '',
            ),
            'prediction_1p0_error': self.latest_prediction_error.get(
                ('guidance', 1.0),
                '',
            ),
            'prediction_2p0_error': self.latest_prediction_error.get(
                ('guidance', 2.0),
                '',
            ),
        }

    def _prediction_summary(self):
        summary = {}
        for (model, horizon), values in self.prediction_errors.items():
            label = str(horizon).replace('.', 'p')
            key = f'{model}_prediction_{label}'
            ordered = sorted(values)
            p95_index = max(math.ceil(0.95 * len(ordered)) - 1, 0)
            summary[key] = {
                'count': len(values),
                'rmse': (
                    math.sqrt(sum(value * value for value in values) / len(values))
                    if values else 0.0
                ),
                'p95': ordered[p95_index] if ordered else 0.0,
            }
        return summary

    def _summary(self, result):
        summary = {
            'outcome': result.outcome,
            'failure_reason': '' if result.success else result.reason,
            'elapsed_time': result.elapsed_time,
            'capture_radius': self.capture_radius,
            'minimum_distance': result.minimum_distance,
            'horizontal_distance': result.horizontal_distance,
            'vertical_error': result.vertical_error,
            'relative_speed': result.relative_speed,
            'closing_speed': result.closing_speed,
            'max_horizontal_speed': result.maximum_horizontal_speed,
            'max_vertical_speed': result.maximum_vertical_speed,
            'max_horizontal_acceleration': (
                result.maximum_horizontal_acceleration
            ),
            'max_vertical_acceleration': result.maximum_vertical_acceleration,
            'truth_topic': self.truth_topic,
            'truth_role': 'evaluation_only',
            'prediction_errors': self._prediction_summary(),
            'controller_callback_p95': 0.0,
        }
        if self.controller_compute_times:
            ordered = sorted(self.controller_compute_times)
            summary['controller_callback_p95'] = ordered[
                max(math.ceil(0.95 * len(ordered)) - 1, 0)
            ]
            summary['controller_callback_p50'] = statistics.median(ordered)
            summary['controller_callback_max'] = ordered[-1]
        else:
            summary['controller_callback_p50'] = 0.0
            summary['controller_callback_max'] = 0.0
        summary.update(self.event_metrics.summary(result.elapsed_time))
        return summary

    def timer_callback(self):
        if (
            self.evaluator.started_at is None
            or self.result_published
            or self.latest_uav is None
            or self.latest_truth is None
        ):
            return
        now = self._now()
        result = self.evaluator.update(
            now,
            self.latest_uav,
            self.latest_truth,
        )
        if self.writer is not None:
            self.writer.append_sample(self._sample_row(now))
        if result is None:
            return
        self.result_pub.publish(
            result_to_message(result, now, self.capture_radius)
        )
        hit = Bool()
        hit.data = bool(result.success)
        self.hit_pub.publish(hit)
        if self.writer is not None:
            paths = self.writer.finalize(self._summary(result))
            self.get_logger().info(
                f'{result.outcome}: {result.reason} | '
                f'minimum distance={result.minimum_distance:.3f} m | '
                f'artifacts={paths.csv_path}'
            )
        self.result_published = True


def main(args=None):
    rclpy.init(args=args)
    node = InterceptEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.writer is not None and not node.result_published:
            node.writer.finalize({
                'outcome': 'ABORTED',
                'failure_reason': 'NODE_SHUTDOWN',
                **node.event_metrics.summary(0.0),
            })
        node.destroy_node()
        rclpy.shutdown()
