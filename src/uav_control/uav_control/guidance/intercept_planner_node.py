"""Independent latest-input-only Fast MINCO planning node."""

import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from uav_usv_interfaces.msg import InterceptTrajectory, MissionState
from uav_usv_interfaces.msg import PlannerDiagnostic, PolynomialSegment
from uav_usv_interfaces.msg import TargetPrediction

from uav_control.tracking.target_predictor_node import seconds_to_duration
from uav_control.tracking.target_predictor_node import seconds_to_time

from .fast_minco_planner import FastMincoPlanner, FastPlanningFailure
from .fast_minco_planner import FastPlanningOutcome
from .finite_horizon_intercept_planner import PlannerDiagnostics
from .planner_pipeline import LatestRequestSlot, PlannerRequest
from .planner_pipeline import PredictionSample, PredictionSeries
from .planner_pipeline import UavKinematicState, validate_input
from .planner_pipeline import validate_plan_arrival, validate_target_shift
from .planner_pipeline import validate_total_deadline


@dataclass(frozen=True)
class PlannerJobResult:
    """One completed worker event with ROS and monotonic timing."""

    request: PlannerRequest
    outcome: FastPlanningOutcome
    planning_started_stamp: float
    generated_stamp: float
    compute_time: float


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def prediction_from_message(message):
    """Convert a valid prediction message without changing source time."""
    samples = tuple(PredictionSample(
        relative_time=_stamp_seconds(sample.relative_time),
        position=(
            float(sample.position.x),
            float(sample.position.y),
            float(sample.position.z),
        ),
        velocity=(
            float(sample.velocity.x),
            float(sample.velocity.y),
            float(sample.velocity.z),
        ),
        acceleration=(
            float(sample.acceleration.x),
            float(sample.acceleration.y),
            float(sample.acceleration.z),
        ),
    ) for sample in message.samples)
    return PredictionSeries(
        mission_id=int(message.mission_id),
        sequence_id=int(message.sequence_id),
        source_stamp=_stamp_seconds(message.source_stamp),
        valid_until=_stamp_seconds(message.valid_until),
        samples=samples,
        source=str(message.source),
    )


def uav_state_from_message(message, received_stamp):
    """Convert PX4 state using ROS receipt time as the comparable timestamp."""
    values = []
    for name in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az'):
        value = float(getattr(message, name, math.nan))
        values.append(value if math.isfinite(value) else 0.0)
    return UavKinematicState(
        stamp=float(received_stamp),
        position=tuple(values[:3]),
        velocity=tuple(values[3:6]),
        acceleration=tuple(values[6:9]),
    )


def plan_to_message(
    plan,
    mission_id,
    plan_id,
    prediction_sequence_id,
    source_stamp,
    planning_started_stamp,
    generated_stamp,
    target_state_source,
    frame_id='local_ned',
):
    """Serialize a complete MINCO polynomial without resetting plan age."""
    message = InterceptTrajectory()
    message.mission_id = int(mission_id)
    message.plan_id = int(plan_id)
    message.prediction_sequence_id = int(prediction_sequence_id)
    message.source_stamp = seconds_to_time(source_stamp)
    message.planning_started_stamp = seconds_to_time(
        planning_started_stamp
    )
    message.generated_stamp = seconds_to_time(generated_stamp)
    message.valid_until = seconds_to_time(source_stamp + plan.duration)
    message.target_state_source = str(target_state_source)
    message.frame_id = str(frame_id)
    message.planner_type = str(plan.planner_type)
    message.t_go = float(plan.duration)
    message.trajectory_duration = float(plan.duration)

    segments = []
    coefficients = plan.minco_trajectory.coefficients
    for duration, piece_coefficients in zip(
        plan.piece_durations,
        coefficients,
    ):
        segment = PolynomialSegment()
        segment.duration = seconds_to_duration(duration)
        segment.coefficients = [
            float(piece_coefficients[power][axis])
            for axis in range(3)
            for power in range(6)
        ]
        segments.append(segment)
    message.segments = segments
    message.piece_count = len(segments)
    terminal = plan.sample(plan.duration)
    message.terminal_position.x = terminal.position[0]
    message.terminal_position.y = terminal.position[1]
    message.terminal_position.z = terminal.position[2]
    message.terminal_velocity.x = terminal.velocity[0]
    message.terminal_velocity.y = terminal.velocity[1]
    message.terminal_velocity.z = terminal.velocity[2]
    message.planned_closing_speed = float(plan.closing_speed)
    message.planned_max_horizontal_speed = float(
        plan.maximum_horizontal_speed
    )
    message.planned_max_vertical_speed = float(plan.maximum_vertical_speed)
    message.planned_max_horizontal_acceleration = float(
        plan.maximum_horizontal_acceleration
    )
    message.planned_max_vertical_acceleration = float(
        plan.maximum_vertical_acceleration
    )
    message.valid = True
    message.invalid_reason = ''
    return message


FAILURE_CONSTANTS = {
    FastPlanningFailure.NONE: PlannerDiagnostic.NONE,
    FastPlanningFailure.STATE_STALE: PlannerDiagnostic.STATE_STALE,
    FastPlanningFailure.PREDICTION_STALE: (
        PlannerDiagnostic.PREDICTION_STALE
    ),
    FastPlanningFailure.HORIZON_INSUFFICIENT: (
        PlannerDiagnostic.HORIZON_INSUFFICIENT
    ),
    FastPlanningFailure.CAPTURE_GEOMETRY: (
        PlannerDiagnostic.CAPTURE_GEOMETRY
    ),
    FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL: (
        PlannerDiagnostic.DYNAMIC_LIMIT_HORIZONTAL
    ),
    FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL: (
        PlannerDiagnostic.DYNAMIC_LIMIT_VERTICAL
    ),
    FastPlanningFailure.SEA_CLEARANCE: PlannerDiagnostic.SEA_CLEARANCE,
    FastPlanningFailure.MINCO_CONSTRUCTION_FAIL: (
        PlannerDiagnostic.MINCO_CONSTRUCTION_FAIL
    ),
    FastPlanningFailure.OPTIMIZATION_FAIL: (
        PlannerDiagnostic.OPTIMIZATION_FAIL
    ),
    FastPlanningFailure.DEADLINE_EXCEEDED: (
        PlannerDiagnostic.DEADLINE_EXCEEDED
    ),
    FastPlanningFailure.PLAN_STALE_ON_ARRIVAL: (
        PlannerDiagnostic.PLAN_STALE_ON_ARRIVAL
    ),
}


class InterceptPlannerNode(Node):
    """Run Fast MINCO in its own process and retain only the latest request."""

    def __init__(self):
        super().__init__('intercept_planner_node')
        self.declare_parameter('planning_rate_hz', 5.0)
        self.declare_parameter('minimum_duration', 1.0)
        self.declare_parameter('maximum_duration', 3.0)
        self.declare_parameter('duration_margin', 0.35)
        self.declare_parameter('sample_step', 0.05)
        self.declare_parameter('maximum_horizontal_speed', 6.5)
        self.declare_parameter('maximum_vertical_speed', 4.0)
        self.declare_parameter('maximum_horizontal_acceleration', 3.0)
        self.declare_parameter('maximum_vertical_acceleration', 3.0)
        self.declare_parameter('preferred_closing_speed', 1.5)
        self.declare_parameter('conservative_closing_speed', 0.3)
        self.declare_parameter('capture_radius', 0.25)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('contact_clearance', 0.05)
        self.declare_parameter('preferred_clearance', 0.1)
        self.declare_parameter('piece_count', 3)
        self.declare_parameter('target_curve_weight', 0.7)
        self.declare_parameter('solver_deadline_seconds', 0.075)
        self.declare_parameter('deadline_seconds', 0.08)
        self.declare_parameter('maximum_input_age', 0.125)
        self.declare_parameter('endpoint_tolerance', 0.5)
        self.declare_parameter('response_delay', 0.15)
        self.declare_parameter(
            'effective_vertical_braking_acceleration',
            2.5,
        )
        self.declare_parameter('quadrature_intervals_per_piece', 6)
        self.declare_parameter('frame_id', 'local_ned')

        self.maximum_input_age = float(
            self.get_parameter('maximum_input_age').value
        )
        self.endpoint_tolerance = float(
            self.get_parameter('endpoint_tolerance').value
        )
        self.hard_deadline_seconds = float(
            self.get_parameter('deadline_seconds').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.planner = FastMincoPlanner(
            minimum_duration=self.get_parameter('minimum_duration').value,
            maximum_duration=self.get_parameter('maximum_duration').value,
            duration_margin=self.get_parameter('duration_margin').value,
            sample_step=self.get_parameter('sample_step').value,
            maximum_horizontal_speed=self.get_parameter(
                'maximum_horizontal_speed'
            ).value,
            maximum_vertical_speed=self.get_parameter(
                'maximum_vertical_speed'
            ).value,
            maximum_horizontal_acceleration=self.get_parameter(
                'maximum_horizontal_acceleration'
            ).value,
            maximum_vertical_acceleration=self.get_parameter(
                'maximum_vertical_acceleration'
            ).value,
            preferred_closing_speed=self.get_parameter(
                'preferred_closing_speed'
            ).value,
            conservative_closing_speed=self.get_parameter(
                'conservative_closing_speed'
            ).value,
            capture_radius=self.get_parameter('capture_radius').value,
            sea_surface_z=self.get_parameter('sea_surface_z').value,
            contact_clearance=self.get_parameter(
                'contact_clearance'
            ).value,
            preferred_clearance=self.get_parameter(
                'preferred_clearance'
            ).value,
            piece_count=self.get_parameter('piece_count').value,
            target_curve_weight=self.get_parameter(
                'target_curve_weight'
            ).value,
            deadline_seconds=self.get_parameter(
                'solver_deadline_seconds'
            ).value,
            response_delay=self.get_parameter('response_delay').value,
            effective_vertical_braking_acceleration=self.get_parameter(
                'effective_vertical_braking_acceleration'
            ).value,
            quadrature_intervals_per_piece=self.get_parameter(
                'quadrature_intervals_per_piece'
            ).value,
        )
        planning_rate_hz = float(
            self.get_parameter('planning_rate_hz').value
        )
        if not math.isfinite(planning_rate_hz) or planning_rate_hz <= 0.0:
            raise ValueError('planning_rate_hz must be finite and positive')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.prediction_sub = self.create_subscription(
            TargetPrediction,
            '/planning/target_prediction',
            self.prediction_callback,
            px4_qos,
        )
        self.uav_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.uav_callback,
            px4_qos,
        )
        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_callback,
            output_qos,
        )
        self.trajectory_pub = self.create_publisher(
            InterceptTrajectory,
            '/planning/intercept_trajectory',
            output_qos,
        )
        self.diagnostic_pub = self.create_publisher(
            PlannerDiagnostic,
            '/planning/diagnostic',
            output_qos,
        )
        self.timer = self.create_timer(
            1.0 / planning_rate_hz,
            self.timer_callback,
        )

        self.latest_prediction = None
        self.latest_uav = None
        self.mission_id = 0
        self.intercept_requested = False
        self.plan_id = 0
        self.completed_plan_count = 0
        self.completion_times = deque(maxlen=100)
        self.request_slot = LatestRequestSlot()
        self.executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='fast_minco',
        )
        self.future = None
        self.last_submitted_key = None
        self.get_logger().info(
            'Fast MINCO planner ready | rate='
            f'{planning_rate_hz:.1f} Hz | candidates<=6 | '
            f'solver deadline={self.planner.deadline_seconds * 1000.0:.0f} '
            f'ms | hard deadline={self.hard_deadline_seconds * 1000.0:.0f} ms'
        )

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def prediction_callback(self, message):
        if message.valid and message.samples:
            self.latest_prediction = prediction_from_message(message)

    def uav_callback(self, message):
        self.latest_uav = uav_state_from_message(
            message,
            self._ros_seconds(),
        )

    def mission_callback(self, message):
        if int(message.mission_id) != self.mission_id:
            self.request_slot.take()
            self.last_submitted_key = None
        self.mission_id = int(message.mission_id)
        self.intercept_requested = bool(message.intercept_requested)

    def _current_request(self):
        if self.latest_prediction is None or self.latest_uav is None:
            return None
        return PlannerRequest(
            mission_id=self.mission_id,
            prediction=self.latest_prediction,
            uav=self.latest_uav,
        )

    def _run_request(self, request):
        planning_started_stamp = self._ros_seconds()
        monotonic_start = time.perf_counter()
        failure = validate_input(
            request,
            planning_started_stamp,
            self.maximum_input_age,
        )
        if failure == FastPlanningFailure.NONE:
            outcome = self.planner.plan(
                initial_position=request.uav.position,
                initial_velocity=request.uav.velocity,
                initial_acceleration=request.uav.acceleration,
                target_state_at_time=request.prediction.state_at,
            )
        else:
            outcome = FastPlanningOutcome(
                plan=None,
                failure=failure,
                diagnostics=PlannerDiagnostics(),
            )
        elapsed = time.perf_counter() - monotonic_start
        deadline_failure = validate_total_deadline(
            elapsed,
            self.hard_deadline_seconds,
        )
        if (
            outcome.plan is not None
            and deadline_failure != FastPlanningFailure.NONE
        ):
            outcome = FastPlanningOutcome(
                plan=None,
                failure=deadline_failure,
                diagnostics=outcome.diagnostics,
            )
        generated_stamp = self._ros_seconds()
        if outcome.plan is not None:
            arrival_failure = validate_plan_arrival(
                request,
                generated_stamp,
                self.maximum_input_age,
            )
            if arrival_failure != FastPlanningFailure.NONE:
                outcome = FastPlanningOutcome(
                    plan=None,
                    failure=arrival_failure,
                    diagnostics=outcome.diagnostics,
                )
        return PlannerJobResult(
            request=request,
            outcome=outcome,
            planning_started_stamp=planning_started_stamp,
            generated_stamp=generated_stamp,
            compute_time=time.perf_counter() - monotonic_start,
        )

    def _completion_frequency(self):
        if len(self.completion_times) < 2:
            return 0.0
        elapsed = self.completion_times[-1] - self.completion_times[0]
        return (
            (len(self.completion_times) - 1) / elapsed
            if elapsed > 1e-9 else 0.0
        )

    def _publish_job(self, job):
        outcome = job.outcome
        if outcome.plan is not None:
            shift_failure = validate_target_shift(
                request=job.request,
                latest_prediction=self.latest_prediction,
                intercept_time=outcome.plan.duration,
                tolerance=self.endpoint_tolerance,
            )
            if shift_failure != FastPlanningFailure.NONE:
                outcome = FastPlanningOutcome(
                    plan=None,
                    failure=shift_failure,
                    diagnostics=outcome.diagnostics,
                )

        self.completed_plan_count += 1
        self.completion_times.append(time.monotonic())
        self.plan_id += 1
        diagnostics = outcome.diagnostics
        message = PlannerDiagnostic()
        message.mission_id = job.request.mission_id
        message.plan_id = self.plan_id
        message.prediction_sequence_id = (
            job.request.prediction.sequence_id
        )
        message.source_stamp = seconds_to_time(job.request.source_stamp)
        message.planning_started_stamp = seconds_to_time(
            job.planning_started_stamp
        )
        message.generated_stamp = seconds_to_time(job.generated_stamp)
        message.result = (
            PlannerDiagnostic.RESULT_SUCCESS
            if outcome.plan is not None
            else PlannerDiagnostic.RESULT_FAILURE
        )
        message.failure_reason = FAILURE_CONSTANTS[outcome.failure]
        message.failure_detail = outcome.failure.value
        message.compute_time = job.compute_time
        message.generation_time = diagnostics.generation_compute_time
        message.optimization_time = diagnostics.optimization_compute_time
        message.input_age_at_start = (
            job.planning_started_stamp - job.request.source_stamp
        )
        message.input_age_at_finish = (
            job.generated_stamp - job.request.source_stamp
        )
        message.candidate_count = diagnostics.candidates_checked
        message.replaced_request_count = (
            self.request_slot.replaced_request_count
        )
        message.completed_plan_count = self.completed_plan_count
        message.actual_completion_frequency = self._completion_frequency()
        self.diagnostic_pub.publish(message)

        if outcome.plan is not None:
            self.trajectory_pub.publish(plan_to_message(
                outcome.plan,
                mission_id=job.request.mission_id,
                plan_id=self.plan_id,
                prediction_sequence_id=(
                    job.request.prediction.sequence_id
                ),
                source_stamp=job.request.source_stamp,
                planning_started_stamp=job.planning_started_stamp,
                generated_stamp=job.generated_stamp,
                target_state_source=job.request.prediction.source,
                frame_id=self.frame_id,
            ))

    def timer_callback(self):
        if self.future is not None and self.future.done():
            self._publish_job(self.future.result())
            self.future = None

        if not self.intercept_requested:
            return
        current = self._current_request()
        if current is not None:
            key = (
                current.mission_id,
                current.prediction.sequence_id,
                current.uav.stamp,
            )
            if key != self.last_submitted_key:
                self.request_slot.submit(current)
                self.last_submitted_key = key
        if self.future is None:
            request = self.request_slot.take()
            if request is not None:
                self.future = self.executor.submit(
                    self._run_request,
                    request,
                )

    def destroy_node(self):
        self.executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InterceptPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
