"""Pure truth-only interception evaluation and event-based statistics."""

import csv
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class KinematicState:
    """Three-dimensional position and velocity in local NED coordinates."""

    position: tuple
    velocity: tuple


class TimestampedStateHistory:
    """Bounded kinematic history with linear interpolation in ROS time."""

    def __init__(self, maximum_age=0.25):
        self.maximum_age = max(float(maximum_age), 1e-3)
        self._samples = deque()

    @property
    def latest_stamp(self):
        return self._samples[-1][0] if self._samples else None

    def clear(self):
        self._samples.clear()

    def add(self, stamp, state):
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError('state history stamp must be finite')
        if self._samples and stamp < self._samples[-1][0] - 1e-9:
            return False
        if self._samples and abs(stamp - self._samples[-1][0]) <= 1e-9:
            self._samples[-1] = stamp, state
        else:
            self._samples.append((stamp, state))
        cutoff = stamp - self.maximum_age
        while len(self._samples) > 2 and self._samples[1][0] < cutoff:
            self._samples.popleft()
        return True

    @staticmethod
    def _blend(first, second, fraction):
        return tuple(
            start + fraction * (finish - start)
            for start, finish in zip(first, second)
        )

    def state_at(self, stamp):
        stamp = float(stamp)
        if not self._samples:
            return None
        if stamp < self._samples[0][0] - 1e-9:
            return None
        if stamp > self._samples[-1][0] + 1e-9:
            return None
        for index, (sample_stamp, state) in enumerate(self._samples):
            if abs(stamp - sample_stamp) <= 1e-9:
                return state
            if sample_stamp > stamp and index > 0:
                previous_stamp, previous = self._samples[index - 1]
                span = max(sample_stamp - previous_stamp, 1e-9)
                fraction = (stamp - previous_stamp) / span
                return KinematicState(
                    position=self._blend(
                        previous.position,
                        state.position,
                        fraction,
                    ),
                    velocity=self._blend(
                        previous.velocity,
                        state.velocity,
                        fraction,
                    ),
                )
        return self._samples[-1][1]


def synchronize_histories(uav_history, target_history):
    """Return both histories interpolated at their newest common time."""
    if (
        uav_history.latest_stamp is None
        or target_history.latest_stamp is None
    ):
        return None
    stamp = min(uav_history.latest_stamp, target_history.latest_stamp)
    uav = uav_history.state_at(stamp)
    target = target_history.state_at(stamp)
    if uav is None or target is None:
        return None
    return stamp, uav, target


@dataclass(frozen=True)
class EvaluationResult:
    """Terminal result derived only from evaluator truth inputs."""

    mission_id: int
    success: bool
    outcome: str
    reason: str
    elapsed_time: float
    minimum_distance: float
    horizontal_distance: float
    vertical_error: float
    relative_speed: float
    closing_speed: float
    maximum_horizontal_speed: float
    maximum_vertical_speed: float
    maximum_horizontal_acceleration: float
    maximum_vertical_acceleration: float


def _norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = min(
        max(int(math.ceil(float(fraction) * len(ordered))) - 1, 0),
        len(ordered) - 1,
    )
    return ordered[index]


class PlannerEventAccumulator:
    """Count each planner event once rather than once per control sample."""

    def __init__(self):
        self._planner_events = {}
        self._completion_stamps = []
        self._compute_times = []
        self._reachability_times = []
        self._generation_times = []
        self._validation_times = []
        self._publish_ages = []
        self._publication_delays = []
        self._executed_plan_ids = set()
        self._controller_event_count = 0
        self._hold_event_count = 0
        self._tracker_rejection_histogram = {}

    def observe_planner(
        self,
        mission_id,
        plan_id,
        success,
        failure_reason,
        compute_time,
        generation_time,
        completion_stamp,
        reachability_time=0.0,
        validation_time=0.0,
        input_age_at_publish=0.0,
        completion_to_publish_delay=0.0,
    ):
        """Store a completed event if its mission/plan identity is new."""
        key = int(mission_id), int(plan_id)
        if key in self._planner_events:
            return False
        event = {
            'success': bool(success),
            'failure_reason': str(failure_reason),
            'compute_time': max(float(compute_time), 0.0),
            'reachability_time': max(float(reachability_time), 0.0),
            'generation_time': max(float(generation_time), 0.0),
            'validation_time': max(float(validation_time), 0.0),
            'completion_stamp': float(completion_stamp),
            'input_age_at_publish': max(float(input_age_at_publish), 0.0),
            'completion_to_publish_delay': max(
                float(completion_to_publish_delay),
                0.0,
            ),
        }
        self._planner_events[key] = event
        self._completion_stamps.append(event['completion_stamp'])
        self._compute_times.append(event['compute_time'])
        self._reachability_times.append(event['reachability_time'])
        self._generation_times.append(event['generation_time'])
        self._validation_times.append(event['validation_time'])
        self._publish_ages.append(event['input_age_at_publish'])
        self._publication_delays.append(
            event['completion_to_publish_delay']
        )
        return True

    def observe_controller(
        self,
        mission_id,
        plan_id,
        status,
        rejection_reason='',
    ):
        """Record execution and hold events without duplicating plan IDs."""
        self._controller_event_count += 1
        status = str(status)
        if status in ('PLAN_ACCEPTED', 'TRACKING') and int(plan_id) > 0:
            self._executed_plan_ids.add((int(mission_id), int(plan_id)))
        if status in ('NO_VALID_PLAN', 'SAFE_WAIT', 'HOLD'):
            self._hold_event_count += 1
        if status == 'PLAN_REJECTED' and rejection_reason:
            reason = str(rejection_reason)
            self._tracker_rejection_histogram[reason] = (
                self._tracker_rejection_histogram.get(reason, 0) + 1
            )

    def summary(self, elapsed_time):
        """Return numeric event rates even when no event has occurred."""
        events = list(self._planner_events.values())
        succeeded = sum(event['success'] for event in events)
        failed = len(events) - succeeded
        deadlines = sum(
            event['failure_reason'] == 'DEADLINE_EXCEEDED'
            for event in events
        )
        failure_histogram = {}
        for event in events:
            if event['success']:
                continue
            reason = event['failure_reason']
            failure_histogram[reason] = failure_histogram.get(reason, 0) + 1
        executed_successes = sum(
            key in self._executed_plan_ids and event['success']
            for key, event in self._planner_events.items()
        )
        elapsed_time = max(float(elapsed_time), 0.0)
        stamps = sorted(self._completion_stamps)
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            completion_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        else:
            completion_hz = 0.0
        return {
            'planner_started': len(events),
            'planner_completed': len(events),
            'planner_succeeded': succeeded,
            'planner_failed': failed,
            'planner_deadline': deadlines,
            'planner_failure_histogram': dict(sorted(
                failure_histogram.items()
            )),
            'attempt_rate': (
                len(events) / elapsed_time if elapsed_time > 0.0 else 0.0
            ),
            'execution_rate': (
                executed_successes / succeeded if succeeded else 0.0
            ),
            'hold_rate': (
                self._hold_event_count / self._controller_event_count
                if self._controller_event_count else 0.0
            ),
            'tracker_rejection_histogram': dict(sorted(
                self._tracker_rejection_histogram.items()
            )),
            'actual_completion_hz': completion_hz,
            'planner_compute_p50': _percentile(self._compute_times, 0.50),
            'planner_compute_p95': _percentile(self._compute_times, 0.95),
            'planner_compute_max': max(self._compute_times, default=0.0),
            'reachability_compute_p50': _percentile(
                self._reachability_times,
                0.50,
            ),
            'reachability_compute_p95': _percentile(
                self._reachability_times,
                0.95,
            ),
            'reachability_compute_max': max(
                self._reachability_times,
                default=0.0,
            ),
            'generation_compute_p50': _percentile(
                self._generation_times,
                0.50,
            ),
            'generation_compute_p95': _percentile(
                self._generation_times,
                0.95,
            ),
            'generation_compute_max': max(
                self._generation_times,
                default=0.0,
            ),
            'validation_compute_p50': _percentile(
                self._validation_times,
                0.50,
            ),
            'validation_compute_p95': _percentile(
                self._validation_times,
                0.95,
            ),
            'validation_compute_max': max(
                self._validation_times,
                default=0.0,
            ),
            'planner_source_age_at_publish_p50': _percentile(
                self._publish_ages,
                0.50,
            ),
            'planner_source_age_at_publish_p95': _percentile(
                self._publish_ages,
                0.95,
            ),
            'planner_source_age_at_publish_max': max(
                self._publish_ages,
                default=0.0,
            ),
            'planner_publish_delay_p50': _percentile(
                self._publication_delays,
                0.50,
            ),
            'planner_publish_delay_p95': _percentile(
                self._publication_delays,
                0.95,
            ),
            'planner_publish_delay_max': max(
                self._publication_delays,
                default=0.0,
            ),
        }


class RuntimePerformanceAccumulator:
    """Aggregate runtime rates and callback costs by target-distance band."""

    BUCKETS = (
        ('less_than_2m', lambda distance: distance < 2.0),
        ('between_2m_and_5m', lambda distance: distance <= 5.0),
        ('between_5m_and_10m', lambda distance: distance <= 10.0),
        ('greater_than_10m', lambda _distance: True),
    )

    def __init__(self):
        self._values = {
            name: {}
            for name, _predicate in self.BUCKETS
        }

    def observe(self, distance, metrics):
        distance = float(distance)
        if not math.isfinite(distance):
            return
        bucket = next(
            name
            for name, predicate in self.BUCKETS
            if predicate(distance)
        )
        for name, value in dict(metrics).items():
            value = float(value)
            if math.isfinite(value) and value >= 0.0:
                self._values[bucket].setdefault(str(name), []).append(value)

    def summary(self):
        document = {}
        for bucket, metrics in self._values.items():
            document[bucket] = {}
            for name, values in metrics.items():
                document[bucket][name] = {
                    'count': len(values),
                    'p50': _percentile(values, 0.50),
                    'p95': _percentile(values, 0.95),
                    'max': max(values, default=0.0),
                }
        return document


class InterceptEvaluatorCore:
    """Detect capture, sea contact, timeout, and motion extrema from truth."""

    def __init__(
        self,
        capture_radius=0.25,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
        maximum_duration=30.0,
    ):
        self.capture_radius = float(capture_radius)
        self.sea_surface_z = float(sea_surface_z)
        self.enable_sea_contact_failure = bool(enable_sea_contact_failure)
        self.maximum_duration = float(maximum_duration)
        self.reset()

    def reset(self):
        """Clear all mission-scoped evaluation state."""
        self.mission_id = 0
        self.started_at = None
        self.previous_time = None
        self.previous_uav = None
        self.previous_target = None
        self.minimum_distance = math.inf
        self.closest_horizontal_distance = math.inf
        self.closest_vertical_error = math.inf
        self.maximum_horizontal_speed = 0.0
        self.maximum_vertical_speed = 0.0
        self.maximum_horizontal_acceleration = 0.0
        self.maximum_vertical_acceleration = 0.0
        self.result = None

    def begin(self, mission_id, now):
        """Start a clean truth-evaluation interval for one mission."""
        self.reset()
        self.mission_id = int(mission_id)
        self.started_at = float(now)

    @staticmethod
    def _relative(uav, target):
        return tuple(
            target_value - uav_value
            for target_value, uav_value in zip(
                target.position,
                uav.position,
            )
        )

    @staticmethod
    def _closest_relative(relative_start, relative_end):
        delta = tuple(
            end - start
            for start, end in zip(relative_start, relative_end)
        )
        denominator = sum(value * value for value in delta)
        if denominator <= 1e-12:
            fraction = 0.0
        else:
            fraction = -sum(
                start * change
                for start, change in zip(relative_start, delta)
            ) / denominator
            fraction = min(max(fraction, 0.0), 1.0)
        return tuple(
            start + fraction * change
            for start, change in zip(relative_start, delta)
        )

    def _update_closest(self, relative):
        horizontal = math.hypot(relative[0], relative[1])
        distance = math.hypot(horizontal, relative[2])
        if distance < self.minimum_distance:
            self.minimum_distance = distance
            self.closest_horizontal_distance = horizontal
            self.closest_vertical_error = relative[2]

    def _capture_fraction(self, start, end):
        delta = tuple(finish - begin for begin, finish in zip(start, end))
        c = sum(value * value for value in start) - self.capture_radius**2
        if c <= 0.0:
            return 0.0
        a = sum(value * value for value in delta)
        if a <= 1e-12:
            return None
        b = 2.0 * sum(value * change for value, change in zip(start, delta))
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return None
        root = math.sqrt(discriminant)
        candidates = (
            (-b - root) / (2.0 * a),
            (-b + root) / (2.0 * a),
        )
        valid = [value for value in candidates if 0.0 <= value <= 1.0]
        return min(valid) if valid else None

    def _sea_fraction(self, start_z, end_z):
        if not self.enable_sea_contact_failure:
            return None
        if start_z >= self.sea_surface_z:
            return 0.0
        delta = end_z - start_z
        if delta <= 0.0:
            return None
        fraction = (self.sea_surface_z - start_z) / delta
        return fraction if 0.0 <= fraction <= 1.0 else None

    @staticmethod
    def instantaneous_metrics(uav, target):
        """Return range decomposition and relative motion from truth."""
        relative_position = InterceptEvaluatorCore._relative(uav, target)
        relative_velocity = tuple(
            target_value - uav_value
            for target_value, uav_value in zip(
                target.velocity,
                uav.velocity,
            )
        )
        distance = _norm(relative_position)
        horizontal = math.hypot(
            relative_position[0],
            relative_position[1],
        )
        relative_speed = _norm(relative_velocity)
        closing_speed = (
            -sum(
                position * velocity
                for position, velocity in zip(
                    relative_position,
                    relative_velocity,
                )
            ) / distance
            if distance > 1e-9 else 0.0
        )
        return (
            distance,
            horizontal,
            relative_position[2],
            relative_speed,
            closing_speed,
        )

    def _finish(self, success, reason, now, uav, target):
        metrics = self.instantaneous_metrics(uav, target)
        self._update_closest(self._relative(uav, target))
        self.result = EvaluationResult(
            mission_id=self.mission_id,
            success=bool(success),
            outcome='SUCCESS' if success else 'FAILURE',
            reason=str(reason),
            elapsed_time=max(float(now) - self.started_at, 0.0),
            minimum_distance=self.minimum_distance,
            horizontal_distance=metrics[1],
            vertical_error=metrics[2],
            relative_speed=metrics[3],
            closing_speed=metrics[4],
            maximum_horizontal_speed=self.maximum_horizontal_speed,
            maximum_vertical_speed=self.maximum_vertical_speed,
            maximum_horizontal_acceleration=(
                self.maximum_horizontal_acceleration
            ),
            maximum_vertical_acceleration=self.maximum_vertical_acceleration,
        )
        return self.result

    def update(self, now, uav, target):
        """Consume one synchronized truth sample and return a terminal event."""
        if self.started_at is None or self.result is not None:
            return self.result
        now = float(now)
        relative = self._relative(uav, target)
        self._update_closest(relative)
        self.maximum_horizontal_speed = max(
            self.maximum_horizontal_speed,
            math.hypot(uav.velocity[0], uav.velocity[1]),
        )
        self.maximum_vertical_speed = max(
            self.maximum_vertical_speed,
            abs(uav.velocity[2]),
        )

        event = None
        if self.previous_uav is not None and self.previous_target is not None:
            previous_relative = self._relative(
                self.previous_uav,
                self.previous_target,
            )
            self._update_closest(
                self._closest_relative(previous_relative, relative)
            )
            capture_fraction = self._capture_fraction(
                previous_relative,
                relative,
            )
            sea_fraction = self._sea_fraction(
                self.previous_uav.position[2],
                uav.position[2],
            )
            if capture_fraction is not None and (
                sea_fraction is None or capture_fraction <= sea_fraction
            ):
                event = True, 'CAPTURE_RADIUS_REACHED'
            elif sea_fraction is not None:
                event = False, 'SEA_CONTACT'
            dt = now - self.previous_time
            if dt > 1e-6:
                acceleration = tuple(
                    (current - previous) / dt
                    for current, previous in zip(
                        uav.velocity,
                        self.previous_uav.velocity,
                    )
                )
                self.maximum_horizontal_acceleration = max(
                    self.maximum_horizontal_acceleration,
                    math.hypot(acceleration[0], acceleration[1]),
                )
                self.maximum_vertical_acceleration = max(
                    self.maximum_vertical_acceleration,
                    abs(acceleration[2]),
                )
        else:
            if _norm(relative) <= self.capture_radius:
                event = True, 'CAPTURE_RADIUS_REACHED'
            elif (
                self.enable_sea_contact_failure
                and uav.position[2] >= self.sea_surface_z
            ):
                event = False, 'SEA_CONTACT'

        self.previous_time = now
        self.previous_uav = uav
        self.previous_target = target
        if event is not None:
            return self._finish(event[0], event[1], now, uav, target)
        if now - self.started_at >= self.maximum_duration:
            return self._finish(False, 'TIMEOUT', now, uav, target)
        return None


@dataclass(frozen=True)
class ArtifactPaths:
    """Paths produced for one evaluation run."""

    csv_path: Path
    summary_path: Path
    config_path: Path


class ExperimentArtifactWriter:
    """Write isolated sample, summary, and configuration artifacts."""

    CSV_FIELDS = (
        'time', 'mission_id', 'phase',
        'uav_x', 'uav_y', 'uav_z', 'uav_vx', 'uav_vy', 'uav_vz',
        'target_x', 'target_y', 'target_z',
        'target_vx', 'target_vy', 'target_vz',
        'distance', 'horizontal_distance', 'vertical_error',
        'relative_speed', 'closing_speed',
        'controller_status', 'plan_id', 'prediction_age', 'trajectory_age',
        'tracker_rejection_reason',
        'plan_prediction_sequence_id', 'latest_prediction_sequence_id',
        'planner_failure_reason', 'planner_failure_detail',
        'planner_required_time',
        'planner_horizontal_min_time',
        'planner_vertical_min_time',
        'planner_sea_safe_min_time',
        'planner_search_min_time', 'planner_search_max_time',
        'planner_available_prediction_duration',
        'planner_locked_remaining_t_go',
        'planner_reachability_time', 'planner_generation_time',
        'planner_validation_time', 'planner_candidate_diagnostics',
        'planner_source_age_at_publish',
        'planner_completion_to_publish_delay',
        'selected_t_go', 'contact_stamp', 'remaining_t_go',
        'terminal_mode', 'planned_capture_margin', 'target_yaw',
        'sea_safety_state', 'sea_safety_margin',
        'gazebo_real_time_factor',
        'front_rgb_hz', 'front_depth_hz',
        'down_rgb_hz', 'down_depth_hz',
        'rgbd_localizer_compute_time',
        'front_monitor_compute_time', 'down_monitor_compute_time',
        'tracker_hz', 'tracker_callback_time', 'planner_compute_time',
        'prediction_0p5_error', 'prediction_1p0_error',
        'prediction_2p0_error',
        'kf_prediction_0p5_error',
        'kf_prediction_1p0_error',
        'kf_prediction_2p0_error',
        'shadow_bctra_prediction_0p5_error',
        'shadow_bctra_prediction_1p0_error',
        'shadow_bctra_prediction_2p0_error',
    )

    def __init__(
        self,
        log_directory,
        mission_id,
        config,
        prefix=None,
    ):
        directory = Path(log_directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        if prefix is None:
            prefix = 'modular_intercept_' + datetime.now().strftime(
                '%Y%m%d_%H%M%S_%f'
            )
        stem = f'{prefix}_mission_{int(mission_id)}'
        self.paths = ArtifactPaths(
            csv_path=directory / f'{stem}.csv',
            summary_path=directory / f'{stem}_summary.json',
            config_path=directory / f'{stem}_config.yaml',
        )
        self.mission_id = int(mission_id)
        self._stream = self.paths.csv_path.open(
            'x',
            newline='',
            encoding='utf-8',
            buffering=1,
        )
        self._writer = csv.DictWriter(
            self._stream,
            fieldnames=self.CSV_FIELDS,
            extrasaction='ignore',
        )
        self._writer.writeheader()
        with self.paths.config_path.open('x', encoding='utf-8') as stream:
            json.dump(config, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        self._finalized = False

    def append_sample(self, sample):
        """Append one evaluation sample outside the control process."""
        if self._finalized:
            raise RuntimeError('experiment artifacts are already finalized')
        self._writer.writerow(dict(sample))

    def finalize(self, summary):
        """Write exactly one summary and close the line-buffered CSV."""
        if self._finalized:
            return self.paths
        document = dict(summary)
        document['mission_id'] = self.mission_id
        with self.paths.summary_path.open('x', encoding='utf-8') as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        self._stream.close()
        self._finalized = True
        return self.paths
