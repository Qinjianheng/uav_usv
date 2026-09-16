from concurrent.futures import ThreadPoolExecutor
import time

from uav_control.control.trajectory_tracking import PolynomialSegmentData
from uav_control.control.trajectory_tracking import PolynomialTrajectory
from uav_control.control.trajectory_tracking import TrackerKinematicState
from uav_control.control.trajectory_tracking import TrajectoryRejectReason
from uav_control.control.trajectory_tracking import TrajectoryTrackerCore
from uav_control.evaluation.intercept_evaluator import InterceptEvaluatorCore
from uav_control.evaluation.intercept_evaluator import KinematicState
from uav_control.guidance.fast_minco_planner import FastPlanningFailure
from uav_control.guidance.planner_pipeline import PlannerRequest
from uav_control.guidance.planner_pipeline import PredictionSample
from uav_control.guidance.planner_pipeline import PredictionSeries
from uav_control.guidance.planner_pipeline import UavKinematicState
from uav_control.guidance.planner_pipeline import validate_input
from uav_control.guidance.planner_pipeline import validate_plan_arrival
from uav_control.guidance.planner_pipeline import LatestRequestSlot
from uav_control.guidance.planner_pipeline import validate_target_shift
from uav_control.mission.mission_manager import MissionManagerCore
from uav_control.mission.mission_manager import MissionPhase
from uav_control.tracking.target_prediction import PredictionEngine
from uav_control.tracking.target_prediction import TargetKinematicState


def begin_intercept(manager):
    manager.set_flight_ready(True)
    manager.handle_command('X', 1.0)
    manager.mark_takeoff_complete(2.0)
    manager.handle_command('Y', 3.0)


def request(prediction_stamp=10.0, uav_stamp=10.02):
    prediction = PredictionSeries(
        mission_id=1,
        sequence_id=5,
        source_stamp=prediction_stamp,
        valid_until=prediction_stamp + 0.125,
        samples=(
            PredictionSample(
                0.0,
                (0.0, 0.0, -1.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                2.0,
                (2.0, 0.0, -0.6),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        source='simulation_truth',
    )
    return PlannerRequest(
        mission_id=1,
        prediction=prediction,
        uav=UavKinematicState(
            stamp=uav_stamp,
            position=(0.0, 0.0, -1.0),
            velocity=(1.0, 0.0, 0.2),
            acceleration=(0.0, 0.0, 0.0),
        ),
    )


def trajectory(mission_id=1, source_stamp=10.0, plan_id=7):
    return PolynomialTrajectory(
        mission_id=mission_id,
        plan_id=plan_id,
        prediction_sequence_id=5,
        source_stamp=source_stamp,
        generated_stamp=source_stamp + 0.02,
        valid_until=source_stamp + 2.0,
        segments=(PolynomialSegmentData(2.0, (
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -1.0, 0.2, 0.0, 0.0, 0.0, 0.0,
        )),),
        terminal_position=(2.0, 0.0, -0.6),
        terminal_velocity=(1.0, 0.0, 0.2),
        target_state_source='simulation_truth',
    )


def tracker_state(stamp=10.05):
    elapsed = stamp - 10.0
    return TrackerKinematicState(
        stamp=stamp,
        position=(elapsed, 0.0, -1.0 + 0.2 * elapsed),
        velocity=(1.0, 0.0, 0.2),
    )


def test_predictor_staleness_is_preserved_at_planner_boundary():
    engine = PredictionEngine(input_timeout=0.125)
    engine.update(TargetKinematicState(
        stamp=10.0,
        position=(1.0, 0.0, -0.1),
        velocity=(4.0, 0.0, 0.0),
    ))

    prediction = engine.generate(10.2, mission_id=1, sequence_id=1)
    failure = validate_input(
        request(prediction_stamp=9.8, uav_stamp=10.0),
        now=10.01,
        maximum_age=0.125,
    )

    assert not prediction.valid
    assert prediction.invalid_reason == 'STATE_STALE'
    assert failure == FastPlanningFailure.PREDICTION_STALE


def test_500_ms_planner_result_cannot_enter_minco_ready():
    manager = MissionManagerCore()
    begin_intercept(manager)

    failure = validate_plan_arrival(
        request(),
        generated_stamp=10.5,
        maximum_age=0.125,
    )
    manager.observe_planner(False, plan_id=7, now=10.5)

    assert failure == FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    assert manager.phase == MissionPhase.FAR_GUIDANCE


def test_fast_valid_plan_flows_tracker_to_mission_and_truth_evaluator():
    manager = MissionManagerCore()
    begin_intercept(manager)
    tracker = TrajectoryTrackerCore()
    accepted = tracker.accept(
        trajectory(),
        tracker_state(),
        mission_id=manager.mission_id,
        prediction_sequence_id=5,
        target_endpoint=(2.0, 0.0, -0.6),
    )
    manager.observe_tracker(
        mission_id=manager.mission_id,
        plan_id=7,
        status='PLAN_ACCEPTED',
        source_age=0.05,
        remaining_time=1.95,
        now=10.05,
    )
    manager.tick(10.10)

    evaluator = InterceptEvaluatorCore(capture_radius=0.25)
    evaluator.begin(manager.mission_id, 10.0)
    evaluator.update(
        10.0,
        KinematicState((0.0, 0.0, -0.1), (2.0, 0.0, 0.0)),
        KinematicState((1.0, 0.0, -0.1), (0.0, 0.0, 0.0)),
    )
    result = evaluator.update(
        10.5,
        KinematicState((2.0, 0.0, -0.1), (2.0, 0.0, 0.0)),
        KinematicState((1.0, 0.0, -0.1), (0.0, 0.0, 0.0)),
    )

    assert accepted == TrajectoryRejectReason.NONE
    assert manager.phase == MissionPhase.MINCO_TRACKING
    assert result.success


def test_planner_failure_retains_old_plan_then_recovers_from_safe_wait():
    manager = MissionManagerCore(plan_recovery_timeout=0.1)
    begin_intercept(manager)
    tracker = TrajectoryTrackerCore()
    tracker.accept(trajectory(), tracker_state(), manager.mission_id)
    manager.observe_tracker(
        manager.mission_id,
        7,
        'PLAN_ACCEPTED',
        0.05,
        1.95,
        10.05,
    )
    manager.tick(10.06)

    manager.observe_planner(False, plan_id=8, now=10.08)
    assert tracker.command(tracker_state(10.10), manager.mission_id) is not None
    assert manager.phase == MissionPhase.MINCO_TRACKING

    assert tracker.command(tracker_state(12.01), manager.mission_id) is None
    manager.observe_tracker(
        manager.mission_id,
        7,
        'NO_VALID_PLAN',
        2.01,
        0.0,
        12.01,
    )
    manager.tick(12.12)
    assert manager.phase == MissionPhase.SAFE_WAIT

    new_trajectory = trajectory(source_stamp=12.2, plan_id=9)
    new_state = TrackerKinematicState(
        stamp=12.25,
        position=(0.05, 0.0, -0.99),
        velocity=(1.0, 0.0, 0.2),
    )
    assert tracker.accept(
        new_trajectory,
        new_state,
        manager.mission_id,
    ) == TrajectoryRejectReason.NONE
    manager.observe_tracker(
        manager.mission_id,
        9,
        'PLAN_ACCEPTED',
        0.05,
        1.95,
        12.25,
    )
    manager.tick(12.26)
    assert manager.phase == MissionPhase.MINCO_TRACKING


def test_mission_reset_immediately_rejects_old_trajectory():
    manager = MissionManagerCore()
    begin_intercept(manager)
    old = trajectory(mission_id=manager.mission_id)
    manager.handle_command('R', now=11.0)
    tracker = TrajectoryTrackerCore()

    rejected = tracker.accept(old, tracker_state(), manager.mission_id)

    assert rejected == TrajectoryRejectReason.MISSION_MISMATCH


def test_async_20_5_20_hz_pipeline_accepts_completed_older_prediction():
    """A 55 ms worker result survives newer prediction frames."""
    first = request()

    def prediction(source_stamp, sequence_id):
        samples = tuple(
            PredictionSample(
                float(index),
                (4.0 * (source_stamp + index - 12.5), 0.0, -0.1),
                (4.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
            for index in range(5)
        )
        return PredictionSeries(
            mission_id=1,
            sequence_id=sequence_id,
            source_stamp=source_stamp,
            valid_until=source_stamp + 4.0,
            samples=samples,
            source='simulation_truth',
        )

    first = PlannerRequest(
        mission_id=1,
        prediction=prediction(10.0, 100),
        uav=first.uav,
    )
    newer = prediction(10.2, 102)
    slot = LatestRequestSlot()
    slot.submit(first)
    worker_request = slot.take()
    started = time.perf_counter()
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(
            lambda: (time.sleep(0.055), worker_request)[1]
        )
        slot.submit(PlannerRequest(1, prediction(10.1, 101), first.uav))
        slot.submit(PlannerRequest(1, newer, first.uav))
        while not future.done():
            time.sleep(0.01)
        completed = future.result()
    published = time.perf_counter()

    assert 0.05 <= published - started < 0.20
    assert published - (started + 0.055) < 0.03
    assert validate_target_shift(
        completed,
        newer,
        intercept_time=3.0,
        tolerance=0.05,
    ) == FastPlanningFailure.NONE

    tracker = TrajectoryTrackerCore()
    assert tracker.accept(
        trajectory(),
        tracker_state(),
        mission_id=1,
        prediction_sequence_id=102,
        target_endpoint=(2.0, 0.0, -0.6),
    ) == TrajectoryRejectReason.NONE
