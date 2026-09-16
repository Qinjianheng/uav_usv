import json

import pytest

from uav_control.evaluation.intercept_evaluator import ExperimentArtifactWriter
from uav_control.evaluation.intercept_evaluator import InterceptEvaluatorCore
from uav_control.evaluation.intercept_evaluator import KinematicState
from uav_control.evaluation.intercept_evaluator import PlannerEventAccumulator


def state(position, velocity=(0.0, 0.0, 0.0)):
    return KinematicState(tuple(position), tuple(velocity))


def test_capture_is_detected_between_truth_samples():
    evaluator = InterceptEvaluatorCore(capture_radius=0.25)
    evaluator.begin(mission_id=4, now=10.0)

    assert evaluator.update(
        10.0,
        state((0.0, 0.0, -0.1), (2.0, 0.0, 0.0)),
        state((1.0, 0.0, -0.1)),
    ) is None
    result = evaluator.update(
        10.5,
        state((2.0, 0.0, -0.1), (2.0, 0.0, 0.0)),
        state((1.0, 0.0, -0.1)),
    )

    assert result.success
    assert result.reason == 'CAPTURE_RADIUS_REACHED'
    assert result.minimum_distance == pytest.approx(0.0)


def test_sea_contact_before_capture_is_a_failure():
    evaluator = InterceptEvaluatorCore(
        capture_radius=0.25,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
    )
    evaluator.begin(mission_id=5, now=20.0)
    evaluator.update(
        20.0,
        state((0.0, 0.0, -0.1), (0.0, 0.0, 0.4)),
        state((10.0, 0.0, 0.0)),
    )

    result = evaluator.update(
        20.5,
        state((0.0, 0.0, 0.1), (0.0, 0.0, 0.4)),
        state((10.0, 0.0, 0.0)),
    )

    assert not result.success
    assert result.reason == 'SEA_CONTACT'


def test_planner_statistics_are_unique_per_plan_event():
    metrics = PlannerEventAccumulator()

    assert metrics.observe_planner(
        mission_id=2,
        plan_id=7,
        success=True,
        failure_reason='NONE',
        compute_time=0.02,
        generation_time=0.01,
        completion_stamp=1.0,
    )
    assert not metrics.observe_planner(
        mission_id=2,
        plan_id=7,
        success=True,
        failure_reason='NONE',
        compute_time=0.02,
        generation_time=0.01,
        completion_stamp=1.0,
    )
    metrics.observe_planner(
        mission_id=2,
        plan_id=8,
        success=False,
        failure_reason='DEADLINE_EXCEEDED',
        compute_time=0.08,
        generation_time=0.03,
        completion_stamp=1.5,
    )
    metrics.observe_controller(2, 7, 'PLAN_ACCEPTED')
    metrics.observe_controller(2, 7, 'TRACKING')
    metrics.observe_controller(2, 0, 'NO_VALID_PLAN')

    summary = metrics.summary(elapsed_time=2.0)

    assert summary['planner_started'] == 2
    assert summary['planner_completed'] == 2
    assert summary['planner_succeeded'] == 1
    assert summary['planner_failed'] == 1
    assert summary['planner_deadline'] == 1
    assert summary['planner_failure_histogram'] == {
        'DEADLINE_EXCEEDED': 1,
    }
    assert summary['attempt_rate'] == pytest.approx(1.0)
    assert summary['execution_rate'] == pytest.approx(1.0)
    assert summary['hold_rate'] == pytest.approx(1.0 / 3.0)


def test_empty_event_rates_are_numeric_not_null():
    summary = PlannerEventAccumulator().summary(elapsed_time=0.0)

    assert summary['attempt_rate'] == 0.0
    assert summary['execution_rate'] == 0.0
    assert summary['hold_rate'] == 0.0
    assert summary['actual_completion_hz'] == 0.0
    assert summary['planner_failure_histogram'] == {}


def test_artifact_writer_creates_csv_summary_and_config(tmp_path):
    writer = ExperimentArtifactWriter(
        log_directory=tmp_path,
        mission_id=3,
        config={'capture_radius': 0.25, 'truth_topic': '/target/state'},
        prefix='unit_test',
    )
    writer.append_sample({'time': 0.0, 'distance': 1.2})

    paths = writer.finalize({
        'outcome': 'FAILURE',
        'failure_reason': 'TIMEOUT',
        'attempt_rate': 0.0,
        'execution_rate': 0.0,
        'hold_rate': 0.0,
    })

    assert paths.csv_path.exists()
    assert paths.summary_path.exists()
    assert paths.config_path.exists()
    summary = json.loads(paths.summary_path.read_text(encoding='utf-8'))
    assert summary['mission_id'] == 3
    assert summary['attempt_rate'] == 0.0
