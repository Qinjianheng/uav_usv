import json

import pytest

from uav_control.evaluation import intercept_evaluator
from uav_control.evaluation.intercept_evaluator import ExperimentArtifactWriter
from uav_control.evaluation.intercept_evaluator import InterceptEvaluatorCore
from uav_control.evaluation.intercept_evaluator import KinematicState
from uav_control.evaluation.intercept_evaluator import PlannerEventAccumulator


def state(position, velocity=(0.0, 0.0, 0.0)):
    return KinematicState(tuple(position), tuple(velocity))


def test_csv_separates_prediction_and_trajectory_age():
    assert 'prediction_age' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'trajectory_age' in ExperimentArtifactWriter.CSV_FIELDS


def test_csv_records_planner_failure_diagnostics():
    assert 'planner_failure_reason' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_failure_detail' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_reachability_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_generation_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_validation_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_candidate_diagnostics' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'attempted_plan_id' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_planning_cycle_id' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_rejection_stage' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_rejection_detail' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_contact_recovery_reason' in (
        ExperimentArtifactWriter.CSV_FIELDS
    )
    assert 'planner_contact_delay' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_target_prediction_shift' in (
        ExperimentArtifactWriter.CSV_FIELDS
    )
    assert 'planner_candidate_published' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_required_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_horizontal_min_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_vertical_min_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_sea_safe_min_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_search_min_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_search_max_time' in ExperimentArtifactWriter.CSV_FIELDS
    assert (
        'planner_available_prediction_duration'
        in ExperimentArtifactWriter.CSV_FIELDS
    )
    assert (
        'planner_locked_remaining_t_go'
        in ExperimentArtifactWriter.CSV_FIELDS
    )


def test_capture_is_detected_between_truth_samples():
    # Isolate capture interpolation from the body-water barrier: these samples
    # sit at the sea-level target altitude, which already counts as body
    # contact once the airframe extent is applied.
    evaluator = InterceptEvaluatorCore(
        capture_radius=0.25,
        body_lower_extent=0.0,
    )
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


def test_async_truth_histories_are_interpolated_before_capture_evaluation():
    """Catch false misses caused by pairing latest samples from different times."""
    uav_history = intercept_evaluator.TimestampedStateHistory(0.25)
    target_history = intercept_evaluator.TimestampedStateHistory(0.25)
    # Same isolation as above: this test is about history interpolation.
    evaluator = InterceptEvaluatorCore(
        capture_radius=0.50,
        body_lower_extent=0.0,
    )
    evaluator.begin(mission_id=8, now=10.025)

    uav_history.add(10.00, state((0.0, 0.0, -0.1)))
    uav_history.add(10.05, state((0.0, 0.0, -0.1)))
    target_history.add(
        10.025,
        state((-0.70, 0.0, -0.1), (4.0, 0.0, 0.0)),
    )
    synchronized = intercept_evaluator.synchronize_histories(
        uav_history,
        target_history,
    )
    assert synchronized is not None
    stamp, uav, target = synchronized
    assert stamp == pytest.approx(10.025)
    assert evaluator.update(stamp, uav, target) is None

    target_history.add(
        10.075,
        state((-0.50, 0.0, -0.1), (4.0, 0.0, 0.0)),
    )
    uav_history.add(10.10, state((0.0, 0.0, -0.1)))
    stamp, uav, target = intercept_evaluator.synchronize_histories(
        uav_history,
        target_history,
    )
    result = evaluator.update(stamp, uav, target)

    assert stamp == pytest.approx(10.075)
    assert result.success
    assert result.minimum_distance == pytest.approx(0.50)


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


def test_body_contact_fires_while_the_reference_point_is_still_dry():
    """
    Catch a water strike being missed while the gear is already submerged.

    The X500 skids reach 0.227 m below the PX4 local-position reference, so a
    reference altitude of -0.1 m already means the airframe is in the water
    even though the old "reference point at z >= 0" test saw nothing.
    """
    evaluator = InterceptEvaluatorCore(
        capture_radius=0.25,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
        body_lower_extent=0.23,
    )
    evaluator.begin(mission_id=6, now=30.0)
    assert evaluator.body_contact_z == pytest.approx(-0.23)
    assert evaluator.update(
        30.0,
        state((0.0, 0.0, -0.6), (0.0, 0.0, 0.5)),
        state((10.0, 0.0, 0.0)),
    ) is None

    result = evaluator.update(
        30.5,
        state((0.0, 0.0, -0.1), (0.0, 0.0, 0.5)),
        state((10.0, 0.0, 0.0)),
    )

    assert result is not None
    assert not result.success
    assert result.reason == 'SEA_CONTACT'
    assert result.detail == 'BODY_LOWEST_POINT_AT_SEA_SURFACE'


def test_submerged_reference_point_reports_the_deeper_contact_detail():
    evaluator = InterceptEvaluatorCore(
        capture_radius=0.25,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
        body_lower_extent=0.23,
    )
    evaluator.begin(mission_id=7, now=40.0)
    assert evaluator.update(
        40.0,
        state((0.0, 0.0, -0.30), (0.0, 0.0, 0.5)),
        state((10.0, 0.0, 0.0)),
    ) is None

    result = evaluator.update(
        40.5,
        state((0.0, 0.0, 0.05), (0.0, 0.0, 0.5)),
        state((10.0, 0.0, 0.0)),
    )

    assert result.reason == 'SEA_CONTACT'
    assert result.detail == 'REFERENCE_POINT_BELOW_SEA_SURFACE'


def test_zero_body_lower_extent_keeps_the_reference_point_criterion():
    evaluator = InterceptEvaluatorCore(
        capture_radius=0.25,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
        body_lower_extent=0.0,
    )
    evaluator.begin(mission_id=8, now=50.0)

    assert evaluator.body_contact_z == pytest.approx(0.0)
    assert evaluator.update(
        50.0,
        state((0.0, 0.0, -0.10), (0.0, 0.0, 0.4)),
        state((10.0, 0.0, 0.0)),
    ) is None
    result = evaluator.update(
        50.5,
        state((0.0, 0.0, 0.05), (0.0, 0.0, 0.4)),
        state((10.0, 0.0, 0.0)),
    )

    assert result.reason == 'SEA_CONTACT'


def test_capture_inside_the_body_contact_band_still_succeeds():
    """A capture must be credited before the airframe reaches the water."""
    evaluator = InterceptEvaluatorCore(
        capture_radius=0.50,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
        body_lower_extent=0.23,
    )
    evaluator.begin(mission_id=9, now=60.0)
    evaluator.update(
        60.0,
        state((-1.0, 0.0, -0.45), (2.0, 0.0, 0.3)),
        state((0.0, 0.0, 0.0)),
    )
    result = evaluator.update(
        60.5,
        state((0.0, 0.0, -0.30), (2.0, 0.0, 0.3)),
        state((0.0, 0.0, 0.0)),
    )

    assert result is not None
    assert result.success
    assert result.reason == 'CAPTURE_RADIUS_REACHED'


def test_control_unrecoverable_alone_is_not_a_water_contact():
    """A barrier warning is not a strike; only airframe geometry terminates."""
    from uav_control.common.sea_safety import apply_sea_safety_guard

    guard = apply_sea_safety_guard(
        current_z=-0.26,
        current_vz=1.0,
        proposed_vz=1.0,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        response_delay=0.15,
        effective_braking_acceleration=2.5,
        control_dt=0.05,
    )
    assert guard.state == 'UNRECOVERABLE'

    evaluator = InterceptEvaluatorCore(
        capture_radius=0.25,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
        body_lower_extent=0.23,
    )
    evaluator.begin(mission_id=10, now=70.0)
    # z = -0.26 is still above the -0.23 body-contact altitude, so the
    # evaluator must not manufacture a strike from the barrier state.
    evaluator.update(
        70.0,
        state((0.0, 0.0, -0.26), (0.0, 0.0, 1.0)),
        state((10.0, 0.0, 0.0)),
    )
    result = evaluator.update(
        70.5,
        state((0.0, 0.0, -0.26), (0.0, 0.0, 1.0)),
        state((10.0, 0.0, 0.0)),
    )

    assert result is None


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
        input_age_at_publish=0.06,
        completion_to_publish_delay=0.005,
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
        input_age_at_publish=0.07,
        completion_to_publish_delay=0.006,
    )
    metrics.observe_controller(2, 7, 'PLAN_ACCEPTED')
    metrics.observe_controller(2, 7, 'TRACKING')
    metrics.observe_controller(2, 0, 'NO_VALID_PLAN')
    metrics.observe_controller(
        2,
        8,
        'PLAN_REJECTED',
        'TARGET_ENDPOINT_MISMATCH',
    )

    summary = metrics.summary(elapsed_time=2.0)

    assert summary['planner_started'] == 2
    assert summary['planner_completed'] == 2
    assert summary['planner_succeeded'] == 1
    assert summary['planner_failed'] == 1
    assert summary['planner_deadline'] == 1
    assert summary['planner_source_age_at_publish_p95'] == pytest.approx(
        0.07
    )
    assert summary['planner_publish_delay_p95'] == pytest.approx(0.006)
    assert summary['planner_failure_histogram'] == {
        'DEADLINE_EXCEEDED': 1,
    }
    assert summary['tracker_rejection_histogram'] == {
        'TARGET_ENDPOINT_MISMATCH': 1,
    }
    assert summary['attempt_rate'] == pytest.approx(1.0)
    assert summary['execution_rate'] == pytest.approx(1.0)
    assert summary['hold_rate'] == pytest.approx(1.0 / 4.0)


def test_empty_event_rates_are_numeric_not_null():
    summary = PlannerEventAccumulator().summary(elapsed_time=0.0)

    assert summary['attempt_rate'] == 0.0
    assert summary['execution_rate'] == 0.0
    assert summary['hold_rate'] == 0.0
    assert summary['actual_completion_hz'] == 0.0
    assert summary['planner_failure_histogram'] == {}
    assert summary['tracker_rejection_histogram'] == {}
    assert summary['planner_source_age_at_publish_p95'] == 0.0
    assert summary['planner_publish_delay_p95'] == 0.0


def test_runtime_performance_is_grouped_by_target_distance():
    """Catch near-target camera load being hidden by one whole-run average."""
    metrics = intercept_evaluator.RuntimePerformanceAccumulator()
    metrics.observe(12.0, {'tracker_hz': 20.0, 'front_rgb_hz': 19.0})
    metrics.observe(3.0, {'tracker_hz': 19.5, 'front_rgb_hz': 10.0})
    metrics.observe(1.5, {'tracker_hz': 19.0, 'front_rgb_hz': 7.0})

    summary = metrics.summary()

    assert summary['greater_than_10m']['front_rgb_hz']['p50'] == 19.0
    assert summary['between_2m_and_5m']['front_rgb_hz']['p50'] == 10.0
    assert summary['less_than_2m']['front_rgb_hz']['p50'] == 7.0


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


def test_csv_records_camera_shadow_prediction_errors():
    for prefix in ('kf', 'shadow_bctra'):
        for label in ('0p5', '1p0', '2p0'):
            field = f'{prefix}_prediction_{label}_error'
            assert field in ExperimentArtifactWriter.CSV_FIELDS
