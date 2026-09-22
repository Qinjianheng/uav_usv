import json
import math

import pytest

from uav_control.evaluation import intercept_evaluator
from uav_control.evaluation.intercept_evaluator import ExperimentArtifactWriter
from uav_control.evaluation.intercept_evaluator import InterceptEvaluatorCore
from uav_control.evaluation.intercept_evaluator import KinematicState
from uav_control.evaluation.intercept_evaluator import PlannerEventAccumulator
from uav_control.evaluation.intercept_evaluator import VisionMetricAccumulator


def state(position, velocity=(0.0, 0.0, 0.0)):
    return KinematicState(tuple(position), tuple(velocity))


def test_csv_separates_prediction_and_trajectory_age():
    assert 'prediction_age' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'trajectory_age' in ExperimentArtifactWriter.CSV_FIELDS


def test_daily_csv_keeps_decision_fields_and_moves_verbose_planner_detail():
    assert 'planner_failure_reason' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_result' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_event_id' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'approach_phase' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'terminal_admission' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'terminal_admission_reason' in ExperimentArtifactWriter.CSV_FIELDS
    assert 'body_clearance' in ExperimentArtifactWriter.CSV_FIELDS
    fields = ExperimentArtifactWriter.CSV_FIELDS
    assert 'planner_candidate_diagnostics' not in fields
    assert 'planner_rejection_detail' not in fields
    assert 'planner_planning_cycle_id' not in fields


def test_visual_metrics_keep_raw_and_kf_errors_separate():
    metrics = VisionMetricAccumulator()
    metrics.observe_raw(
        source='front_rgbd_red_sphere',
        measurement_stamp=10.0,
        receipt_stamp=10.04,
        estimate=(1.2, 1.8, 0.1),
        truth=(1.0, 2.0, 0.0),
        valid=True,
        distance_bin='MID',
        motion_regime='TURNING',
        approach_phase='PREPARATION',
    )
    metrics.observe_kf(
        stamp=10.0,
        position=(1.1, 2.0, 0.0),
        velocity=(3.5, 0.0, 0.0),
        truth_position=(1.0, 2.0, 0.0),
        truth_velocity=(4.0, 0.0, 0.0),
    )

    summary = metrics.summary()

    assert summary['front']['valid_observation_rate'] == pytest.approx(1.0)
    assert summary['front']['raw_position_3d']['count'] == 1
    assert summary['front']['raw_position_horizontal']['count'] == 1
    assert summary['front']['observation_age']['p50'] == pytest.approx(0.04)
    assert summary['strata'][0]['camera'] == 'front'
    assert summary['strata'][0]['distance_bin'] == 'MID'
    assert summary['strata'][0]['motion_regime'] == 'TURNING'
    assert summary['strata'][0]['approach_phase'] == 'PREPARATION'
    assert summary['strata'][0]['raw_position_3d']['count'] == 1
    assert summary['kf_position_3d']['count'] == 1
    assert summary['kf_velocity_3d']['rmse'] == pytest.approx(0.5)


def test_visual_loss_duration_counts_explicit_invalid_interval_only():
    metrics = VisionMetricAccumulator()
    metrics.observe_raw(
        source='front', measurement_stamp=10.0, receipt_stamp=10.0,
        estimate=(0.0, 0.0, 0.0), truth=(0.0, 0.0, 0.0), valid=True,
    )
    metrics.observe_raw(
        source='front', measurement_stamp=0.0, receipt_stamp=10.5,
        estimate=(math.nan,) * 3, truth=(math.nan,) * 3, valid=False,
    )
    metrics.observe_raw(
        source='front', measurement_stamp=11.0, receipt_stamp=11.1,
        estimate=(0.0, 0.0, 0.0), truth=(0.0, 0.0, 0.0), valid=True,
    )

    assert metrics.summary()['front']['longest_continuous_loss'] == (
        pytest.approx(0.6)
    )


def test_visual_no_measurements_report_no_data_instead_of_zero_error():
    metrics = VisionMetricAccumulator()
    metrics.observe_raw(
        source='front', measurement_stamp=0.0, receipt_stamp=10.5,
        estimate=(math.nan,) * 3, truth=(math.nan,) * 3, valid=False,
    )

    summary = metrics.summary()['front']
    assert summary['raw_position_3d']['count'] == 0
    assert not summary['raw_position_3d']['available']
    assert summary['raw_position_3d']['rmse'] is None
    assert summary['observation_age']['count'] == 0


def test_visual_summary_counts_distinct_rejection_reasons():
    metrics = VisionMetricAccumulator()
    for reason in (
        'IMAGE_CLOCK_REFERENCE_UNAVAILABLE',
        'IMAGE_CLOCK_REFERENCE_UNAVAILABLE',
        'IMAGE_TIMESTAMP_STALE',
    ):
        metrics.observe_raw(
            source='front_rgbd_red_sphere',
            measurement_stamp=0.0,
            receipt_stamp=10.0,
            estimate=(math.nan,) * 3,
            truth=(math.nan,) * 3,
            valid=False,
            rejection_reason=reason,
        )

    assert metrics.summary()['front']['rejection_histogram'] == {
        'IMAGE_CLOCK_REFERENCE_UNAVAILABLE': 2,
        'IMAGE_TIMESTAMP_STALE': 1,
    }


def test_optional_visual_event_file_is_event_based(tmp_path):
    writer = ExperimentArtifactWriter(
        tmp_path,
        mission_id=3,
        config={},
        prefix='vision',
        visual_evaluation_enabled=True,
    )

    assert writer.append_visual_event({
        'measurement_stamp': 10.0,
        'receipt_stamp': 10.04,
        'source': 'front_rgbd_red_sphere',
        'valid': True,
    })
    paths = writer.finalize({'outcome': 'TEST'})

    lines = paths.visual_path.read_text(encoding='utf-8').splitlines()
    assert len(lines) == 2
    assert 'measurement_stamp' in lines[0]
    assert 'position_source_stamp' in lines[0]
    assert 'attitude_history_start_stamp' in lines[0]
    assert 'px4_clock_reset_count' in lines[0]
    assert 'px4_clock_calibration_count' in lines[0]
    assert 'px4_clock_recalibration_count' in lines[0]
    assert 'image_measurement_stamp' in lines[0]
    assert 'image_clock_mapping_mode' in lines[0]
    assert 'image_clock_status' in lines[0]
    assert 'image_clock_reset_count' in lines[0]
    assert 'image_clock_anchor_sim_stamp' in lines[0]
    assert 'image_clock_anchor_system_stamp' in lines[0]
    assert 'image_clock_reference_age' in lines[0]
    assert 'image_clock_sync_quality' in lines[0]
    assert 'image_measurement_time_source' in lines[0]
    assert 'front_rgbd_red_sphere' in lines[1]


def test_visual_csv_preserves_distinct_image_clock_failure_reasons(tmp_path):
    writer = ExperimentArtifactWriter(
        tmp_path,
        mission_id=4,
        config={},
        prefix='clock_failures',
        visual_evaluation_enabled=True,
    )
    for reason, status in (
        ('IMAGE_CLOCK_REFERENCE_UNAVAILABLE',
         'CLOCK_REFERENCE_UNAVAILABLE'),
        ('IMAGE_CLOCK_RESET', 'SIM_TIME_RESET'),
        ('IMAGE_TIMESTAMP_IN_FUTURE', 'MAPPED_INTERPOLATED'),
        ('IMAGE_TIMESTAMP_STALE', 'MAPPED_INTERPOLATED'),
    ):
        writer.append_visual_event({
            'measurement_stamp': 10.0,
            'receipt_stamp': 10.1,
            'source': 'front_rgbd_red_sphere',
            'valid': False,
            'rejection_reason': reason,
            'image_clock_mapping_mode': (
                'GAZEBO_CLOCK_SYSTEM_INTERPOLATION'
            ),
            'image_clock_status': status,
        })
    path = writer.finalize({'outcome': 'TEST'}).visual_path
    contents = path.read_text(encoding='utf-8')

    assert 'IMAGE_CLOCK_REFERENCE_UNAVAILABLE' in contents
    assert 'IMAGE_CLOCK_RESET' in contents
    assert 'IMAGE_TIMESTAMP_IN_FUTURE' in contents
    assert 'IMAGE_TIMESTAMP_STALE' in contents
    assert 'GAZEBO_CLOCK_SYSTEM_INTERPOLATION' in contents


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
    """Catch misses from pairing latest samples at different times."""
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
    assert summary['planner_primary_failure_reason'] == 'DEADLINE_EXCEEDED'
    assert summary['planner_success_rate'] == pytest.approx(0.5)
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
    assert summary['planner_primary_failure_reason'] == ''
    assert summary['planner_success_rate'] == 0.0
    assert summary['tracker_rejection_histogram'] == {}
    assert summary['planner_source_age_at_publish_p95'] == 0.0
    assert summary['planner_publish_delay_p95'] == 0.0


def test_preparation_admission_wait_is_not_counted_as_planner_failure():
    metrics = PlannerEventAccumulator()
    metrics.observe_planner(
        mission_id=2,
        plan_id=1,
        success=False,
        failure_reason='NONE',
        compute_time=0.02,
        generation_time=0.01,
        completion_stamp=1.0,
        admission_wait=True,
    )

    summary = metrics.summary(elapsed_time=1.0)

    assert summary['planner_admission_wait'] == 1
    assert summary['planner_failed'] == 0
    assert summary['planner_failure_histogram'] == {}


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


def test_optional_detailed_diagnostics_are_event_based_jsonl(tmp_path):
    writer = ExperimentArtifactWriter(
        log_directory=tmp_path,
        mission_id=4,
        config={'detailed_diagnostics_enabled': True},
        prefix='detail_test',
        detailed_diagnostics_enabled=True,
    )
    writer.append_sample({'time': 0.0, 'distance': 1.2})
    writer.append_detail_event('planner', '4:7', {'candidate_count': 3})
    paths = writer.finalize({'outcome': 'FAILURE'})

    assert paths.diagnostics_path is not None
    lines = paths.diagnostics_path.read_text(encoding='utf-8').splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0]) == {
        'event_type': 'planner',
        'event_id': '4:7',
        'candidate_count': 3,
    }


def test_csv_records_camera_shadow_prediction_errors():
    for prefix in ('kf', 'shadow_bctra'):
        for label in ('0p5', '1p0', '2p0'):
            field = f'{prefix}_prediction_{label}_error'
            assert field in ExperimentArtifactWriter.CSV_FIELDS
