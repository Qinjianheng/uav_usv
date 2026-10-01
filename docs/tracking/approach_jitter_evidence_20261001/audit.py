"""Recompute this recorded run only; no ROS nodes, fitting or extrapolation."""

from collections import Counter
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from follow_failure_audit import error_stats, percentiles, vector_at  # noqa: E402
from p7_attitude_clock_analysis import decompose, quaternion_at, series  # noqa: E402

BASE = ROOT / 'data/experiments/current/modular_intercept_20260930_164821_621968_mission_1'
ULOG = Path('/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log/'
            '2026-09-30/08_48_06.ulg')
OUTPUT = Path(__file__).resolve().parent


def rows(suffix):
    with Path(str(BASE) + suffix).open() as stream:
        return list(csv.DictReader(stream))


def vec(row, prefix):
    return np.array([float(row[f'{prefix}_{axis}']) for axis in 'xyz'])


def segments(times, values):
    indices = np.r_[0, np.flatnonzero(values[1:] != values[:-1]) + 1, len(values)]
    return [(float(times[a]), float(times[b]) if b < len(times) else float(times[-1]),
             str(values[a])) for a, b in zip(indices[:-1], indices[1:])]


def native_vectors(dataset, keys):
    data = dataset.data
    time_key = 'timestamp_sample' if 'timestamp_sample' in data else 'timestamp'
    times, indices = np.unique(data[time_key] * 1e-6, return_index=True)
    return times, np.column_stack([data[k] for k in keys])[indices]


def main():
    main_rows, vision = rows('.csv'), rows('_vision.csv')
    summary = json.loads(Path(str(BASE) + '_summary.json').read_text())
    start = summary['run_started_at']
    valid = [r for r in vision if r['observation_valid'] == 'True']
    paired = [r for r in valid if r['truth_available'] == 'True']
    log = ULog(str(ULOG), message_name_filter_list=[
        'offboard_control_mode', 'vehicle_local_position_setpoint',
        'vehicle_local_position', 'vehicle_local_position_groundtruth',
        'vehicle_attitude', 'vehicle_attitude_groundtruth',
    ])
    datasets = {d.name: d for d in log.data_list if d.multi_id == 0}
    anchors = sorted(set((float(r['image_clock_anchor_sim_stamp']),
                          float(r['image_clock_anchor_system_stamp'])) for r in vision
                         if float(r['image_clock_anchor_sim_stamp']) > 0.))
    raw, epoch = np.array(anchors).T
    assert np.all(np.diff(raw) > 0.) and np.all(np.diff(epoch) > 0.)
    native_begin = raw[0]
    native_end = min(raw[-1], np.interp(start + float(main_rows[-1]['time']), epoch, raw))

    def elapsed(stamp):
        assert raw[0] <= stamp <= raw[-1]
        return float(np.interp(stamp, raw, epoch) - start)

    mode = datasets['offboard_control_mode'].data
    mt = mode['timestamp'] * 1e-6
    mask = (mt >= native_begin) & (mt <= native_end)
    values = np.where(mode['position'][mask], 'P',
                      np.where(mode['velocity'][mask], 'V', 'OTHER'))
    mode_segments = segments(mt[mask], values)
    internal = datasets['vehicle_local_position_setpoint'].data
    it = internal['timestamp'] * 1e-6
    velocity = np.column_stack([internal[k] for k in ('vx', 'vy')])
    delta = np.linalg.norm(np.diff(velocity, axis=0), axis=1)
    finite = np.isfinite(delta) & (it[1:] >= native_begin) & (it[1:] <= native_end)
    jump_index = int(np.where(finite)[0][np.argmax(delta[finite])])

    gt_t, gt_pos = native_vectors(datasets['vehicle_local_position_groundtruth'], list('xyz'))
    ekf_t, ekf_pos = native_vectors(datasets['vehicle_local_position'], list('xyz'))
    _, gt_vel = native_vectors(datasets['vehicle_local_position_groundtruth'], ['vx', 'vy', 'vz'])
    reversals = []
    for begin, end, value in mode_segments:
        if value != 'P' or end - begin < .6 or elapsed(begin) < 8.5:
            continue
        try:
            entry_velocity = vector_at(begin, gt_t, gt_vel)
        except ValueError:
            continue
        speed = np.linalg.norm(entry_velocity[:2])
        if speed < 4.:
            continue
        direction = entry_velocity[:2] / speed
        select = (gt_t >= begin) & (gt_t < end)
        along = gt_vel[select, :2] @ direction
        negative = np.flatnonzero(along < -.05)
        anchor_index = int(np.searchsorted(it, begin))
        anchor = np.array([internal[k][anchor_index] for k in ('x', 'y')])
        reversals.append({
            'elapsed_start_s': elapsed(begin), 'duration_s': end - begin,
            'entry_gt_horizontal_speed_mps': float(speed),
            'minimum_gt_along_entry_direction_mps': float(along.min()),
            'time_to_reverse_s': (float(gt_t[select][negative[0]] - begin)
                                  if len(negative) else None),
            'maximum_gt_distance_from_hold_anchor_m': float(np.linalg.norm(
                gt_pos[select, :2] - anchor, axis=1).max()),
        })

    estimated = series(datasets['vehicle_attitude'])
    truth = series(datasets['vehicle_attitude_groundtruth'])
    frames, rejected = [], Counter()
    for row in paired:
        if (row['geometry_diagnostics_enabled'] != 'True'
                or row['heading_diagnostics_status'] != 'VALID'):
            rejected['exact_geometry_or_independent_heading_unavailable'] += 1
            continue
        stamp = float(row['rgb_raw_stamp'])
        try:
            image_e, image_g = quaternion_at(stamp, *estimated), quaternion_at(stamp, *truth)
            position_g = vector_at(stamp, gt_t, gt_pos)
            position_e = vector_at(stamp, ekf_t, ekf_pos)
        except ValueError as exc:
            rejected[str(exc)] += 1
            continue
        online = Rotation.from_quat([float(row['interpolated_attitude_' + a])
                                    for a in ('x', 'y', 'z', 'w')])
        model = Rotation.from_quat([float(row['uav_reference_attitude_' + a])
                                   for a in ('x', 'y', 'z', 'w')])
        body = vec(row, 'target_body_flu') * [1., -1., -1.]
        terms = decompose(body, online, image_e, image_g, model)
        measured = vec(row, 'position') - vec(row, 'truth')
        counterfactual = (measured - terms['total'])
        # Same native image time and model-origin NED frame; reference only.
        position_timing = vec(row, 'interpolated_uav') - position_e
        position_estimation = position_e - position_g
        position_total = position_timing + position_estimation
        sensor_residual = counterfactual - position_total
        full_check = measured - (position_total + terms['total'] + sensor_residual)
        assert np.linalg.norm(full_check) < 1e-10
        saved_counterfactual = vec(row, 'attitude_counterfactual_error')
        # The saved counterfactual targets the rendered entity reference.
        check = counterfactual - saved_counterfactual - vec(row, 'entity_to_truth')
        frames.append({
            'elapsed_s': float(row['measurement_stamp']) - start,
            'raw_stamp_s': stamp, 'skew_s': float(row['rgb_depth_acquisition_skew']),
            'raw_error_m': measured.tolist(), 'counterfactual_error_m': counterfactual.tolist(),
            'position_estimation_m': position_estimation.tolist(),
            'position_timing_m': position_timing.tolist(),
            'position_total_m': position_total.tolist(),
            'after_pose_replacement_m': sensor_residual.tolist(),
            'full_pose_decomposition_check_m': full_check.tolist(),
            'decomposition_check_m': check.tolist(),
            'model_position_minus_native_gt_m': (
                vec(row, 'uav_reference_position') - position_g).tolist(),
            **{k: v.tolist() for k, v in terms.items()},
        })
    groups = {}
    for name, selected in (
        ('all', frames),
        ('same_stamp', [r for r in frames if abs(r['skew_s']) < 1e-8]),
        ('different_stamp', [r for r in frames if abs(r['skew_s']) >= 1e-8]),
        ('window_38_43p2_same_stamp', [
            r for r in frames if 38. <= r['elapsed_s'] <= 43.2 and abs(r['skew_s']) < 1e-8]),
        ('window_38_43p2_different_stamp', [
            r for r in frames if 38. <= r['elapsed_s'] <= 43.2 and abs(r['skew_s']) >= 1e-8]),
        ('steady_20_90', [r for r in frames if 20. <= r['elapsed_s'] <= 90.]),
    ):
        groups[name] = {key: error_stats([r[key] for r in selected]) for key in (
            'raw_error_m', 'counterfactual_error_m', 'timing', 'estimation', 'reference',
            'position_estimation_m', 'position_timing_m', 'position_total_m',
            'after_pose_replacement_m')}
    lock_losses = [b for a, b in zip(main_rows[:-1], main_rows[1:])
                   if a['target_locked'] == 'True' and b['target_locked'] == 'False']
    bearing_segments = [s for s in segments(
        np.array([float(r['time']) for r in main_rows]),
        np.array([r['controller_status'] for r in main_rows])) if s[2] == 'BEARING_APPROACH']
    brake_causes = Counter()
    for a, b in zip(main_rows[:-1], main_rows[1:]):
        if a['controller_status'] != 'VISUAL_BRAKING':
            continue
        category = ('rgb_invisible' if a['target_visible'] != 'True'
                    else 'visible_outside_center_gate' if abs(float(
                        a['last_valid_image_bearing'])) > .15
                    else 'visible_without_three_centered_frames' if int(
                        a['consecutive_valid_frames']) < 3 else 'visible_other_gate')
        brake_causes[category] += float(b['time']) - float(a['time'])
    paths = [Path(str(BASE) + suffix) for suffix in (
        '.csv', '_vision.csv', '_summary.json', '_config.yaml')] + [ULOG]
    report = {
        'sources_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        'method': 'Recorded server clock anchors; native timestamp_sample; '
                  'no fit or extrapolation. '
                  'ULog mode sampled at about 10Hz may miss single control frames.',
        'run_metrics': summary['run_metrics'], 'intercept_started': summary['intercept_started'],
        'bearing_segments': len(bearing_segments),
        'bearing_duration_sampled_s': sum(b-a for a, b, _ in bearing_segments),
        'bearing_segment_duration_s': percentiles([b-a for a, b, _ in bearing_segments]),
        'sampled_visual_braking_cause_duration_s': dict(brake_causes),
        'native_offboard_mode_transitions': len(mode_segments)-1,
        'maximum_internal_velocity_jump': {
            'elapsed_s': elapsed(float(it[jump_index+1])),
            'delta_mps': float(delta[jump_index]),
            'interval_s': float(it[jump_index+1]-it[jump_index]),
            'before_xy_mps': velocity[jump_index].tolist(),
            'after_xy_mps': velocity[jump_index+1].tolist(),
        },
        'high_speed_hold_reversals': reversals,
        'lock_losses': {
            'count': len(lock_losses),
            'all_rgb_visible': all(r['target_visible'] == 'True' for r in lock_losses),
            'all_no_lost_rgb_frames': all(int(r['consecutive_lost_frames']) == 0
                                          for r in lock_losses),
            'kf_age_s': percentiles([r['kf_state_age'] for r in lock_losses]),
        },
        'valid_frame_skew_counts_s': dict(Counter(str(round(float(
            r['rgb_depth_acquisition_skew']), 3)) for r in valid)),
        'valid_frame_latency_s': {
            k: percentiles([float(r[b])-float(r[a]) for r in valid]) for k, a, b in (
                ('acquisition_to_receipt', 'measurement_stamp', 'receipt_stamp'),
                ('receipt_to_processed', 'receipt_stamp', 'processed_stamp'),
                ('acquisition_to_published', 'measurement_stamp', 'published_stamp'),
            )},
        'paired_geometry_groups': groups, 'native_bracket_rejections': dict(rejected),
        'maximum_decomposition_check_m': max(
            (np.linalg.norm(r['decomposition_check_m']) for r in frames), default=None),
        'maximum_full_pose_decomposition_check_m': max(
            (np.linalg.norm(r['full_pose_decomposition_check_m']) for r in frames),
            default=None),
        'recorded_visual_errors': error_stats([vec(r, 'position')-vec(r, 'truth')
                                              for r in paired]),
        'model_position_vs_native_gt': error_stats([
            r['model_position_minus_native_gt_m'] for r in frames]),
        'px4_initial_magnetic_parameters': {k: log.initial_parameters.get(k) for k in (
            'CAL_MAG0_ID', 'CAL_MAG0_XOFF', 'CAL_MAG0_YOFF', 'CAL_MAG0_ZOFF', 'EKF2_MAG_DECL')},
        'limitations': ['Offline recorded-run audit; see hashed inputs and validation report.',
                        'Different RGB/depth timestamps correlate with outliers; original images '
                        'were not saved, so mask/depth pixel-level causality is not certified.',
                        'Truth and native groundtruth are used by this offline audit only.'],
    }
    (OUTPUT / 'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (OUTPUT / 'paired_frames.jsonl').write_text(''.join(
        json.dumps(r, allow_nan=False)+'\n' for r in frames))
    print(json.dumps({k: report[k] for k in (
        'native_offboard_mode_transitions', 'maximum_internal_velocity_jump',
        'high_speed_hold_reversals', 'lock_losses', 'valid_frame_skew_counts_s',
        'maximum_decomposition_check_m')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=BASE)
    parser.add_argument('--ulog', type=Path, default=ULOG)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    arguments = parser.parse_args()
    BASE, ULOG, OUTPUT = arguments.base, arguments.ulog, arguments.output
    OUTPUT.mkdir(parents=True, exist_ok=True)
    main()
