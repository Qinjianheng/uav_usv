# P8.5 offline visual-bias analyzer

Created only:

- `scripts/p8_visual_bias_analysis.py`
- `src/uav_control/test/test_p8_visual_bias_analysis.py`

## Usage

```bash
python3 /home/qin/data/uav_usv/scripts/p8_visual_bias_analysis.py \
  --vision /absolute/path/to/vision.csv \
  --output /absolute/path/to/p8_bias.json
```

With an explicitly prepared same-acquisition, causal, same-NED-frame heading reference:

```bash
python3 /home/qin/data/uav_usv/scripts/p8_visual_bias_analysis.py \
  --vision /absolute/path/to/vision.csv \
  --heading-reference /absolute/path/to/prealigned_heading.csv \
  --output /absolute/path/to/p8_bias.json
```

The optional heading CSV columns are `measurement_stamp,px4_heading,reference_heading`. Producer must establish causal correspondence and common frame; analyzer verifies positive finite timestamps/headings and numeric timestamp equality within 1e-6 s, performs no temporal interpolation/shift, and rejects ambiguous duplicate matches. It never derives a reference heading from target truth direction or attitude quaternion columns.

## Statistics and units

- Valid rate: observation-valid CSV rows / all CSV rows (dimensionless); missing/empty data yields JSON null rather than a zero success rate.
- Raw horizontal RMSE: `sqrt(mean(error_x^2 + error_y^2))`, metres, recomputed from signed components. Selection requires observation valid, truth available, positive image acquisition stamp and finite XY errors. Y mean preserves its sign (metres).
- `POSITION_TIMESTAMP_AFTER_HISTORY` row count comes from `rejection_reason`, counted over all CSV rows.
- Image bearing: `atan((mask_centroid_u-camera_cx)/camera_fx)`, radians. Requires enabled geometry diagnostics and actual finite centroid/camera fields with fx > 0; missing/disabled/invalid fields produce null, never a bogus zero bearing.
- For exact-acquisition heading matches with enabled finite relative geometry: wrap `delta=px4_heading-reference_heading` into [-pi,pi); scalar proxy `target_range*sin(delta)` in metres; signed NED proxy `estimated_rel - Rz(-delta)*estimated_rel`, where `estimated_rel=(position_x-interpolated_uav_x,position_y-interpolated_uav_y)`.
- Reports signed X/Y proxy correlations with corresponding measured errors (dimensionless), matched raw horizontal RMSE, and residual horizontal RMSE after subtracting this externally specified geometric proxy (metres). It performs no fit, constant offset search, yaw correction or delay estimation.
- JSON marks `offline_only=true`, `online_correction="none"`, and `root_cause="not_confirmed"`. Missing reference or required geometry yields `heading_diagnostic.status="not_evaluated"`, reason `insufficient_same_time_reference`; counts identify unmatched, ambiguous and geometry-insufficient rows. Singular correlations return null. JSON export disallows NaN/Infinity; RMSE uses scaled arithmetic to avoid overflow from large finite CSV errors.
- No ROS import, publishers, subscriptions, Gazebo interaction, or online production changes.

## Verification

TDD first run failed because the requested analyzer did not exist. After implementation, **15 tests passed**. Additional numeric robustness test first failed with infinite RMSE for finite 1e200 errors, then passed after scaled RMSE. Tests cover latest-like negative Y bias/RMSE, all-row valid-rate denominator, timestamp-after-history counts, image bearing geometry requirements, same-acquisition explicit references, excluded misalignment, maximum 1 us numerical tolerance, absent reference despite quaternion/truth fields, missing geometry, duplicate ambiguity, wrapped headings, CLI JSON nulls, finite large errors, and empty input.

`python3 -m flake8` on both new files: passed. `py_compile`: passed. Diff whitespace check: passed. Package-wide pytest was left to root during concurrent integration, per root's instruction.

## Existing historical CSV smoke result (not a new simulation)

Input: `/home/qin/data/uav_usv/data/experiments/current/modular_intercept_20260929_213155_029414_mission_1_vision.csv` (latest available file by mtime).

Output: `/tmp/p8_bias_latest_available.json`.

- Total rows: 49; valid rows: 47; valid rate: 95.918367%.
- Raw horizontal RMSE: **0.5661656904 m**.
- Y mean: **-0.5197911273 m**; X mean: 0.1567864470 m.
- `POSITION_TIMESTAMP_AFTER_HISTORY`: 0 rows in this file.
- Geometry diagnostics were unavailable/disabled: bearing is null/not_evaluated.
- No same-time heading reference was supplied: yaw attribution is not_evaluated / insufficient_same_time_reference.

These values reproduce the roughly 0.57 m horizontal error and negative Y bias described in the requirements. They do not confirm PX4 yaw bias as the root cause. User-run staged simulations and fresh geometry-enabled records remain necessary; no new closed-loop or Gazebo experiment was performed.
