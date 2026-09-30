# Current P8.1 causal pose callback fixture repair

Only `src/uav_control/test/test_rgbd_target_localizer.py` changed in this task.
Production `rgbd_target_localizer.py` remains unchanged (empty current Git diff).
No Gazebo, commit, subagent or build was started by this task.

Root build log `/tmp/p8_visual_build.log` confirms all 3 packages built successfully
before final tests: uav_usv_interfaces, uav_control, uav_usv_bringup (2 min 1 s).

## Fixture architecture

New `_make_causal_pose_localizer` initializes the production callback state,
including `timesync_history`, pending raw/legacy pose queues, histories, cache
stamps and the trusted Gazebo sim/system anchor mapper. No receipt-time mapper is
used for callback tests. `_send_timesync` uses real `TimesyncStatus` messages;
`_send_pose` uses real PX4 position/attitude messages and production callbacks.

The fixture deliberately distinguishes all three timestamp domains:

- uXRCE-converted PX4 source sample: approximately 50 s.
- causal `estimated_offset`: -40 s, recovering raw PX4/Gazebo sample ≈10 s.
- server Clock maps Gazebo sim≈10 s to system/ROS≈100 s.
- DDS receipt is separately delayed to approximately 100.42–100.49 s.

Thus either direct source-stamp use, receipt-based calibration, or newest/future
timesync fallback fails the expectations. Anchors are available before the delayed
sample receipt; interpolation-only behavior is tested explicitly.

## Seven outdated callback tests repaired

1. Old node receipt-offset recalibration test becomes
   `test_causal_timesync_offset_change_preserves_previous_pose_history`.
   Timesync offsets at 49.9, 50.1 and 50.3 differ; a delayed sample at 50.2 must use
   the 50.1 offset, while 50.05 still uses 49.9. Old good history is preserved.
2. Cross-topic interleaving checks exact independent position/attitude history
   ranges rather than reset counters from the unused legacy receipt mapper.
3. One bad position checks missing causal offset, older mapped position and a
   position beyond the clock bracket; valid opposite attitude history and good
   position cache remain intact, with only the future raw sample pending.
4. Confirmed PX4 restart uses the actual current trigger: timesync source rewind.
   Both histories, both UAV caches, mapped stamps, waiting RGB-D pair and raw/legacy
   pending samples clear. A new causal offset then populates the new epoch.
5. Mapped-measurement callback test verifies interpolation at acquisition time,
   raw sample recovery and explicit inequality from DDS receipt.
6. Timestamp-sample callback test uses different `timestamp`/`timestamp_sample`
   values and verifies the sample stamp is selected before offset recovery and
   Gazebo mapping. Its name now states that behavior rather than obsolete direct
   ROS domain mode.
7. Antipodal quaternion callback fixture now maps through the causal chain and
   verifies sign continuity at an interpolated acquisition time and latest cache.

All pure `Px4RosClockMapper` tests and pure math/geometry tests remain unchanged.

## Three added current-chain regressions

- A future-only timesync sample cannot initialize either pose history or queue an
  invented raw sample; receipt/newest-offset fallback is rejected.
- Poses beyond the Gazebo right bracket remain raw/pending, then flush exactly to
  acquisition time when a genuine later anchor arrives; attitude sign continuity
  survives the flush.
- Gazebo sim rewind through `gazebo_clock_callback` clears histories, pending raw
  samples and waiting image pair, and raises the image clock reset flag.

## Final focused checks

After sourcing `/opt/ros/humble/setup.bash` and workspace `install/setup.bash`:

```
PYTHONPATH=src/uav_control:$PYTHONPATH python3 -m pytest -q src/uav_control/test/test_rgbd_target_localizer.py
```

**75 passed** in 0.41 s. Own test file `ament_flake8` and `ament_pep257`:
**no problems found**.

Full package `pytest -q --tb=no` after these repairs: **591 passed, 5 failed,
1 skipped** in 5.47 s. The remaining failures outside this task were sent to root:

- `test_flake8.py::test_flake8` (one concurrent integration-file style problem)
- `test_intercept_evaluation.py::test_failure_result_freezes_target_and_requests_gazebo_pause`
- `test_modular_configuration.py::test_main_prediction_uses_kf_while_direct_tracker_truth_is_explicit`
- `test_trajectory_tracker_node.py::test_future_trajectory_waits_for_state_at_execution_start_then_accepts`
- `test_trajectory_tracker_node.py::test_pending_trajectory_that_becomes_too_old_is_rejected`

This verifies callback/unit behavior only; it makes no new live Gazebo, tracking
rate, RMSE or flight claims and does not modify the validated P8.1 time chain.
