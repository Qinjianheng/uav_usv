# Read-only visibility / tracker / mission integration review

Reviewed current source methods in `trajectory_tracker_node.py`, `target_visibility.py`, `mission_manager.py`, `mission_manager_node.py` against requirements sections 4-11 and 13-16. No source edits, Gazebo runs, commits or subagents. Findings below concern current production control behavior, not old test fixtures.

**Latest re-review status: both findings below are resolved in current source.** The original findings and reproductions are retained as review history. Scoped re-review and current verification appear at the end of this report; no remaining critical issue was found in the requested normal/default visibility-control paths.

## Findings

### P1 — A fresh trajectory can be accepted after KF invalidation but before the next timer

Location: `src/uav_control/uav_control/control/trajectory_tracker_node.py:808`, especially cached `visibility_decision.locked` at lines 815-817; related invalidation callback at lines 698-706.

Concrete trigger:

1. Previous timer produced `visibility_decision.locked=True`.
2. A current KF callback is invalid; `target_state_callback` clears `latest_kf_message` and `latest_target_state`, but cached lock remains true.
3. Before the next timer, a fresh valid trajectory callback arrives with fresh matching prediction and tracking source.
4. `_evaluate_trajectory` checks only the cached lock, not current fresh KF authority, and calls `tracker.accept`. A terminal candidate also sets `terminal_mode_latched=True`.

Read-only reproduction used actual `_evaluate_trajectory`, `target_state_callback`, trajectory conversion and real `TrajectoryTrackerCore`, with valid polynomial/start/endpoint data. Result:

```text
INVALID_KF_ACCEPT_WINDOW {
  rejection: NONE,
  latest_kf: None,
  active_plan: 7,
  terminal_latched: True
}
```

The next timer will correctly preempt control, so this is a plan/mission authority violation rather than demonstrated blind PX4 descent. However, the accepted candidate diagnostic can transiently move mission into MINCO/terminal using cached lock; it violates the requirement to stop accepting terminal plans immediately when KF authority is unavailable. Pending-plan processing also runs before `_update_visibility` in the timer, making the same stale-lock gate relevant there.

Suggested fix: gate trajectory admission against current `fresh_flight_target(latest_kf_message, now, maximum_state_age, expected_frame_id)` and current bearing/lock authority rather than relying solely on a previously cached decision. Invalidate diagnostic lock on invalid KF callback or compute current authority before publishing acceptance diagnostics. Add callback-order regression: locked timer → invalid KF callback → fresh terminal candidate → must reject without latching terminal mode.

### P2 — Prediction-only terminal loss can hold below configured recovery clearance

Location: `src/uav_control/uav_control/control/trajectory_tracker_node.py:1163`, recovery condition at lines 1181-1188, prediction failure branch around line 1392.

Concrete trigger:

1. Terminal mode is active, and RGB + 3D + KF remain fresh, so visibility decision remains locked.
2. Prediction becomes missing/stale and the terminal command path invokes `_search_or_recovery_command`.
3. Configure `target_search_enable_height=1.5`, `recovery_clearance=2.0`, with actual height 1.7 m and zero vertical speed.
4. `_update_visibility` does not latch recovery because visual lock is still true. `_search_or_recovery_command` compares height only with search height, so it selects anchored hold at 1.7 m instead of the required 2.0 m climb.

Read-only reproduction result:

```text
PREDICTION_LOSS_CLEARANCE_BRACKET {
  locked: True,
  recovery_latched: False,
  search_state: TARGET_LOCK,
  position: (0.0, 0.0, -1.7),
  velocity: (0.0, 0.0, 0.0),
  requested_recovery_clearance: 2.0
}
```

This still stops descent. It violates the configured terminal recovery clearance and permits visual yaw/search authority before reaching that clearance. Default recovery clearance .5 m is below search height 1.5 m, so defaults mask this parameter-bracket case.

Suggested fix: missing/stale prediction in terminal context should latch the same recovery mode as KF/visual loss; hold/search should be released only above `max(search_height, recovery_clearance)` with the existing vertical settle condition. Add bracket test where recovery clearance is greater than search height, KF/visual lock stays fresh, and prediction alone expires.

## Reviewed paths without actionable findings

- Ordinary terminal visual/KF loss below sea clearance sets root recovery latch before command selection; final yaw is inhibited while latched. On measured safe height and settled vertical velocity, root clears latch and terminal mode, core receives `terminal=False`, releases its internal recovery latch, and mission can transition SAFE_RECOVERY → REACQUIRE → TARGET_LOCK → FAR_GUIDANCE when intercept was requested.
- Last visible image sign survives recovery because core is not reset merely on recovery state transitions. Mission-id change and clock reset intentionally clear history.
- Initial never-seen sweep is limited to one turn; reacquire scans alternate expanding signed endpoints within +/-pi. Wrapped heading feedback handles crossing +/-pi, finite stage count and time budget prevent unbounded turn commands even if yaw stalls.
- Core search is height gated and final yaw arbiter independently inhibits all yaw below search height. Search holds a fixed XYZ anchor rather than reanchoring every frame. Low-height recovery commands use `max(search_height, recovery_clearance)`.
- Distinct invalid image frames, not timer callbacks, drive loss debounce. One or two invalid frames preserve lock only while current KF remains within root maximum acquisition age .125; this independently limits stale continuation even though core silence debounce is .45 by default.
- RGB-only bearings drive centering but do not satisfy fresh 3D + KF lock gates. Three distinct 10 Hz frames can lock because receipt and interframe freshness are evaluated separately from total window span.
- Final `_final_yaw` is the command arbiter; visible bearing servo replaces trajectory yaw, and recovery inhibition outranks visible bearing. No target-truth direction is needed for these paths.

## Limits

This is a source/component review. Reproductions exercised current production methods with controlled boundary messages and real pure tracker core. No runtime topic graph, flight motion, sea contact, perception validity or Gazebo phases A-E were verified. Root is adding additional component tests and may update these locations while integrating the findings.

## Scoped re-review after root fixes

### P1 resolved — trajectory admission now checks current KF and immediate frame-loss authority

Verified actual current `_evaluate_trajectory` at `trajectory_tracker_node.py:817`. Its admission condition calls `fresh_flight_target(latest_kf_message, now, maximum_state_age, expected_frame_id)` directly, independent of the previously cached decision. Invalid, absent or stale KF therefore rejects the candidate before `tracker.accept` or terminal latch. The gate also directly checks current `consecutive_lost_frames >= target_loss_frames`, so three bearing-loss callbacks cannot exploit a prior timer's locked decision.

Current regression `test_invalid_kf_preempts_candidate_before_next_timer` follows locked timer → invalid KF callback → candidate evaluation and verifies PREDICTION_MISMATCH, no active trajectory, no terminal latch. It passes.

### P2 resolved — terminal prediction recovery respects the larger safety height

Verified `_search_or_recovery_command` at `trajectory_tracker_node.py:1191`. Terminal mission/latch now unconditionally requests the root recovery latch when this loss handler is entered. Its altitude condition and recovery command both use `max(search_height, recovery_clearance)`. The 1.7 m actual height / 1.5 m search / 2.0 m recovery bracket therefore commands upward motion and inhibits yaw instead of anchoring a lower hold.

Current regression `test_terminal_prediction_loss_climbs_to_larger_recovery_height` keeps visual/KF lock available and verifies recovery latch, negative NED vertical velocity, a rising position reference, and SAFE_RECOVERY. It passes.

### KF callback reordering verified

Verified `target_state_callback` at `trajectory_tracker_node.py:698`. A well-formed valid same-frame callback with older state stamp or older originating source stamp returns before the freshness conversion, so a delayed old valid packet cannot clear a newer fresh cache merely because the old source now exceeds maximum age. Invalid packets intentionally remain fail-closed and revoke the cache; the regression explicitly covers that behavior.

Current regression `test_stale_reordered_kf_does_not_revoke_new_valid_cache` passes.

### Current scoped verification

```text
cd /home/qin/data/uav_usv/src/uav_control
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv/install/setup.bash
python3 -m pytest -q test/test_strict_visual_control.py test/test_target_visibility.py
54 passed in 0.29s
```

This run also covers search XYZ anchor retention, low-height yaw inhibition, single yaw arbiter search/vision/recovery priorities, camera clock failure handling, and the 42 pure visibility cases.

No additional critical defect was found within the requested current normal/default visibility, mission recovery, search-direction and bounded-stale paths. Full runtime graph and flight acceptance remain outside this read-only review; passing component tests does not establish Gazebo phases A-E or sea-contact outcomes.
