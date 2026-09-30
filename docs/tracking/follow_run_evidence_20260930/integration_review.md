# Integration review — FOLLOW / run lifecycle / offline heading

Baseline HEAD 057775a. Reviewed actual uncommitted source, new helper modules, tests, dedicated YAML, full user spec and implementation plan. Read-only review; temporary fixture artifacts lived only in /tmp; no repository edits, Gazebo, commits or flight runs.

## Findings (initial snapshot, implementer may already be repairing)

### P2 — Stale controller samples contaminate FOLLOW distance and CSV state
File: src/uav_control/uav_control/evaluation/intercept_evaluator_node.py:1166-1178,1755-1776.
`latest_controller` is assigned before the freshness gate; the gate only protects lock/visible accumulator events. `_write_run_sample` nevertheless consumes this cached message's target_distance every timer cycle and publishes its stale locked/visible/yaw fields. Even a fresh initial callback becomes stale and is repeated indefinitely after the publisher stops.
Minimal verified repro: begin mission1 TAKEOFF at10, FOLLOW at11; deliver mission1 diagnostic stamp9, target_distance99, visible/lockedtrue at11; timer at12. Printed FOLLOW estimate statistic count1/mean99 even though diagnostic age3s. Counters ignored the callback, but distance did not. Reject stale callback before caching and check freshness again when sampling (or explicitly mark stale CSV provenance/unavailable metrics).

### P2 — Older TAKEOFF can start a spurious run after a newer run has finalized
File: src/uav_control/uav_control/evaluation/intercept_evaluator_node.py:1360-1367.
Older mission filtering is conditional on an active writer. After finalization a late old TAKEOFF satisfies mission_id != run_mission_id and creates a new writer for that historical mission. This violates run ownership and can reset accumulators/history in an already-finished session.
Minimal verified repro: mission1 TAKEOFF10/finalize12; mission2 TAKEOFF13/finalize14; receive mission1 TAKEOFF at15. Printed writer=True and run_mission_id=1. Track latest seen mission independently of writer existence and reject older IDs; retain same mission's completed/run-consumed ownership.

### P2 — Vision events are discarded before the independent heading bracket arrives
File: src/uav_control/uav_control/evaluation/intercept_evaluator_node.py:625-632,763-767.
Pending vision waits only for truth. When truth is already bracketed but the independently scheduled Gazebo UAV pose callback is still waiting for its right pose or clock anchor, `_drain_visual_observations` immediately logs REFERENCE_AFTER_HISTORY and removes the observation. No retry uses the bracket that arrives milliseconds later. Under consistent callback ordering this can leave the dedicated heading experiment with zero matched reference samples despite a populated reference history.
Minimal verified repro: clocks raw1→ROS10, raw1.1→ROS10.1; left UAV pose at10; truth sample and valid geometry observation at10.05. Deliver observation first: pending queue becomes0. Deliver right UAV pose at10.1 and drain again: query10.05 is INTERPOLATED but CSV remains REFERENCE_AFTER_HISTORY. When enabled, wait boundedly for required independent heading/entity bracket; preserve original acquisition stamps, force finalization with explicit missing status after timeout.

## Confirmed software behaviors
- Scoped tests: `source /opt/ros/humble/setup.bash; source install/setup.bash; python3 -m pytest -q src/uav_control/test/test_run_logging.py src/uav_control/test/test_follow_wait.py src/uav_control/test/test_uav_heading_diagnostics.py src/uav_control/test/test_p8_visual_bias_analysis.py` → 52 passed in0.39s.
- Mission restoration uses transient TARGET_LOCK then FOLLOW; relock preserves intercept intent; Y requires lock and FOLLOW/TARGET_LOCK.
- Invisible SAFE_WAIT returns rate0/direction0; final yaw arbiter holds even with inconsistent scan decision.
- New logging creates TAKEOFF/phase rows without truth, flushes pending vision and finalizes X-only as ABORTED; Y begins separate evaluator timer and keeps run writer.
- Current evaluator result remains record-only; manager has no result subscription; new Gazebo UAV helper has no online publisher; exact UAV model matching rejects nested links/wrong models.
- Dedicated YAML copies baseline and changes only geometry/evaluator diagnostics flags; production RGB-D, P8.1, KF Q/R, planner/controller gains and baseline YAML are unchanged in this diff.
- Heading source is independent UAV model ENU/FLU rotation converted to NED/FRD with exact/interpolated acquisition brackets; model-origin/PX4 local-position equivalence explicitly remains unestablished. Analyzer declines root-cause proof and fixed correction.

## Declined-to-judge behaviors and reasons
- Real FOLLOW distance near5m, target-relative motion, lock stability, reacquisition duration and SAFE_WAIT observed PX4 flight response: no Gazebo or flight authorized in this review; software tests cannot certify dynamics.
- Current visual0.57m RMSE root cause, pixel/depth/extrinsic errors versus independent PX4 heading bias: historical geometry/reference coverage is incomplete and no new dedicated experiment was run. Pose-conditioned residuals alone do not prove a physical error.
- Current runtime POSITION_TIMESTAMP_AFTER_HISTORY≈0 and95.9% validity: no runtime recording; production P8.1 code unchanged is regression-scope evidence, not a new live rate measurement.
- Runtime DDS truth/command subscriber inventory: source boundary checked, runtime topic graph not active; must perform user's PhaseA audit.
- Gazebo model origin equality with PX4 origin or physical camera link: new code explicitly flags unknown equivalence; no physical frame evidence sufficient to assume equality.
- Full root pytest, flake8, colcon build: primary implementer owns current full checks; scoped review output above is independently verified only.

Assessment: Software-level implementation substantially covers the requested behaviors; address three reproducible logging/diagnostic lifecycle findings before software completion claim. Flight-level and physical visual root-cause validation remain unverified and require user-run PhasesA–D.

## Closure check after implementer fixes

Read current source and ran only `test_run_logging.py`: 11 passed in0.29s. All original three P2 findings are closed in software:
- Controller callback rejects age/reorder before metrics; `_fresh_controller(now)` rechecks cached age when writing CSV and FOLLOW distance.
- Mission ID guard persists while writer is None, comparing run and latest-seen mission IDs.
- Geometry-enabled rows wait at most0.15s for EMPTY/AFTER_HISTORY/QUERY_GAP evaluator pose queries; exact/interpolated acquisition brackets are preserved and force flush remains explicit unavailable.

### Additional P2 — Late planner event from a previous mission contaminates a new run
File: src/uav_control/uav_control/evaluation/intercept_evaluator_node.py:1100-1123.
`planner_callback` still assigns latest_planner_diagnostic, counts PlannerEventAccumulator and writes detail before checking run ownership. A delayed asynchronous planner completion after R/new X is then counted in the new run, unlike controller diagnostics which now enforce ownership.
Minimal verified ephemeral repro: mission2 TAKEOFF10; deliver PlannerDiagnostic mission1,plan123,RESULT_SUCCESS; `_sample_row(10.1)['planner_event_id']` prints `1:123`; `event_metrics.summary(1.)` gives planner_started1/planner_succeeded1. Add an active-run mission_id guard before assigning/counting. Do not apply controller's0.125s decision freshness to legitimate same-mission historical planner completions: completed event accounting may intentionally be retrospective.

Updated assessment: Original findings closed; one additional planner ownership issue remains until implementer closes it. No new flight or physical-error evidence was produced.

## Final scoped closure

Confirmed last P2 planner ownership issue closed: active-run wrong-mission planner diagnostics are rejected before caching, counting, or writing details. Main/shadow prediction callbacks enforce the same active-run ownership; _start_run_artifacts clears latest_prediction.

Independent scoped run: test_run_logging.py => 15 passed in0.29s. Additional read-only ephemeral check: current mission2 atROS12 accepts mission2 planner completion generated atROS10, preserving CSV planner_event_id 2:123 and planner_succeeded1. Thus legitimate same-mission late completions remain countable; no controller-style0.125s freshness gate was applied to planner event accounting.

Final assessment: All four reported P2 issues are closed with source checks and scoped regression evidence. No remaining finding in the requested closure scope. Software validation and flight/physical root-cause validation remain distinct; the previously listed declined-to-judge runtime behaviors still require user-run experiments. No repository modifications, full-suite expansion, Gazebo or commits performed by this reviewer.
