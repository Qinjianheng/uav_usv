# FOLLOW / run artifacts / SAFE_WAIT / visual attribution plan

Baseline: HEAD 057775a (9.30.1), initially clean. User supplied full execution specification; implement in place, no rollback, commit, Gazebo or P8.1 production edits.

## Confirmed causes
- FOLLOW controller and tracker branch remain, but mission tick never enters FOLLOW and Y only accepts TARGET_LOCK.
- Visibility computes search yaw then changes only the status on timeout; final arbiter does not inhibit invisible SAFE_WAIT.
- Evaluator writer and begin() share _start_mission, called only after intercept_requested; timer requires evaluator started and truth.
- Latest historical 0.57m bias log lacks independent heading/geometry, so no physical fix justified yet.

## Interfaces and tasks
1. TDD mission/search: TARGET_LOCK tick -> FOLLOW when no Y request; FOLLOW+locked accepts Y; relock returns FOLLOW or FAR by intercept flag. Preserve one published lock event. SAFE_WAIT timeout/completion gives zero rate/direction; final arbiter HOLD on invisible wait. Reuse existing fresh KF guidance/RGB yaw and unmodified follow parameters. Scoped worker owns mission files, visibility, tracker changes and their tests.
2. TDD logging: _start_run_artifacts(mission_id,now) starts on accepted TAKEOFF; _start_intercept_evaluation(mission_id,now) begins only on Y edge. Run timestamps/counters independent of evaluator.started_at. Timer emits rows without truth/intercept; missing geometry explicitly unavailable. _finalize_run(reason,now,result=None) idempotent, handles reset/completion/shutdown and preserves artifacts before next mission. Pure RunMetricAccumulator summarizes phase durations, lock/reacquire events, FOLLOW estimated/evaluation distances and search XY drift.
3. TDD offline attribution: dedicated geometry YAML (baseline unchanged), independent evaluator-only UAV model pose/heading tracker with causal clock mapping, quaternion interpolation and explicit query provenance; no controller publisher. Extend existing offline analyzer with stage residuals and limitations, no fitting or correction. Read existing logs and produce available evidence, without Gazebo. Scoped worker owns new diagnostic helper, analyzer/tests/YAML and audit artifacts; primary integrates its API into evaluator logging.
4. Integrate tests for accepted X writer, lock transition rows, X-only abort/reset, Y-only timer, maximum duration excluding pre-Y, no-truth logging, idempotent finalization, and diagnostics isolation. Update launch command hints to FOLLOW.
5. Run package and requested root pytest, requested flake8 and diff checks; characterize any preexisting generated-file/legacy style failures without hiding them. Build interfaces/control/bringup. Dedicated review, final report 18 requested topics with user-run Phases A-D, no simulation claims.

## Review focus
- No rows, pending vision or summary lost on Y/reset; writer never finalized twice.
- Logging without truth/UAV/intercept; phase transition events retained even below sample rate.
- Evaluator maximum_duration starts at accepted Y only; result stays record-only.
- Wrong mission and late diagnostic do not contaminate run counters or control lock.
- Geometry reference conditioned on PX4 pose cannot by itself prove pixel/extrinsic bias; heading source must be UAV model/independent reference, not USV bearing.
- P8.1, KF Q/R, follow parameters, MINCO/BCTRA and baseline geometry flags remain unchanged.

## Completion evidence
- Steps 1-4 implemented and covered by current tests; no simulation executed.
- Root pytest: 670 passed, 1 skipped. Requested bare flake8, diff check and shell syntax passed.
- P8.1/localizer/bearing focused regressions: 99 passed. Preserved source bytes and dedicated YAML four-difference isolation checked against HEAD.
- Latest three-package colcon build exited 0: 3 packages finished in 2min31s.
- Independent review reproduced and closed four P2 findings (stale diagnostic statistics, old mission reopening, missing right reference bracket, old worker planner events); main/shadow prediction ownership additionally covered.
- Final 18-topic report: docs/tracking/follow_run_logging_20260930.md; raw validation and review: docs/tracking/follow_run_evidence_20260930/.
- Current 0.566m visual bias root cause remains unconfirmed: newest geometry/independent heading unavailable. Historical attribution recomputed, evaluator-only tools delivered. Production perception unmodified; user-run Phases A-D remain required.
