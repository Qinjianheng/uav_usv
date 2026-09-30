# Strict visual loop implementation plan

> For agentic workers: use executing-plans or independent scoped workers; keep the shared workspace changes reviewable and uncommitted.

**Goal:** RGB-D → KF → BCTRA → MINCO → tracker → PX4 without target truth or evaluator feedback, including safe visual search and reacquisition.

**Architecture:** Tracker owns the only PX4 command/yaw arbiter. A pure visibility/search core consumes image bearing and acquisition freshness. Mission consumes controller diagnostics, never evaluator results. Two-dimensional bearing precedes depth and pose gates. Immutable measurement provenance propagates through prediction and plan.

**Spec:** ../specs/2026-09-30-strict-visual-loop-requirements.md

## Global constraints
- Preserve P8.1 production mapping, bounded causal wait, pose history and timeout values.
- No truth fallback, Q/R tuning, yaw/XY correction, capture-radius change or MINCO weight changes.
- Native current workspace requested by the user; no branch changes, automatic commits or edits under data/videos.
- Audit artifacts already contain 102 literal matches, 586 expanded matches and the subscription inventory.
- Keep state evaluation time separate from image acquisition time; trajectory source_stamp remains execution start.

## Review focus
- Old valid prediction followed by invalid input must not remain trusted indefinitely.
- Repeated/late image frames and total stream silence must not create false locks.
- Terminal loss must preempt descent, including command acceleration/braking continuity.
- Evaluator result/hit and world pause cannot affect mission or moving target.
- RGB-only visibility must permit yaw centering but never a 3-D target lock.

### Task 1: Pure visibility/search core
- [x] Tests first: left/right loss, initial sweep, loss debounce, stable lock, deadband, angular wrap, duplicate frames, silence/reset, bounds.
- [x] Implement mission/target_visibility.py with explicit scan and lock state, no ROS or truth inputs.
- [x] Verify unit tests and document interface.

### Task 2: Image-only bearing
- [x] Tests first: image-right→positive NED yaw, image-left→negative yaw, depth invalid, camera pitch and acquisition stamps.
- [x] Add TargetBearing.msg and a separate lightweight RGB bearing node reusing existing detector and clock mapper, no 3-D localizer duplication.
- [x] Register node/interface; no truth or pose subscriptions; run targeted tests.

### Task 3: Acquisition-time provenance and strict prediction
- [x] Tests first: projected KF stamp cannot renew measurement freshness; reject invalid/future/stale inputs; immutable trajectory observation_stamp.
- [x] Update prediction engine/node and planner to carry measurement stamp separately from sample epoch/execution start.
- [x] Main predictor tracking-only; no simulation_truth branch. Reacquisition invalidates planner requests/contact state.
- [x] Keep P8.1 production files untouched; run targeted tests.

### Task 4: Tracker/mission integration and truth isolation
- [x] Tests first: yaw ownership, XYZ anchor hold, search-height gate, KF lock, terminal recovery, evaluator independence, graph subscription policy.
- [x] Tracker target state becomes timestamped KF; endpoint uses fresh BCTRA. Integrate core/bearing, single final yaw arbiter, stage-specific loss handling.
- [x] Append acquire/lock/reacquire/recovery states without renumbering existing contracts; mission uses control diagnostics only.
- [x] Strict launch requires perception regardless of shadow diagnostics; optional evaluator. Isolate legacy truth-control executables and launch.
- [x] Evaluator evaluation_only: no world pause, no feedback to mission or target generator. Truth-angle diagnostics remain marked shadow-only.
- [x] Add YAML safety initial values verbatim.

### Task 5: Verification and report
- [x] Update old P8.1 test fixtures for current timesync fields without changing mapping; fresh baseline found 110 passed/7 fixture failures.
- [x] Build ROS interfaces/control/bringup; package tests, style and diff checks.
- [ ] User-run Phases A/B/C/D/E: deferred per explicit user request to avoid Gazebo. Ordered instructions and honest unverified boundaries are delivered in the final report.
- [x] Deliver truth audit, state diagram, ownership, parameters, tests and remaining limits.

## Decisions
- Continue in place after explicit user '继续修改'; treat this as authorization to execute, no repeated design approval.
- Search loss counts unique acquisition frames; silence uses acquisition-age expiration rather than callback/timer counts.
- Lock requires fresh RGB bearing and fresh position measurement/KF; RGB-only cannot authorize translation/intercept.
- Reacquire timeout 2 s moves to SAFE_WAIT (hold); bounded scan may continue at safe altitude until a complete bounded scan finishes, then yaw holds.
- Terminal recovery must climb to max(recovery_clearance, target_search_enable_height) before any yaw scan.
- Short loss continuation is allowed only inside the unchanged strict acquisition age limits; freshness wins over frame debounce.

## Final software verification
- 626 package tests passed, 1 skipped; interface/control/bringup build passed.
- Offline-only P8.5 analyzer and image-time bearing/heading log fields added; no online bias correction.
- Report: docs/tracking/strict_visual_loop_p8_2_p8_3_20260930.md. No Gazebo, commits, pushes or memory edits.
