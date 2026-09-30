# P8 strict visual integration review (read-only)

Formal assigned suite after interface generation: **100 passed** (four files, 0.39 s). Package-wide pytest intentionally not run during concurrent root edits. No Gazebo, commits, subagents, or production edits were made during this review.

## Remaining actionable finding

**P2 — stale out-of-order KF packet still revokes a newer valid tracker cache.**

`src/uav_control/uav_control/control/trajectory_tracker_node.py:698-710`: the newly added `fresh_flight_target` check runs before ordering. Example: cache contains stamp=10.10/source=10.00 at now=10.11; a delayed valid stamp=9.50/source=9.40 packet returns `converted=None` and clears the fresh cache at lines 703-706 before reaching the order guard. `_update_visibility` then interprets no KF as loss, potentially causing terminal recovery. Move normal valid same-frame epoch ordering before freshness rejection, while preserving immediate invalid/frame/timestamp-inconsistent revocation. Predictor and planner already use that order. Add a callback regression that exercises an older valid-but-stale packet (not just an older still-fresh packet).

## Findings corrected by root during the review

- Online image bridge was gated by `enable_shadow_perception`, so false disabled required RGB-D inputs while online bearing/localizer/KF remained running. Root removed this gate; the remaining shadow gates are diagnostic nodes/predictor only.
- `_final_yaw` previously ignored search-rate limits whenever RGB was visible but target not locked. Root now caps visible-but-unlocked servo with `maximum_search_yaw_rate`; locked tracking retains the existing observation envelope.
- Evaluator duplicate `truth_role` dictionary entries were removed while reviewing; no active issue remains.

## Confirmed architecture / behavior boundaries

- TargetPrediction source stamp remains KF evaluation epoch; immutable observation stamp remains acquisition time. Completed worker serialization uses its own request, and new live predictions cannot freshen an old worker. The 100-test suite includes stale-image/projected-KF, malformed/future/frame-invalid inputs, cache revocation, old-worker provenance, and all four search/recovery planner cycle invalidations.
- Main predictor is tracking-only and rejects configured or ROS-remapped truth topics. Online planner transport requires tracking provenance.
- Search/recovery states are outside planner PLANNING_STATES and inside RECOVERY_STATES; entering search cancels pending contact proposals and invalidates old async publication cycles. Returning through TARGET_LOCK to FAR_GUIDANCE resets contact scheduling.
- A single tracker-owned final arbiter writes yaw/yawspeed immediately before both publisher paths. Bearing/localizer/visibility code does not publish PX4 setpoints. Search-height and safe-recovery inhibits prevent sea-level yaw search; XYZ search is anchored rather than tracking measured drift.
- KF freshness in tracker and predictor uses original image stamps, independently of the projected state epoch. Image-bearing and 3D observation gates remain distinct; RGB bearing can steer without valid depth but cannot alone establish 3D/KF lock.
- Default baseline launch forwards to the process-isolated graph with one active PX4 command owner. Legacy truth controllers no longer have console entry points and their main functions are explicitly disabled.
- `enable_evaluator:=false` structurally removes only the evaluator. Mission has no evaluator result subscription; moving target no longer freezes/pauses on evaluator events; evaluator no longer pauses the world. No online dependency on evaluator completion was found. Without evaluator, truth-based success reporting is naturally unavailable, while acquisition/control/recovery still operate.

## Scope limits

This is source and unit-test verification. Autonomous Phase A-E search, successful complete visual interception, visual-valid rate / POSITION_TIMESTAMP_AFTER_HISTORY, XY drift, and water-contact safety have not been verified in this review. Default moving-target motion/range versus lock/search timing needs actual authorized staged simulation; no parameter tuning is justified by this static review.

## Final integration closure by primary agent

The remaining stale-reordered KF finding is resolved in current source: valid same-frame older epochs return before freshness revocation; invalid input still clears cache. Regression test_stale_reordered_kf_does_not_revoke_new_valid_cache passes in the final package suite (626 passed, 1 skipped). Current source also rejects source_stamp after state stamp and configured/remapped non-KF target input. The package-wide final log is package_tests.log. No Gazebo was started.
