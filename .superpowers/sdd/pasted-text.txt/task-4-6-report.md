# Tasks 4–6 implementation report

## Implementation commit

`7cb88700f6c2c115ef1c58b05cfac23ce38dc96c` — `Bound terminal contact rescheduling`

## Modified files

- `src/uav_control/uav_control/guidance/fast_minco_planner.py`
- `src/uav_control/uav_control/guidance/intercept_planner_node.py`
- `src/uav_control/uav_control/guidance/planner_pipeline.py`
- `src/uav_usv_bringup/config/baseline.yaml`
- `src/uav_control/test/test_fast_minco_planner.py`
- `src/uav_control/test/test_intercept_planner_messages.py`
- `src/uav_control/test/test_modular_configuration.py`
- `src/uav_control/test/test_planner_pipeline.py`

## TDD evidence

RED command:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && \
  cd src/uav_control && python3 -m pytest \
  test/test_planner_pipeline.py test/test_fast_minco_planner.py \
  test/test_intercept_planner_messages.py test/test_modular_configuration.py -q
```

RED result: `4 failed, 36 passed`. The four expected failures showed the
missing schedule bound constructor argument, missing planner max-duration
override, missing terminal bounded-search argument at the node boundary, and
missing YAML parameter.

GREEN command: the same focused command above.

GREEN result: `40 passed in 0.57s`.

Necessary regression command:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && \
  cd src/uav_control && python3 -m pytest test -q
```

Regression result: `275 passed, 1 skipped, 2 warnings in 4.23s`.

## Design and boundaries

- `terminal_max_reschedule_delay` is `0.30` in the node default and baseline
  YAML.
- A terminal request with an existing contact searches only from its locked
  remaining duration through at most `remaining + 0.30 s`; the planner's
  normal maximum duration remains the upper cap.
- The Fast MINCO max-duration override temporarily constrains both nominal and
  absolute maximum duration, so the reachability search cannot select a late
  rolling-horizon plan.
- First successful plans set `ContactTimeSchedule.contact_stamp`. Later
  updates require an explicit terminal reschedule and exactly
  `0 < new_contact - old_contact <= 0.30`; unchanged, early, and large-late
  candidates retain the old contact.
- A bounded planner failure keeps the existing success-only publication path:
  no trajectory is published and the schedule is not mutated, so the tracker
  continues its last valid MINCO trajectory until existing recovery applies.
- No planning rates, freeze time, safety guard, tracker gains, PX4 limits,
  capture radii, BCTRA, diagnostics, or Gazebo behavior were changed.

## Self-review

The focused tests cover the new config, schedule acceptance/rejection rules,
the real bounded MINCO outcome that rejects a late otherwise-feasible plan,
and the node's terminal max-duration request. The full suite includes flake8
and passed. `git diff --cached --check` passed before the implementation
commit.
