# Controls upstream-port analysis

This is a portability map for bringing improvements from upstream
[commaai/openpilot](https://github.com/commaai/openpilot) into flowpilot's
longitudinal/lateral control stack (`selfdrive/controls/lib/`).

## Context

flowpilot is a **hard fork** of openpilot pinned at commit `05b37552`
("Retuned desire model", openpilot ~0.8.x, late 2021). It has since diverged
heavily: much of the UI and modeld are rewritten in Java, and several control
files are intentionally customized. A blanket upstream merge is **not feasible**
— current upstream depends on a new model output contract (`desiredCurvature`
directly from the model), the spun-out `opendbc.car` package, new
`CarInterface` methods (`lateral_accel_from_torque`), a breakpoint-table
`PIDController`, and a `lat_delay`/`curvature_limited` planner contract that this
fork does not have.

Ports must therefore be **selective** and fit the existing 0.8.x-era signatures.

## Status legend

- ✅ **Ported** — done, low-risk, fits existing infra.
- 🟡 **Candidate** — portable but needs on-vehicle testing / a decision.
- 🔴 **Do not port** — flowpilot deviates intentionally, or the upstream version
  requires the new model/planner/opendbc contract.

## File-by-file

### `latcontrol_torque.py`

- ✅ **Road-roll + steering-angle-offset compensation.** flowpilot passed
  `roll = 0.0` and ignored `params.angleOffsetDeg` when computing measured
  curvature, and used the raw desired lateral accel as feedforward. The PID and
  INDI controllers in this same tree already use `params.roll` /
  `params.angleOffsetDeg`; the torque controller was the lone holdout. Ported so
  measured curvature uses `(steeringAngleDeg - angleOffsetDeg)` and the estimated
  roll, and the feedforward is gravity-adjusted
  (`desired_lateral_accel - roll * ACCELERATION_DUE_TO_GRAVITY`). No-op when roll
  is 0 / offset ≈ 0. (See the roll/angle-offset PR.)
- 🟡 **Latent divide-by-zero in the non-steering-angle branch.**
  `actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo`
  divides by `vEgo`. For cars with `useSteeringAngle == False`, near standstill
  this can produce `inf`/`nan` in the curvature estimate. Only reachable if
  lateral control stays active at ~0 speed (depends on flowpilot's `latActive`
  gating). Worth a guarded fix (`max(vEgo, MIN_SPEED)`) — but verify it doesn't
  affect intentional low-speed lane centering.
- 🔴 **Full upstream rewrite** (lateral-accel-space error correction, jerk
  lookahead, `lat_delay` compensation, `CI.lateral_accel_from_torque()`).
  Requires the new opendbc/CarInterface/planner contract. Not portable without a
  much larger migration.

### `latcontrol.py`

- `_check_saturation` is byte-for-byte identical to upstream — nothing to port.
- The newer upstream `_check_saturation(..., curvature_limited)` parameter
  requires the lateral planner to emit `curvature_limited`, which this fork's
  planner does not. 🔴

### `latcontrol_pid.py` / `latcontrol_indi.py`

- Already use `params.roll` and `params.angleOffsetDeg`. No outstanding port.

### `longcontrol.py`

- 🔴 **Intentionally gutted.** `long_control_state_trans` short-circuits to
  `return LongCtrlState.pid` (the upstream state machine below it is dead code),
  and `update()` forces `v_target`/`a_target`/`v_target_1sec` to `0.0`. flowpilot
  deliberately delegates the longitudinal target/accel elsewhere. Porting
  upstream's stopping/starting state machine would break this design.

### `drive_helpers.py`

- 🔴 **`VCruiseHelper` cruise-speed logic is bespoke.** `update_v_cruise` /
  `_update_v_cruise_non_pcm` are rewritten with flowpilot's mph-based scheme
  (hardcoded 26 mph base, +4/+2 increments, 70→80 mph caps). Upstream's version
  would overwrite intentional behavior.
- 🔴 **`get_lag_adjusted_curvature`** matches 0.8.x. Upstream later removed it
  entirely once the model emits `desiredCurvature` directly; "updating" it means
  adopting the new model/planner contract.

## Recommendation

The roll/angle-offset compensation is the one clean, self-contained controls win
and is captured in its own PR. Remaining control-loop changes are either
intentional flowpilot customizations (do not port) or require the post-0.8.x
model/planner/opendbc contract (large migration). For incremental, lower-risk
value, prefer **additive opendbc car ports** (new fingerprints/DBC/tuning) over
mutating the tuned, safety-critical control loops, and validate any control-loop
change on-vehicle before merging.
