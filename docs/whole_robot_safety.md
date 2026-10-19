# Whole-robot stability envelope: baseline specification

## Objective and scope

Learn a finite-horizon safety value function with a PINN for coordinated mobile-base,
lift, and seven-joint arm control. Its nonnegative level set will estimate the states
from which an admissible coordinated policy can maintain the chosen stability
requirement. The current deliverable measures the physical signals and defines the
problem; it does not yet train a PINN or compute a viability envelope.

Baseline assumptions: flat, stationary horizontal floor at world z=0; existing
MuJoCo masses, inertias, friction and actuators; no additional payload or external
push. The simulator is a model with estimated parameters, not hardware ground truth.
All seven arm joints remain represented. Initial scenarios vary multiple joints,
base driving/turning/strafing, and lift height/motion.

## Physical failure requirement

Use a configurable **no wheel unloading / bounded base tilt** requirement first:

    g_load = (min_i Fz_i - F_safe) / F_scale
    g_tilt = (tilt_max - tilt) / tilt_max
    g_observed = min(g_load, g_tilt)

`Fz_i` is the sum of upward world-frame ground forces on all rollers belonging to
wheel i. `tilt` is the angle between the base's local vertical and world vertical.
Positive scales make the margins dimensionless. A negative value violates the
requirement; equality is its boundary. This is stricter than simply avoiding
overturning: a wheel can unload without the robot falling.

The initial configuration uses F_safe=5 N, F_scale=100 N, tilt_max=10 degrees.
These are explicit experimental settings, **not validated physical limits**.
Threshold sensitivity and practical requirements must be established before use.

Contact loads are outputs of the constrained dynamics and can depend on commands,
disturbances, and contact mode. Thus `g_observed` is a measurement, not an already
established state-only HJ terminal function. Before constructing the HJ obstacle,
either establish a valid state-only margin under the modeled controller/contact
description, or formulate input-dependent constraints explicitly. Do not silently
substitute this measurement into a state-only PDE.

Additional diagnostics:

- Total floor force F and moment M about the world origin, including nonwheel
  floor contacts. The force convention is floor acting on robot.
- Contact CoP on z=0: x=-My/Fz, y=Mx/Fz when Fz>0. This is an observed resultant,
  not a predicted unconstrained ZMP. It is null without upward resultant force.
- Convex hull of loaded wheel contact positions and signed inward CoP edge margin.
  The margin is null for fewer than three noncollinear points. Being inside this
  active-contact hull is not independent proof against tipping: the measured CoP
  is already constrained by the forces of the contacts that remain.
- Nonwheel floor contacts, whole-robot CoM, base roll/pitch/tilt and actuator forces.
  Nonwheel contact is reported separately; it is not presently an additional term
  in g. Sliding/friction failure and self-collision are outside this first requirement.

## State and control specification

The baseline XML compiles to nq=74, nv=73 and nu=12. These include the free base,
passive rollers, and lift loop coordinates, so they are not 73 independent
unconstrained mechanical degrees of freedom.

The initial full representation is `(qpos, qvel, act, controller_state)`:

| Component | Recorded state |
| --- | --- |
| Free base | World position, quaternion, translational/angular velocity |
| Lift and closed linkage | All modeled generalized positions/velocities |
| Arm | All seven joint positions and velocities |
| Wheel hubs and rollers | All generalized positions and velocities |
| Arm controller | Targets, position references, reference velocities |
| Lift controller | Target and ramped position reference |
| Base command generator | Target and ramped forward/lateral/yaw references |

Quaternion coordinates lie on a manifold. Linkage states must satisfy loop closure;
arbitrary uniform sampling of raw coordinates is not valid. Derive a consistent
parameterization before PINN collocation. Future restart/replay tooling should also
capture MuJoCo integration state (including solver warm starts); the JSONL records
are observations, not claimed bit-exact restart checkpoints.

The baseline control interface has 11 high-level requests:

    u = [v_forward_target, v_lateral_target, yaw_rate_target,
         height_target, arm_position_target_1, ..., arm_position_target_7]

The existing mixer produces four wheel-speed references, giving 12 actuator inputs
including lift and arm. The runner uses the keyboard/CLI mixer path, not independent
wheel-slider commands. The target domain for the first study is bounded by forward
and lateral speed 0.30 m/s, yaw speed 0.25 rad/s, catalog lift travel and finite arm
joint limits. Bounds for continuous-joint target coordinates must be explicitly
chosen for PINN sampling. Scenario motion names currently exercise one base target
axis at a time; simultaneous translation/rotation is a later scenario extension.

Existing controller behavior is retained:

- Base translational references ramp at 0.15 m/s²; yaw reference at 0.4 rad/s².
- Wheel velocity servos: reference range ±6 rad/s and torque cap ±12 Nm.
- Lift reference: 0.265–1.270 m above lift base, ramped at 0.016 m/s.
  Lift actuator uses platform position with a ±2000 N force cap.
- Arm position servos: torque caps ±39 Nm for joints 1–4 and ±9 Nm for 5–7;
  trajectory references have the speed/acceleration limits in `joint_control.py`.

Those ramps limit references, not actual physical acceleration or joint speed.
The full command domain must also respect the coupled wheel-reference limits of
the mixer. Do not assume independently achievable CoM/base/lift accelerations.

## Reduction and reachability decisions

Before reducing state, check whether full states with the same proposed reduced
description have materially different evolution or available control authority.
CoM/ZMP alone can hide joint limits, inertia, angular momentum and actuator saturation.
Either retain necessary configuration information or conservatively bound omitted
effects. Do not commit to a two-dimensional robot model just for convenience.

Choose the horizon explicitly; start nominal without adversarial disturbances.
For a valid state-only margin g and state dynamics f, use

    V(x,t) = sup_policy min_{s in [t,T]} g(x(s))
    min(V_t + max_{u in U(x)} grad(V) dot f(x,u), g(x)-V(x,t)) = 0
    V(x,T) = g(x)

This defines finite-horizon viability, not indefinite safety. The PINN loss must
include the variational inequality and terminal condition. The induced controller
must use the same admissible commands and controller dynamics as the model.
For robustness, later specify bounded disturbances, their information structure,
and a realizable control policy; payload/friction uncertainty need not be arbitrary
time-varying adversarial inputs. A finite-horizon safety filter must account for
V_t as well as the state gradient.

## Running and interpreting experiments

From the repository root:

```sh
.venv/bin/python scripts/run_stability_scenarios.py --output outputs/stability/baseline-001
.venv/bin/python scripts/check_stability.py
```

Use the activated environment's `python` on other platforms. `--config` accepts a
JSON scenario file; default is `scenarios/stability_baseline.json`. Output directories
must be new so previous experiments cannot be overwritten accidentally.

Each scenario starts from new MuJoCo data and a consistently initialized linkage.
Arm initialization sets joint positions/references as an initial condition; later
arm/lift targets pass through the existing rate-limited controllers. Settling is
logged separately and excluded from maneuver outcome summaries. Segment targets
persist until replaced. Every physics step is checked; detailed snapshots are
written at approximately 50 Hz and at segment ends. Requested durations are rounded
to the nearest physics step. No viewer is required.

`manifest.json` records configuration, MuJoCo version, coordinate layout, actuator
ordering, source XML/Python hashes, actual sampling rate and per-scenario summaries.
Mesh files and installed libraries are not exhaustively hashed. JSONL files contain
full coordinates, applied controls, controller references, forces and metrics.
Moments and CoP use the world frame; wheel ordering is FL, FR, BL, BR. The separately
recorded MuJoCo body-object spatial velocity uses world axes, with angular
components followed by linear components. Raw qvel retains MuJoCo conventions.
Time starts at
initialization, so maneuver violation times include the preceding settling period.
The first snapshot is after the first physics step.

An observed violation means the **scripted policy** violated the specified margin.
It does not establish that every possible policy would fail. Likewise, an observed
safe maneuver is not a viability label. Use the eventual PINN-induced policy for
validation, separate training and held-out scenarios, and report boundary errors
and safety violations without implying formal guarantees from rollout counts alone.

Next deliverables: select validated thresholds/horizon, characterize the coupled
state/control domain and reduction error, then implement the neural HJ solver.

## Initial baseline results (2026-10-15)

Run: `outputs/stability/baseline-002/manifest.json`, MuJoCo 3.14.0, 1 ms physics
step. Five resettable scenarios covered 31 seconds including settling. The summary
examines every maneuver physics step; JSONL snapshots are approximately 50 Hz.

| Scenario | Lowest wheel load (N) | Maximum base tilt (degrees) | Observed requirement |
| --- | ---: | ---: | --- |
| Stationary, mid height | 112.33 | 0.0062 | Satisfied |
| Forward/brake, high lift | 45.97 | 0.0522 | Satisfied |
| Strafe/reverse, mid height | 65.34 | 0.0279 | Satisfied |
| Coordinated lowering and turning | 57.92 | 0.0545 | Satisfied |
| Coordinated raising and driving | 65.06 | 0.0309 | Satisfied |

There were no nonwheel floor contacts or MuJoCo warnings. These scenarios establish
a supported baseline, not the envelope boundary. Targeted configuration/motion
sweeps are still needed to approach and bracket margin violations.

`check_stability.py` passed force/weight balance, CoP translation, analytical hull
geometry, airborne support-loss detection, deterministic repeated logs, controller
reference behavior and malformed-target rejection. Existing `check_motion_limits.py`
also passed actuator saturation, lift ramp, arm reference and coupled-motion checks.
