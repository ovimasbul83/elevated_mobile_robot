"""Regress large slider jumps, reversals, bounds, torque, and lift travel time."""
from pathlib import Path
import numpy as np
import mujoco
from build_scissor_lift import MIN_HEIGHT, STROKE, NO_LOAD_SPEED
from joint_control import POSITION_LIMITS, SPEED_LIMITS, ACCELERATION_LIMITS, TORQUE_LIMITS
from lift_kinematics import set_lift_height
from viewer_control import ViewerControl

model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'models/scene.xml'))
data = mujoco.MjData(model)
set_lift_height(model, data, MIN_HEIGHT)
control = ViewerControl(model, data, MIN_HEIGHT)
dt = model.opt.timestep


def panel(lift=None, arm=None):
    applied = data.ctrl.copy()
    shown = control.before_sync(data)
    assert shown[control.lift_id] == control.lift_target
    assert np.array_equal(shown[control.arm.ids], control.arm.target)
    if lift is not None:
        data.ctrl[control.lift_id] = lift
    if arm is not None:
        data.ctrl[control.arm.ids] = arm
    control.after_sync(shown, data)
    assert np.array_equal(data.ctrl, applied), 'A raw slider target leaked into physics'


for i, limit in enumerate(POSITION_LIMITS, 1):
    joint, actuator = model.joint(f'joint_{i}'), model.actuator(f'joint_{i}_actuator')
    assert bool(joint.limited[0]) == np.isfinite(limit)
    if np.isfinite(limit):
        assert np.allclose(joint.range, [-limit, limit])
        assert np.allclose(actuator.ctrlrange, joint.range)
    assert actuator.forcelimited[0]
    assert np.allclose(actuator.forcerange, [-TORQUE_LIMITS[i-1], TORQUE_LIMITS[i-1]])

# Force saturation must hold even if a caller bypasses the trajectory controller.
# Evaluate excessive position errors without stepping or moving the robot.
probe = mujoco.MjData(model)
set_lift_height(model, probe, .8)
for sign in (-1, 1):
    probe.ctrl[control.arm.ids] = sign*100
    mujoco.mj_forward(model, probe)
    assert np.allclose(probe.actuator_force[control.arm.ids], sign*TORQUE_LIMITS)
    for i, limit in enumerate(TORQUE_LIMITS, 1):
        dof = model.joint(f'joint_{i}').dofadr[0]
        assert np.isclose(probe.qfrc_actuator[dof], sign*limit)
print('All seven actuator torques saturate at their rated soft limits in both directions.')

# Controller-only full strokes give exact command timing without servo lag.
for target in (STROKE, 0.):
    panel(lift=target)
    steps = int(np.ceil(STROKE/NO_LOAD_SPEED/dt))
    for n in range(steps):
        old = control.lift_command
        control.update(data, dt)
        assert abs(control.lift_command-old) <= NO_LOAD_SPEED*dt+1e-12
        if n % 17 == 0:
            panel()
    assert abs(control.lift_command-target) < 1e-10
    print(f'Full lift stroke to {target:g} m: {steps*dt:.3f} s (16 mm/s).')

panel(lift=-100, arm=[100]*7)
assert control.lift_target == 0
assert np.allclose(control.arm.target[[1, 3, 5]], POSITION_LIMITS[[1, 3, 5]])
panel(lift=100)
assert control.lift_target == STROKE
panel(lift=float('nan'), arm=[float('nan')]*7)
assert np.isfinite(control.lift_target) and np.isfinite(control.arm.target).all()

# Repeated abrupt retargets including hard stops and reversal mid-movement.
rng = np.random.default_rng(4)
for n in range(18000):
    if n % 379 == 0:
        panel(arm=rng.uniform(-5, 5, 7))
    previous = control.arm.command.copy()
    velocity = control.arm.velocity.copy()
    control.update(data, dt)
    assert (np.abs(control.arm.command-previous) <= SPEED_LIMITS*dt+1e-12).all()
    assert (np.abs(control.arm.velocity-velocity) <= ACCELERATION_LIMITS*dt+1e-12).all()
    assert (np.abs(control.arm.command) <= POSITION_LIMITS+1e-10).all()
panel(arm=[0]*7)
for _ in range(15000):
    control.update(data, dt)
assert np.max(np.abs(control.arm.command)) < 1e-8
assert np.max(np.abs(control.arm.velocity)) < 1e-8
print('All seven joint references obey angle, speed, and acceleration limits during retargeting.')

# Exercise actual coupled dynamics with the 15 kg base, lift, and arm attached.
data = mujoco.MjData(model)
set_lift_height(model, data, .8)
control = ViewerControl(model, data, .8)
arm_dofs = [model.joint(f'joint_{i}').dofadr[0] for i in range(1, 8)]
peak_torque = np.zeros(7)
peak_speed = np.zeros(7)
for n in range(8000):
    if n == 1000:
        panel(lift=STROKE, arm=[.6, .3, -.5, -.4, .5, .3, -.5])
    if n % 17 == 0:
        panel()
    control.update(data, dt)
    mujoco.mj_step(model, data)
    peak_torque = np.maximum(peak_torque, np.abs(data.actuator_force[control.arm.ids]))
    peak_speed = np.maximum(peak_speed, np.abs(data.qvel[arm_dofs]))
    assert (peak_torque <= TORQUE_LIMITS+1e-9).all()
    assert np.isfinite(data.qpos).all()
assert not any(w.number for w in data.warning)
assert (peak_speed <= SPEED_LIMITS).all(), peak_speed
print('Coupled dynamics passed. Peak arm speeds (rad/s):', np.round(peak_speed, 3))
print('Peak arm torques (Nm):', np.round(peak_torque, 3))
