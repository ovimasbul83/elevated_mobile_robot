"""Exercise keyboard commands through complete MuJoCo drive/stop simulations."""
from pathlib import Path
import numpy as np
import mujoco
from lift_kinematics import set_lift_height
from wheel_control import WheelControl

ROOT = Path(__file__).resolve().parents[1]
model = mujoco.MjModel.from_xml_path(str(ROOT/'models/scene.xml'))
data = mujoco.MjData(model)
for key, name, direction in [('W', 'forward', 1), ('S', 'reverse', -1),
                             ('A', 'left', 1), ('D', 'right', -1),
                             ('Q', 'strafe-left', 1), ('E', 'strafe-right', -1)]:
    mujoco.mj_resetData(model, data)
    set_lift_height(model, data, .8)
    control = WheelControl(model)
    control.key_callback(ord(key))
    for _ in range(4000):
        control.update(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    rotation = data.body('mobile_base').xmat.reshape(3, 3)
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    x = data.body('mobile_base').xpos[0]
    y = data.body('mobile_base').xpos[1]
    assert np.isfinite(data.qpos).all() and not any(w.number for w in data.warning)
    assert rotation[2, 2] > .98, 'Base tilted excessively'
    if key in 'WS':
        assert direction*x > .2, (name, x)
        assert abs(y) < .03 and abs(yaw) < .06, (name, y, yaw)
    elif key in 'QE':
        assert direction*y > .25, (name, y)
        assert abs(x) < .03 and abs(yaw) < .06, (name, x, yaw)
    else:
        assert direction*yaw > .6, (name, yaw)
        assert np.hypot(x, y) < .03
    control.key_callback(ord('X'))
    for _ in range(2500):
        control.update(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    wheel_speed = max(abs(data.qvel[model.joint('wheel_'+c+'_joint').dofadr[0]]) for c in ('fl', 'fr', 'bl', 'br'))
    assert wheel_speed < .05, (name, wheel_speed)
    assert np.linalg.norm(data.qvel[:3]) < .015
    assert np.linalg.norm(data.qvel[3:6]) < .03
    print(f'{name}: x={x:.3f} m, y={y:.3f} m, yaw={yaw:.3f} rad; X stopped the wheels.')

# A strafe command must follow the robot frame after changing heading.
mujoco.mj_resetData(model, data)
data.qpos[3:7] = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
set_lift_height(model, data, .8)
control = WheelControl(model)
control.set_motion('strafe-left')
for _ in range(4000):
    control.update(data, model.opt.timestep)
    mujoco.mj_step(model, data)
assert data.body('mobile_base').xpos[0] < -.25
assert abs(data.body('mobile_base').xpos[1]) < .03
assert not any(w.number for w in data.warning)
print('Strafing follows robot heading after a 90-degree rotation.')
