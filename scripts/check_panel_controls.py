"""Exercise native Control edits using the same after-sync path as run_sim."""
from pathlib import Path
import numpy as np
import mujoco
from lift_kinematics import set_lift_height
from viewer_control import ViewerControl

model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'models/scene.xml'))
data = mujoco.MjData(model)
set_lift_height(model, data, .8)
controls = ViewerControl(model, data, .9, motion='forward')


def panel_edit(changes):
    previous = controls.before_sync(data)
    for name, value in changes.items():
        data.ctrl[model.actuator(name).id] = value
    controls.after_sync(previous, data)


def step(seconds):
    for _ in range(round(seconds/model.opt.timestep)):
        controls.update(data, model.opt.timestep)
        mujoco.mj_step(model, data)


step(.1)
assert data.ctrl[controls.lift_id] > .535, 'CLI lift ramp should still work'
assert not controls.wheels_from_panel
panel_edit({'lift_motor': .555, **{'wheel_'+c+'_motor': .8 for c in ('fl', 'fr', 'bl', 'br')}})
requested = data.ctrl[controls.wheel_ids].copy()
step(3)
assert np.array_equal(data.ctrl[controls.wheel_ids], requested), 'Panel commands were overwritten'
assert controls.lift_target == .555
assert data.body('mobile_base').xpos[0] > .12
lift_height = data.qpos[model.joint('lift_joint').qposadr[0]]+.265
assert abs(lift_height-.820) < .015, lift_height
print('Panel lift and wheel commands persist and move the model.')

panel_edit({'wheel_'+c+'_motor': 0. for c in ('fl', 'fr', 'bl', 'br')})
step(2)
assert np.linalg.norm(data.qvel[:3]) < .015
assert controls.wheels_from_panel
controls.drive.key_callback(ord('Q'))
step(3)
assert not controls.wheels_from_panel
assert data.body('mobile_base').xpos[1] > .2
assert data.ctrl[controls.lift_id] == .555, 'Keyboard must not take over lift slider'

# Moving one wheel slider releases all four motors from the keyboard mixer.
panel_edit({'wheel_fl_motor': 0.})
requested = data.ctrl[controls.wheel_ids].copy()
step(.1)
assert controls.wheels_from_panel
assert np.array_equal(data.ctrl[controls.wheel_ids], requested)
controls.drive.key_callback(ord('X'))
step(2)
assert np.linalg.norm(data.qvel[:3]) < .015
assert not any(w.number for w in data.warning)
print('Panel stop, keyboard takeover, slider takeover, and X stop all pass.')
