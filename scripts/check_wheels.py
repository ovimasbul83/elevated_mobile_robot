"""Check imported wheel placement/contact and render CAD wheel details."""
from pathlib import Path
import json
import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
(ROOT / 'outputs').mkdir(exist_ok=True)
model = mujoco.MjModel.from_xml_path(str(ROOT/'models/scene.xml'))
data = mujoco.MjData(model)
meta = json.loads((ROOT/'models/assets/meshes/wheels/wheel_metadata.json').read_text())
mujoco.mj_forward(model, data)
base_mass = model.body_subtreemass[model.body('mobile_base').id]-model.body_subtreemass[model.body('scissor_lift_base').id]
assert abs(base_mass-15.0) < 1e-9, f'Mobile base including wheels: {base_mass} kg'
print(f'Mobile base including four wheels: {base_mass:.6f} kg (excluding lift, platform, and robot).')
radius = meta['diameter_m']/2
for corner in ('fl', 'fr', 'bl', 'br'):
    body = data.body('wheel_'+corner)
    assert abs(body.xpos[2]-radius) < 1e-8, 'Wheel not tangent to ground'
    assert abs(abs(body.xpos[1])-(.215+.04857332)) < 1e-8
    assert model.geom(corner+'_motor_visual').bodyid != model.body('wheel_'+corner).id
    # Changing wheel angle must not move its fixed motor/mount.
    fixed = data.geom(corner+'_motor_visual').xmat.copy()
    data.qpos[model.joint('wheel_'+corner+'_joint').qposadr[0]] = .5
    mujoco.mj_forward(model, data)
    assert np.allclose(data.geom(corner+'_motor_visual').xmat, fixed)
mujoco.mj_resetData(model, data)
for _ in range(2000):
    mujoco.mj_step(model, data)
floor = model.geom('floor').id
touching = set()
for contact in data.contact:
    if floor in contact.geom:
        other = contact.geom[0] if contact.geom[1] == floor else contact.geom[1]
        touching.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other))
assert all(any(name.startswith(c+'_roller_') for name in touching) for c in ('fl', 'fr', 'bl', 'br')), touching
for corner in ('fl', 'fr', 'bl', 'br'):
    assert abs(model.body_subtreemass[model.body('wheel_'+corner).id]-1) < 1e-9
    assert model.geom('wheel_'+corner+'_geom').contype[0] == 0
assert not any(w.number for w in data.warning)
print('Four wheels touch the floor; fixed hardware remains stationary; simulation stable.')
model.vis.global_.offwidth = 1400
model.vis.global_.offheight = 1000
model.vis.headlight.ambient[:] = [.55, .55, .55]
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=900, width=1200) as renderer:
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, .29]
    cam.distance, cam.azimuth, cam.elevation = 1.45, 135, -24
    renderer.update_scene(data, camera=cam)
    Image.fromarray(renderer.render()).save(ROOT/'outputs/robot_cad_wheels.png')
    for geom in range(model.ngeom):
        body = model.geom_bodyid[geom]
        ancestors = []
        while body:
            ancestors.append(body)
            body = model.body_parentid[body]
        if model.body('wheel_module_fr').id not in ancestors:
            model.geom_rgba[geom, 3] = 0
    cam.lookat[:] = data.body('wheel_fr').xpos
    cam.distance, cam.azimuth, cam.elevation = .33, -60, -20
    renderer.update_scene(data, camera=cam)
    Image.fromarray(renderer.render()).save(ROOT/'outputs/wheel_cad_detail.png')
print('Rendered robot_cad_wheels.png and wheel_cad_detail.png')
