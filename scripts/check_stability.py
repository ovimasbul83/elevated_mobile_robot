"""Check contact-wrench conventions, physical balance, failure detection and replay."""
import json
import tempfile
from pathlib import Path
import numpy as np
import mujoco
from lift_kinematics import set_lift_height
from stability_metrics import StabilityMetrics, convex_hull, hull_margin
from run_stability_scenarios import ROOT, run, validate

model = mujoco.MjModel.from_xml_path(str(ROOT/'models/scene.xml'))
data = mujoco.MjData(model)
meter = StabilityMetrics(model)
limits = {'wheel_load_N': 5., 'load_scale_N': 100., 'tilt_rad': np.deg2rad(10)}
square = convex_hull([(1,1),(-1,1),(1,-1),(-1,-1),(0,0)])
assert np.isclose(hull_margin([0,0], square), 1)
assert np.isclose(hull_margin([2,0], square), -1)
assert hull_margin([0,0], [(0,0),(1,0)]) is None
set_lift_height(model, data, .8)
for _ in range(2000):
    mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)
metrics = meter.measure(data, limits)
weight = meter.mass*9.81
assert np.isclose(sum(metrics['wheel_load_N']), weight, rtol=.01)
assert np.isclose(metrics['ground_force_world_N'][2], weight, rtol=.01)
assert metrics['g_observed'] > 0
assert metrics['nonwheel_ground_contacts'] == 0
assert metrics['cop_active_support_margin_m'] > .1
assert np.linalg.norm(np.array(metrics['cop_world_xy_m'])-data.subtree_com[meter.base][:2]) < .005
# A rigid translation changes moment about origin, but preserves relative CoP.
original_cop = np.array(metrics['cop_world_xy_m'])
data.qpos[:2] += [2., -3.]
mujoco.mj_forward(model, data)
translated = meter.measure(data, limits)
assert np.allclose(np.array(translated['cop_world_xy_m'])-original_cop, [2.,-3.], atol=1e-5)
# An airborne configuration must not be called supported or assigned a CoP.
data.qpos[2] += 2
mujoco.mj_forward(model, data)
airborne = meter.measure(data, limits)
assert airborne['g_observed'] < 0 and airborne['cop_world_xy_m'] is None
assert airborne['cop_active_support_margin_m'] is None
print('Contact force sign, weight balance, CoP translation and support-loss checks passed.')

config = {'settle_s': .1, 'sample_hz': 50, 'safety': limits,
          'scenarios': [{'name': 'repeat', 'initial_height_m': .8,
                         'initial_arm_rad': [0]*7,
                         'segments': [{'duration_s': .1, 'motion': 'forward',
                                       'arm_rad': [.1]*7, 'height_m': .81}]}]}
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)
    first, second = run(config, path/'a'), run(config, path/'b')
    assert first['scenarios'] == second['scenarios']
    assert (path/'a/repeat.jsonl').read_bytes() == (path/'b/repeat.jsonl').read_bytes()
    rows = [json.loads(line) for line in (path/'a/repeat.jsonl').read_text().splitlines()]
    assert {r['phase'] for r in rows} == {'settle','maneuver'}
    assert len(rows[-1]['arm_reference_rad']) == 7
    assert 0 < rows[-1]['lift_reference_m']-.8 <= .016*.1+1e-10
    try:
        run(config, path/'a')
    except FileExistsError:
        pass
    else:
        raise AssertionError('Existing output directory was overwritten')
bad = json.loads(json.dumps(config))
bad['scenarios'][0]['segments'][0]['arm_rad'] = [0,10,0,0,0,0,0]
try:
    validate(bad, model.opt.timestep)
except ValueError:
    pass
else:
    raise AssertionError('Out-of-range joint target was accepted')
print('Scenario reproducibility, controller limits, logging and input validation passed.')
