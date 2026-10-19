"""Repeatable, resettable whole-robot MuJoCo measurements without a viewer."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from build_scissor_lift import MIN_HEIGHT, MAX_HEIGHT
from joint_control import POSITION_LIMITS
from lift_kinematics import set_lift_height
from viewer_control import ViewerControl
from stability_metrics import StabilityMetrics, CORNERS

ROOT = Path(__file__).resolve().parents[1]
MOTIONS = {'stop', 'forward', 'backward', 'left', 'right', 'strafe-left', 'strafe-right'}


def validate(config, dt):
    def positive(value):
        return np.isfinite(value) and value > 0
    def height(value):
        return np.isfinite(value) and MIN_HEIGHT <= value <= MAX_HEIGHT
    def arm(value):
        a = np.asarray(value)
        return a.shape == (7,) and np.isfinite(a).all() and (np.abs(a) <= POSITION_LIMITS).all()
    if not positive(config['settle_s']) or not positive(config['sample_hz']) or config['sample_hz'] > 1/dt:
        raise ValueError('Require positive settling time and sampling rate <= physics rate')
    limits = config['safety']
    if not all(positive(limits[k]) for k in ('wheel_load_N', 'load_scale_N', 'tilt_rad')) or limits['tilt_rad'] >= np.pi/2:
        raise ValueError('Require positive safety thresholds and tilt < pi/2')
    names = set()
    if not config['scenarios']:
        raise ValueError('At least one scenario is required')
    for s in config['scenarios']:
        name = s['name']
        if not name or not all(c.isalnum() or c in '_-' for c in name) or name in names:
            raise ValueError('Scenario names must be unique simple filename components')
        names.add(name)
        if not height(s['initial_height_m']) or not arm(s['initial_arm_rad']) or not s['segments']:
            raise ValueError(f'Invalid initialization or empty segments: {name}')
        for seg in s['segments']:
            if (not positive(seg['duration_s']) or seg['motion'] not in MOTIONS
                    or not height(seg.get('height_m', s['initial_height_m']))
                    or not arm(seg.get('arm_rad', s['initial_arm_rad']))):
                raise ValueError(f'Invalid segment: {name}')


def snapshot(model, data, control, metrics, phase, segment):
    spatial_velocity = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                            model.body('mobile_base').id, spatial_velocity, 0)
    return {
        'time_s': float(data.time), 'phase': phase, 'segment': segment,
        'qpos': data.qpos.tolist(), 'qvel': data.qvel.tolist(), 'act': data.act.tolist(),
        'ctrl': data.ctrl.tolist(), 'actuator_force': data.actuator_force.tolist(),
        'arm_target_rad': control.arm.target.tolist(),
        'arm_reference_rad': control.arm.command.tolist(),
        'arm_reference_velocity_rad_s': control.arm.velocity.tolist(),
        'lift_target_m': control.lift_target+MIN_HEIGHT,
        'lift_reference_m': control.lift_command+MIN_HEIGHT,
        'base_command_target': list(control.drive.target),
        'base_command_reference': [control.drive.linear, control.drive.lateral, control.drive.yaw],
        'base_spatial_velocity_world_angular_linear': spatial_velocity.tolist(),
        'metrics': metrics,
    }


def run(config, output):
    model = mujoco.MjModel.from_xml_path(str(ROOT/'models/scene.xml'))
    dt = model.opt.timestep
    validate(config, dt)
    # Refuse existing directories to preserve earlier experiment results.
    output.mkdir(parents=True, exist_ok=False)
    meter = StabilityMetrics(model)
    manifest = {
        'schema_version': 1, 'config': config, 'mujoco_version': mujoco.__version__,
        'timestep_s': dt, 'nq': model.nq, 'nv': model.nv, 'nu': model.nu,
        'mass_kg': meter.mass, 'wheel_order': list(CORNERS),
        'actuator_names': [model.actuator(i).name for i in range(model.nu)],
        'actuator_limits': [{'name': model.actuator(i).name,
                             'control_limited': bool(model.actuator_ctrllimited[i]),
                             'control_range': model.actuator_ctrlrange[i].tolist(),
                             'force_limited': bool(model.actuator_forcelimited[i]),
                             'force_range': model.actuator_forcerange[i].tolist()}
                            for i in range(model.nu)],
        'joint_layout': [{'name': model.joint(i).name, 'type': int(model.jnt_type[i]),
                          'qpos_address': int(model.jnt_qposadr[i]), 'qvel_address': int(model.jnt_dofadr[i])}
                         for i in range(model.njnt)],
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(list((ROOT/'models').glob('*.xml'))+list((ROOT/'scripts').glob('*.py')))},
        'interpretation': 'Observed margins under scripted controls; not viability labels or hardware certification.',
        'scenarios': [],
    }
    interval = max(1, round(1/(config['sample_hz']*dt)))
    manifest['actual_sample_hz'] = 1/(interval*dt)
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    for scenario in config['scenarios']:
        data = mujoco.MjData(model)
        set_lift_height(model, data, scenario['initial_height_m'])
        for i, angle in enumerate(scenario['initial_arm_rad'], 1):
            data.qpos[model.joint(f'joint_{i}').qposadr[0]] = angle
            data.ctrl[model.actuator(f'joint_{i}_actuator').id] = angle
        mujoco.mj_forward(model, data)
        control = ViewerControl(model, data, scenario['initial_height_m'], speed=.3)
        phases = [('settle', {'duration_s': config['settle_s'], 'motion': 'stop'})]
        phases += [('maneuver', seg) for seg in scenario['segments']]
        summary = {'name': scenario['name'], 'minimum_g_observed': None,
                   'minimum_wheel_load_N': [float('inf')]*4, 'maximum_tilt_rad': 0.,
                   'first_violation_time_s': None, 'samples': 0, 'physics_steps': 0,
                   'maximum_nonwheel_ground_contacts': 0}
        step = 0
        with (output/(scenario['name']+'.jsonl')).open('w') as stream:
            for index, (phase, seg) in enumerate(phases):
                control.drive.set_motion(seg['motion'])
                if 'height_m' in seg:
                    control.lift_target = seg['height_m']-MIN_HEIGHT
                if 'arm_rad' in seg:
                    control.arm.set_target(np.array(seg['arm_rad']))
                count = max(1, round(seg['duration_s']/dt))
                for k in range(count):
                    control.update(data, dt)
                    mujoco.mj_step(model, data)
                    mujoco.mj_forward(model, data)
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
                        raise RuntimeError(f'Invalid simulation or MuJoCo warning in {scenario["name"]} at {data.time}')
                    measured = meter.measure(data, config['safety'])
                    # Settling is logged, but excluded from maneuver outcome summaries.
                    if phase == 'maneuver':
                        g = measured['g_observed']
                        summary['minimum_g_observed'] = g if summary['minimum_g_observed'] is None else min(g, summary['minimum_g_observed'])
                        summary['minimum_wheel_load_N'] = np.minimum(summary['minimum_wheel_load_N'], measured['wheel_load_N']).tolist()
                        summary['maximum_tilt_rad'] = max(summary['maximum_tilt_rad'], measured['base_tilt_rad'])
                        summary['maximum_nonwheel_ground_contacts'] = max(summary['maximum_nonwheel_ground_contacts'], measured['nonwheel_ground_contacts'])
                        if g < 0 and summary['first_violation_time_s'] is None:
                            summary['first_violation_time_s'] = float(data.time)
                    if step % interval == 0 or k == count-1:
                        stream.write(json.dumps(snapshot(model, data, control, measured, phase, index-1), allow_nan=False)+'\n')
                        summary['samples'] += 1
                    step += 1
        summary['physics_steps'] = step
        summary['observed_requirement_satisfied'] = summary['minimum_g_observed'] >= 0
        manifest['scenarios'].append(summary)
        (output/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
        print(json.dumps(summary), flush=True)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'scenarios/stability_baseline.json')
    parser.add_argument('--output', type=Path, required=True, help='New directory for manifest and JSONL logs')
    args = parser.parse_args()
    run(json.loads(args.config.read_text()), args.output)
