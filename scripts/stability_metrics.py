"""Whole-robot contact diagnostics; no pose changes or artificial forces."""
import numpy as np
import mujoco

CORNERS = ('fl', 'fr', 'bl', 'br')


def convex_hull(points):
    """Counterclockwise 2-D hull, excluding collinear interior points."""
    points = sorted(set(map(tuple, points)))
    if len(points) < 3:
        return points
    def cross(o, a, b):
        v, w = np.subtract(a, o), np.subtract(b, o)
        return v[0]*w[1]-v[1]*w[0]
    lower, upper = [], []
    for chain, sequence in ((lower, points), (upper, reversed(points))):
        for p in sequence:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
    return lower[:-1] + upper[:-1]


def hull_margin(point, hull):
    """Minimum inward edge distance; undefined for a degenerate polygon."""
    if len(hull) < 3:
        return None
    a = np.asarray(hull)
    edges = np.roll(a, -1, axis=0)-a
    offset = np.asarray(point)-a
    distances = (edges[:, 0]*offset[:, 1]-edges[:, 1]*offset[:, 0])/np.linalg.norm(edges, axis=1)
    return float(distances.min())


class StabilityMetrics:
    def __init__(self, model):
        self.model = model
        self.floor = model.geom('floor').id
        self.base = model.body('mobile_base').id
        self.mass = float(model.body_mass.sum())
        self.geom_corner = {model.geom(f'{c}_roller_{i:02d}_contact').id: k
                            for k, c in enumerate(CORNERS) for i in range(12)}

    def measure(self, data, limits):
        """Call after mj_forward so poses and solved contact forces agree."""
        loads = np.zeros(4)
        force, moment = np.zeros(3), np.zeros(3)
        wheel_force = np.zeros(3)
        points, nonwheel = [], 0
        for i in range(data.ncon):
            contact = data.contact[i]
            if self.floor not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == self.floor else contact.geom1
            local = np.zeros(6)
            mujoco.mj_contactForce(self.model, data, i, local)
            sign = 1 if contact.geom1 == self.floor else -1
            rotation = contact.frame.reshape(3, 3).T
            f, torque = sign*(rotation@local[:3]), sign*(rotation@local[3:])
            force += f
            moment += np.cross(contact.pos, f)+torque
            if other in self.geom_corner:
                loads[self.geom_corner[other]] += f[2]
                wheel_force += f
                if f[2] > 1e-6:
                    points.append(contact.pos[:2].tolist())
            elif np.linalg.norm(f) > 1e-6:
                nonwheel += 1
        rotation = data.xmat[self.base].reshape(3, 3)
        roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
        pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1, 1)))
        tilt = float(np.arccos(np.clip(rotation[2, 2], -1, 1)))
        cop = [-float(moment[1]/force[2]), float(moment[0]/force[2])] if force[2] > 1e-6 else None
        hull = convex_hull(points)
        # Provisional no-unloading/no-large-tilt requirement, not a tipping certificate.
        margins = {
            'wheel_load': float((loads.min()-limits['wheel_load_N'])/limits['load_scale_N']),
            'base_tilt': float((limits['tilt_rad']-tilt)/limits['tilt_rad']),
        }
        return {
            'wheel_load_N': loads.tolist(), 'ground_force_world_N': force.tolist(),
            'ground_moment_world_Nm': moment.tolist(),
            'nonwheel_ground_contacts': nonwheel,
            'wheel_ground_force_world_N': wheel_force.tolist(),
            'cop_world_xy_m': cop, 'active_wheel_support_hull_world_xy_m': hull,
            'cop_active_support_margin_m': hull_margin(cop, hull) if cop is not None else None,
            'com_world_m': data.subtree_com[self.base].tolist(),
            'base_roll_rad': roll, 'base_pitch_rad': pitch, 'base_tilt_rad': tilt,
            'normalized_margins': margins, 'g_observed': min(margins.values()),
        }
