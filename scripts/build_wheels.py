"""Import resolved wheel CAD meshes into MuJoCo with passive roller contacts.

Requires numpy and fast-simplification (the latter may be in .cad_deps).
Source STL units are metres; Inventor occurrence translations are centimetres.
"""
from pathlib import Path
import json
import struct
import sys
import xml.etree.ElementTree as ET
import numpy as np
from mecanum_geometry import configure_wheels, ROLLER_COUNT, ROLLER_MASS

ROOT = Path(__file__).resolve().parents[1]
try:
    import fast_simplification
except ImportError:
    # Retain support for the original local CAD dependency bundle.
    sys.path.insert(0, str(ROOT / '.cad_deps'))
    import fast_simplification

SOURCE = ROOT / 'outputs/wheel_cad'
DEST = ROOT / 'models/assets/meshes/wheels'
DTYPE = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])


def read_stl(path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    assert len(raw) == 84+50*count
    return np.frombuffer(raw, dtype=DTYPE, count=count, offset=84)['v'].astype(float)


def write_stl(path, triangles):
    data = np.zeros(len(triangles), dtype=DTYPE)
    data['v'] = triangles
    normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(normal, axis=1)
    normal /= np.maximum(lengths[:, None], 1e-30)
    data['normal'] = normal
    path.write_bytes(b'Wheel CAD - metres'.ljust(80, b' ')+struct.pack('<I', len(data))+data.tobytes())


def simplify(name, triangles):
    target = 250000 if len(triangles) > 100000 else min(6000, len(triangles))
    cache = SOURCE / (name+f'_simplified_{target}.stl')
    if cache.exists():
        return read_stl(cache)
    vertices, inverse = np.unique(np.round(triangles.reshape(-1, 3), 8), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    if len(faces) > target:
        vertices, faces = fast_simplification.simplify(vertices, faces, target_count=target, agg=5)
    result = vertices[faces]
    write_stl(cache, result)
    print(f'{name}: {len(triangles):,} -> {len(result):,} triangles', flush=True)
    return result


def el(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def vec(v):
    return ' '.join(f'{x:.10g}' for x in v)


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    instances = json.loads((SOURCE/'mesh_instances.json').read_text(encoding='utf-8-sig'))
    groups = {key: [] for key in ('fixed_metal', 'motor', 'wheel_hub', 'wheel_rollers', 'wheel_adapter')}
    parts = {}
    radius = 0
    wheel_center = None
    wheel_width = 0
    for item in instances:
        filename = item['Mesh']
        name = item['Name']
        if filename not in parts:
            original = read_stl(SOURCE/filename)
            parts[filename] = simplify(Path(filename).stem, original)
            if 'Meca Wheel' in name:
                # Use original geometry, not loose CAD bounding boxes, for contact size.
                wheel_width = float(np.ptp(original[:, :, 2]))
                radius = float(np.linalg.norm(original[:, :, :2], axis=2).max())
        local = parts[filename]
        matrix = np.array(item['Transform']).reshape(4, 4)
        triangles = local @ matrix[:3, :3].T + matrix[:3, 3]*.01
        if 'Meca Wheel' in name:
            wheel_center = matrix[:3, 3]*.01
            # Color distinction is cosmetic; every surface remains from the CAD.
            roller = np.linalg.norm(local.mean(axis=1)[:, :2], axis=1) > .050
            groups['wheel_hub'].append(triangles[~roller])
            groups['wheel_rollers'].append(triangles[roller])
        elif 'Wheel Adapter' in name or '4XU49' in name:
            groups['wheel_adapter'].append(triangles)
        elif 'CubeMars' in name:
            groups['motor'].append(triangles)
        else:
            groups['fixed_metal'].append(triangles)
    groups = {key: np.concatenate(values) for key, values in groups.items()}
    assert wheel_center is not None and .07 < radius < .09
    # Source shaft axis X -> robot shaft axis Y; outward is negative Y on right.
    rotate = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.]])
    assets = ET.Element('mujocoinclude')
    asset = el(assets, 'asset')
    wheels = ET.Element('mujocoinclude')
    wheels.append(ET.Comment(' CAD visual meshes with mirrored diagonal handedness and approximate passive 45-degree roller contacts. Wheel motors are defined in scene.xml. Each hub plus rollers totals 1 kg. Fixed hardware mass is lumped into the chassis. '))
    colors = {'fixed_metal': '.55 .58 .62 1', 'motor': '.09 .10 .12 1',
              'wheel_hub': '.66 .69 .73 1', 'wheel_rollers': '.065 .07 .08 1', 'wheel_adapter': '.5 .53 .58 1'}
    for corner, x, side in [('fl', .304, 1), ('fr', .304, -1), ('bl', -.304, 1), ('br', -.304, -1)]:
        # Mirror across the robot centreline and rear axle to form an X pattern.
        mirror = np.diag([1 if x > 0 else -1, 1 if side < 0 else -1, 1])
        transform = mirror @ rotate
        center = transform @ wheel_center
        module = el(wheels, 'body', name=f'wheel_module_{corner}', pos=vec([x, side*.215, -.030]))
        wheel = el(module, 'body', name=f'wheel_{corner}', pos=vec(center))
        el(wheel, 'joint', name=f'wheel_{corner}_joint', type='hinge', axis='0 1 0')
        transverse = (3*radius**2+wheel_width**2)/12
        el(wheel, 'inertial', pos='0 0 0', mass='1', diaginertia=vec([transverse, radius**2/2, transverse]))
        el(wheel, 'geom', name=f'wheel_{corner}_geom', type='cylinder', size=vec([radius, wheel_width/2]),
           euler='1.570796326794897 0 0', mass='0', rgba='0 0 0 0', group='3', friction='1.0 0.01 0.001')
        for group, source in groups.items():
            rotating = group.startswith('wheel_')
            triangles = source @ transform.T
            if rotating:
                triangles -= center
            if np.linalg.det(transform) < 0:
                triangles = triangles[:, [0, 2, 1]]
            # MuJoCo's STL decoder caps each file at 200,000 faces.
            for chunk, offset in enumerate(range(0, len(triangles), 170000)):
                meshname = f'{corner}_{group}' + (f'_{chunk}' if chunk else '')
                write_stl(DEST/(meshname+'.stl'), triangles[offset:offset+170000])
                el(asset, 'mesh', name=meshname, file=f'wheels/{meshname}.stl')
                el(wheel if rotating else module, 'geom', name=meshname+'_visual', type='mesh', mesh=meshname,
                   rgba=colors[group], contype='0', conaffinity='0', group='2', mass='0')
    configure_wheels(wheels)
    for tree, filename in [(assets, 'wheel_assets.xml'), (wheels, 'wheels.xml')]:
        ET.indent(tree, space='    ')
        ET.ElementTree(tree).write(ROOT/'models'/filename, encoding='unicode')
    report = {'diameter_m': 2*radius, 'width_m': wheel_width, 'wheel_center_in_assembly_m': wheel_center.tolist(),
              'mobile_base_z_m': radius+.030, 'mass_per_rotating_wheel_kg': 1,
              'contact_model': '12 passive 45-degree ellipsoid rollers per wheel',
              'rollers_per_wheel': ROLLER_COUNT, 'roller_mass_kg': ROLLER_MASS,
              'roller_geometry': 'approximate; CAD visual geometry unchanged',
              'fixed_hardware_mass': 'not calibrated; visual only',
              'mirroring': 'front-right reference; other corners reflected for diagonal handedness'}
    (DEST/'wheel_metadata.json').write_text(json.dumps(report, indent=2))
    # Only replace wheel bodies and ground clearance in the existing chassis.
    path = ROOT/'models/mobile_manipulator.xml'
    mobile_tree = ET.parse(path)
    base = mobile_tree.getroot().find('body')
    base.set('pos', vec([0, 0, radius+.030]))
    for body in list(base.findall('body')):
        if body.get('name') in ('wheel_fl', 'wheel_fr', 'wheel_bl', 'wheel_br'):
            base.remove(body)
    if not any(include.get('file') == 'wheels.xml' for include in base.findall('include')):
        base.insert(list(base).index(base.find('include')), ET.Element('include', {'file': 'wheels.xml'}))
    ET.indent(mobile_tree, space='    ')
    mobile_tree.write(path, encoding='unicode')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
