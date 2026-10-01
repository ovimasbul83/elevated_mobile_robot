"""Approximate passive 45-degree roller contacts, independent of visual CAD."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

ROLLER_COUNT = 12
ROLLER_MASS = .020
# Roller axes at the bottom: FL/BR oppose FR/BL, giving an X configuration.
HANDEDNESS = {'fl': 1, 'fr': -1, 'bl': -1, 'br': 1}


def vec(values):
    return ' '.join(f'{v:.12g}' for v in values)


def configure_wheels(root):
    for corner, handedness in HANDEDNESS.items():
        wheel = root.find(f".//body[@name='wheel_{corner}']")
        envelope = wheel.find(f"geom[@name='wheel_{corner}_geom']")
        radius, half_width = map(float, envelope.get('size').split())
        envelope.set('contype', '0')
        envelope.set('conaffinity', '0')
        hub_mass = 1-ROLLER_COUNT*ROLLER_MASS
        transverse = hub_mass*(3*radius**2+(2*half_width)**2)/12
        wheel.find('inertial').set('mass', str(hub_mass))
        wheel.find('inertial').set('diaginertia', vec([transverse, hub_mass*radius**2/2, transverse]))
        for child in list(wheel.findall('body')):
            if child.get('name', '').startswith(f'{corner}_roller_'):
                wheel.remove(child)
        # Smooth roller profile approximates the CAD tread; dimensions are not
        # extracted manufacturing geometry. All roller axes are at 45 degrees.
        minor = radius*.22
        ring_radius = radius-minor
        major = math.sqrt(2*half_width**2-minor**2)
        for i in range(ROLLER_COUNT):
            angle = 2*math.pi*i/ROLLER_COUNT
            axis = [math.cos(angle)/math.sqrt(2), handedness/math.sqrt(2), -math.sin(angle)/math.sqrt(2)]
            body = ET.SubElement(wheel, 'body', name=f'{corner}_roller_{i:02}',
                                 pos=vec([ring_radius*math.sin(angle), 0, ring_radius*math.cos(angle)]))
            ET.SubElement(body, 'joint', name=f'{corner}_roller_{i:02}_joint', type='hinge',
                          axis=vec(axis), damping='0.00002', armature='0.000001')
            quat = [1+axis[2], -axis[1], axis[0], 0]
            norm = math.sqrt(sum(v*v for v in quat))
            ET.SubElement(body, 'geom', name=f'{corner}_roller_{i:02}_contact', type='ellipsoid',
                          size=vec([minor, minor, major]), quat=vec([v/norm for v in quat]),
                          mass=str(ROLLER_MASS), rgba='0 0 0 0', group='3',
                          contype='4', conaffinity='1', condim='3', friction='1.0 0.001 0.0001')
    return root


def main():
    root = Path(__file__).resolve().parents[1]
    path = root/'models/wheels.xml'
    tree = ET.parse(path)
    configure_wheels(tree.getroot())
    ET.indent(tree, space='    ')
    tree.write(path, encoding='unicode')
    path = root/'models/assets/meshes/wheels/wheel_metadata.json'
    info = json.loads(path.read_text())
    info.update(contact_model='12 passive 45-degree ellipsoid rollers per wheel',
                rollers_per_wheel=ROLLER_COUNT, roller_mass_kg=ROLLER_MASS,
                mass_per_rotating_wheel_kg=1, roller_geometry='approximate; CAD visual geometry unchanged')
    path.write_text(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
