"""Check geometric closure, mass, travel, and simulation; render a preview."""
import argparse
from pathlib import Path
import mujoco
import numpy as np
from lift_kinematics import set_lift_height
from build_scissor_lift import MODEL_NAME, MIN_HEIGHT, MAX_HEIGHT, STROKE, NO_LOAD_SPEED, LIFT_MASS, FRAME_LENGTH, FRAME_WIDTH, MOUNT_SPACING
from build_scissor_lift import PLATFORM_THICKNESS, PLATFORM_MASS

ROOT = Path(__file__).resolve().parents[1]
PAIRS = [("lower_a_center", "lower_b_center"), ("upper_a_center", "upper_b_center"),
         ("upper_a_tip", "top_fixed_attachment"), ("actuator_tip", "actuator_attachment")]


def residual(model, data):
    return max(np.linalg.norm(data.site(a).xpos-data.site(b).xpos) for a, b in PAIRS)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(ROOT / "models/scene.xml"))
    data = mujoco.MjData(model)
    base_id = model.body("scissor_lift_base").id
    mass = model.body_subtreemass[base_id]-model.body_subtreemass[model.body("upper_platform").id]
    assert abs(mass-LIFT_MASS) < 1e-9, mass
    assert abs(model.body("upper_platform").mass[0]-PLATFORM_MASS) < 1e-9
    assert np.allclose(model.joint("lift_joint").range, [0, STROKE])
    assert np.allclose(model.actuator("lift_motor").ctrlrange, [0, STROKE])
    # Verify frame envelope and the C2-60's two end mounting bars.
    feet = [model.geom(f"base_mount_bar_{side}") for side in (-1, 1)]
    assert abs(feet[1].pos[0]+feet[1].size[0]-(feet[0].pos[0]-feet[0].size[0])-FRAME_LENGTH) < 1e-9
    assert abs(2*feet[0].size[1]-FRAME_WIDTH) < 1e-9
    bars = [model.geom(f"top_mount_bar_{i}") for i in (0, 1)]
    assert abs(bars[1].pos[0]-bars[0].pos[0]-MOUNT_SPACING) < 1e-9
    assert sum((mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("top_mount_bar_") for i in range(model.ngeom)) == 2
    print(f"Lift mass excluding platform and robot: {mass:.6f} kg; aluminium platform: {PLATFORM_MASS:.6f} kg")
    for height in (MIN_HEIGHT, .5, .8, 1.0, MAX_HEIGHT):
        mujoco.mj_resetData(model, data)
        set_lift_height(model, data, height)
        error = residual(model, data)
        assert error < 1e-8, (height, error)
        actual = data.body("upper_platform").xpos[2]-data.body("scissor_lift_base").xpos[2]
        assert abs(actual-height) < 1e-9
        plate = model.geom("aluminium_upper_platform")
        assert np.allclose(2*plate.size, [FRAME_LENGTH, FRAME_WIDTH, PLATFORM_THICKNESS])
        plate_bottom = data.geom("aluminium_upper_platform").xpos[2]-plate.size[2]
        for i in (0, 1):
            bar = model.geom(f"top_mount_bar_{i}")
            bar_bottom = data.geom(f"top_mount_bar_{i}").xpos[2]-bar.size[2]
            bar_top = data.geom(f"top_mount_bar_{i}").xpos[2]+bar.size[2]
            assert abs(plate_bottom-bar_top) < 1e-9, "Gap below aluminium platform"
            for side in (-1, 1):
                rail = model.geom(f"top_rail_{side}")
                rail_top = data.geom(f"top_rail_{side}").xpos[2]+rail.size[2]
                assert abs(bar_bottom-rail_top) < 1e-9, "Gap below mounting bars"
        assert abs(data.body("kinova_mount").xpos[2]-(plate_bottom+PLATFORM_THICKNESS)) < 1e-9
        for _ in range(3000):
            mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all()
        assert not any(w.number for w in data.warning), list(data.warning)
        error = residual(model, data)
        assert error < .001, (height, error)
        print(f"Height {height:.3f} m: pose exact; 3 s simulation loop error {error*1000:.4f} mm")
    for start, direction in ((.5, 1), (.8, -1)):
        mujoco.mj_resetData(model, data)
        set_lift_height(model, data, start)
        for step in range(5000):
            data.ctrl[model.actuator("lift_motor").id] = start-MIN_HEIGHT + direction*NO_LOAD_SPEED*(step+1)*model.opt.timestep
            mujoco.mj_step(model, data)
        actual = data.qpos[model.joint("lift_joint").qposadr[0]]+MIN_HEIGHT
        assert abs(actual-(start+direction*NO_LOAD_SPEED*5)) < .015, actual
        assert residual(model, data) < .001
        assert not any(w.number for w in data.warning)
        print(f"5 s {'raising' if direction > 0 else 'lowering'}: height {actual:.4f} m; no instability")
    if args.render:
        render(model, data)


def render(model, data):
    from PIL import Image, ImageDraw, ImageFont
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    model.vis.global_.offwidth = 1800
    model.vis.global_.offheight = 900
    model.vis.headlight.ambient[:] = [.6, .6, .6]
    # Assembled close-up includes the plate, robot base, and mobile chassis.
    mujoco.mj_resetData(model, data)
    set_lift_height(model, data, MIN_HEIGHT)
    with mujoco.Renderer(model, height=750, width=1100) as renderer:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, .38]
        cam.distance, cam.azimuth, cam.elevation = 1.35, 110, -15
        renderer.update_scene(data, camera=cam)
        Image.fromarray(renderer.render()).save(output / "upper_platform_detail.png")
    # Hide the mobile base and robot for the reference comparison only.
    for geom in range(model.ngeom):
        body = model.geom_bodyid[geom]
        ancestors = []
        while body:
            ancestors.append(body)
            body = model.body_parentid[body]
        if model.body("scissor_lift_base").id not in ancestors or model.body("kinova_mount").id in ancestors:
            model.geom_rgba[geom, 3] = 0
    model.vis.global_.offwidth = 1800
    model.vis.global_.offheight = 900
    model.vis.headlight.ambient[:] = [.6, .6, .6]
    panels = []
    with mujoco.Renderer(model, height=720, width=600) as renderer:
        for height in (MIN_HEIGHT, .8, MAX_HEIGHT):
            mujoco.mj_resetData(model, data)
            set_lift_height(model, data, height)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [0, 0, .78]
            cam.distance, cam.azimuth, cam.elevation = 2.1, 120, -16
            renderer.update_scene(data, camera=cam)
            panels.append(Image.fromarray(renderer.render()))
    canvas = Image.new("RGB", (1800, 820), "#f3f5f7")
    draw = ImageDraw.Draw(canvas)
    font_path = "C:/Windows/Fonts/arial.ttf"
    font = ImageFont.truetype(font_path, 22)
    title = ImageFont.truetype(font_path, 29)
    draw.text((30, 16), f"{MODEL_NAME} | articulated double-scissor model", fill="#202936", font=title)
    for i, (panel, label) in enumerate(zip(panels, (f"{MIN_HEIGHT*1000:.0f} mm | collapsed", "800 mm | intermediate", f"{MAX_HEIGHT*1000:.0f} mm | extended"))):
        canvas.paste(panel, (600*i, 58))
        draw.text((600*i+25, 788), label, fill="#202936", font=font)
    path = output / "scissor_lift_comparison.png"
    canvas.save(path)
    print(f"Preview: {path}")


if __name__ == "__main__":
    main()
