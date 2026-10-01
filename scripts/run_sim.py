import argparse
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
from lift_kinematics import set_lift_height
from build_scissor_lift import MODEL_NAME, MIN_HEIGHT, MAX_HEIGHT, NO_LOAD_SPEED
from viewer_control import ViewerControl


ROOT = Path(__file__).resolve().parent.parent

MODEL_PATH = ROOT / "models" / "scene.xml"


model = mujoco.MjModel.from_xml_path(
    str(MODEL_PATH)
)

data = mujoco.MjData(model)


print("Model loaded successfully")
print("Number of bodies:", model.nbody)
print("Number of joints:", model.njnt)
print("Number of DOFs:", model.nv)
print("Number of actuators:", model.nu)


lift_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_ACTUATOR,
    "lift_motor"
)


parser = argparse.ArgumentParser(description=f"{MODEL_NAME} double-scissor mobile manipulator")
parser.add_argument("--start-height", type=float, default=MIN_HEIGHT,
                    help=f"Initial lift height in metres ({MIN_HEIGHT:.3f} to {MAX_HEIGHT:.3f})")
parser.add_argument("--height", type=float, default=None,
                    help="Target lift height; defaults to the starting height")
parser.add_argument("--drive", choices=['stop', 'forward', 'backward', 'left', 'right', 'strafe-left', 'strafe-right'], default='stop',
                    help="Initial base motion; W/S drive, A/D turn, Q/E strafe, X stops")
parser.add_argument("--drive-speed", type=float, default=.10, help="Forward/reverse/strafe speed in m/s (0 to 0.3)")
parser.add_argument("--turn-speed", type=float, default=.25, help="Turning command in rad/s (0 to 0.6)")
args = parser.parse_args()
target = args.start_height if args.height is None else args.height
if not MIN_HEIGHT <= target <= MAX_HEIGHT or not MIN_HEIGHT <= args.start_height <= MAX_HEIGHT:
    parser.error(f"Lift heights must be between {MIN_HEIGHT:.3f} and {MAX_HEIGHT:.3f} metres")
if not math.isfinite(args.drive_speed) or not 0 <= args.drive_speed <= .3:
    parser.error("Drive speed must be between 0 and 0.3 m/s")
if not math.isfinite(args.turn_speed) or not 0 <= args.turn_speed <= .6:
    parser.error("Turn speed must be between 0 and 0.6 rad/s")
set_lift_height(model, data, args.start_height)
print(f"Lift: {args.start_height:.3f} m -> {target:.3f} m; command ramp: {NO_LOAD_SPEED*1000:g} mm/s")
controls = ViewerControl(model, data, target, args.drive_speed, args.turn_speed, args.drive)
print("Click the simulation window: W forward | S reverse | A/D turn | Q/E slide left/right | X stop")
print("Tap a key to keep moving; tap X to stop. Directions are relative to the robot.")
print("Lift/arm sliders select targets with speed limits; Kinova also has acceleration and torque limits.")
print("A movement key returns wheel control to the keyboard.")
print("lift_motor = height minus 0.265 m (e.g. 0.535 means 0.800 m); wheel motors use rad/s.")


with mujoco.viewer.launch_passive(
    model,
    data,
    key_callback=controls.drive.key_callback,
    show_right_ui=True
) as viewer:

    viewer.cam.lookat[:] = [0, 0, .8]
    viewer.cam.distance = 2.6
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -18

    # Batch physics steps and sync the UI around 60 Hz, rather than sleeping and
    # rebuilding the viewer at every 1 ms physics step.
    steps_per_frame = max(1, round((1/60)/model.opt.timestep))
    frame_duration = steps_per_frame*model.opt.timestep
    while viewer.is_running():

        step_start = time.perf_counter()

        for _ in range(steps_per_frame):
            controls.update(data, model.opt.timestep)
            mujoco.mj_step(model, data)
        previous_ctrl = controls.before_sync(data)
        try:
            viewer.sync()
        finally:
            controls.after_sync(previous_ctrl, data)

        remaining = (
            frame_duration
            - (time.perf_counter() - step_start)
        )

        if remaining > 0:
            time.sleep(remaining)
