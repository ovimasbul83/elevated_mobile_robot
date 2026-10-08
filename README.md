# DHLCT-C2-60 double-scissor lift

The mobile manipulator uses an articulated DHLCT-C2-60 double-scissor lift with two paired stages, pivot pins, bottom and top rollers, two end mounting crossbars, four lower stop posts, and a telescoping electric-actuator assembly. A full-width aluminium upper platform replaces the small robot adapter. The mounting bars touch the upper rails without the previous 10 mm gap, the platform sits on the bars, and the robot is mounted directly on the platform's upper face.

The custom platform is **760 × 420 × 5 mm**, using the user-specified thickness of **0.5 cm**. Using an assumed aluminium density of 2700 kg/m³ gives an added mass of **4.3092 kg**, with box inertia and collision geometry included in the simulation. The platform is separate from the manufacturer's 20 kg lift mass, giving **24.3092 kg for lift plus platform**, before the robot. Its upper surface and robot mount are 5 mm above the nominal lift mounting surface (270–1275 mm above the lift base). Change `PLATFORM_THICKNESS` in `scripts/build_scissor_lift.py` and regenerate to use a different plate thickness.

## Manufacturer specifications and drawing

The user identified the actual lift as **DHLCT-C2-60**. The [manufacturer's French C2-60 table](https://dihool.com/lang_fr/Lifting-Column-DHLCT-C2-60-Lift_detail/4351_340) and [Italian table](https://www.dihool.com/lang_it/detail_4353_340) agree on the values below. The English page retains a C2-60 title but currently contains a DHLCT2-90A table; that conflicting table was not used for this revision. The [C2-60 dimension drawing](https://dihool.com/ueditor/php/upload/image/20240420/1713591796671178.jpg) is saved locally as `outputs/DHLCT-C2-60_dimensions.jpg`.

| Property | Model value |
| --- | --- |
| Mobile base including all four wheels (excluding lift/platform/robot) | 15 kg |
| Steel frame footprint (drawing; modeled) | 760 × 420 mm |
| Catalog overall envelope | 760 × 435 × 265 mm |
| Collapsed / extended mounting surface height | 265 / 1270 mm above lift base |
| Vertical travel | 1005 mm |
| Lift mass, excluding robot, custom platform, and mobile chassis | 20 kg |
| Custom aluminium platform | 760 × 420 × 5 mm; 4.3092 kg additional mass |
| End mounting-hole longitudinal spacing | 715 mm |
| End mounting-hole transverse spacing / diameter | 389 mm / 11 mm |
| Top crossbar section | 40 × 20 mm |
| Platform command ramp | 16 mm/s |
| Manufacturer nominal payload | 60 kg; a rating, not a simulated payload added to the model |

The C2-60 drawing explicitly dimensions the frame width as 420 mm, whereas the catalog table gives 435 mm. The reason for the extra 15 mm is not specified; no connector shape or width expansion has been invented. The model follows the dimensioned steel frame. Height is measured from the lift's bottom, not from the ground under the mobile chassis.

## Run

### Setup on Windows, Linux, and macOS

Use Python 3.12 or newer. From the project folder, create a virtual environment:

```sh
python -m venv .venv
```

On Linux/macOS, use `python3` if `python` is unavailable. Activate the environment for your shell:

| Shell | Activation command |
| --- | --- |
| Windows PowerShell | `.\.venv\Scripts\Activate.ps1` |
| Windows Git Bash | `source .venv/Scripts/activate` |
| Linux/macOS bash or zsh | `source .venv/bin/activate` |

Then install the runtime dependencies and launch:

```sh
python -m pip install -r requirements.txt
python scripts/launch_sim.py --start-height 0.8
```

The launcher forwards all simulation options and uses the active Python environment. On macOS it selects `mjpython`, which [MuJoCo requires for its passive viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer). Windows and Linux use the environment's Python directly. The interactive viewer needs a graphical desktop and working OpenGL drivers. Model and output paths resolve relative to the scripts, so an absolute script path also works from another directory. Run Python scripts through Python; invoking `scripts/run_sim.py` directly in a shell can interpret it as shell code.

`requirements.txt` contains the tested simulation and preview dependencies. Optional wheel-mesh regeneration dependencies are in `requirements-cad.txt`:

```sh
python -m pip install -r requirements-cad.txt
python scripts/build_wheels.py
```

Importing a new SolidWorks assembly through `scripts/inspect_wheel_cad.ps1` still requires Windows and Autodesk Inventor. Supply your own assembly path with `-SourceAssembly`; the script no longer assumes a particular user's Downloads folder. For example, in PowerShell:

```powershell
.\scripts\inspect_wheel_cad.ps1 -SourceAssembly "C:\CAD\Wheel Assembly.SLDASM"
```

Keep the assembly's companion part files available for Inventor to resolve. The checked-in exported meshes let the simulation run without Inventor. One-off report authoring utilities under `tmp/pdfs` are separate from the simulation setup.

### Imported mecanum wheel assemblies

The four placeholder cylinders have been visually replaced with the supplied `002-21 -- _Wheel Assembly, Right.SLDASM` geometry. Its companion files were found in the existing `Mobile Robotic Base with Mecanum Wheels.zip` in Downloads; the assembly in that archive matched the supplied file byte-for-byte. Autodesk Inventor imported 34 component instances and exported 12 distinct part meshes. The wheel, adapter, and wheel fasteners rotate together; the motor and mounting hardware remain fixed to the chassis.

The exported wheel geometry measures approximately **152.46 mm outside diameter and 56 mm width**, despite “165 mm” appearing in the part filename. Dimensions were measured from the original tessellated geometry before reduction. Wheel meshes were reduced from about 2.84 million to 250,000 triangles per wheel. The right-front module uses the supplied orientation; reflections across the vehicle centreline and rear axle create diagonal left/right handedness for the four-wheel arrangement confirmed by the user. The mirrored variants are derived geometry, not separately supplied left-hand CAD. Black rollers and silver hubs are cosmetic color assignments.

Wheel module mounts remain at x = ±304 mm and y = ±215 mm relative to the chassis. The CAD wheel offset places wheel centres at y = ±263.57332 mm. Chassis centre height is now 106.22993 mm so the larger wheels initially touch the floor.

**Mobile base mass:** the user-specified total is **15 kg including all four wheels**, excluding the lift, aluminium platform, and robot arm. The model assigns **11 kg to the chassis and fixed hardware together**, plus **1 kg to each of four rotating wheels**. The 11 kg is represented by the chassis box, so its inertia is recalculated automatically. Motor/mount meshes add no separate mass; their contribution is included in the chassis allocation. The component split and centre of mass remain approximations even though the base total is exactly 15 kg.

**Mecanum contact model:** each wheel now has **12 passive rollers at 45 degrees**, implemented as hidden ellipsoid contact shapes with free hinge joints. The old cylinder envelope has contact disabled. Ground friction and roller rotation generate forward, sideways, and turning motion; the controller does not overwrite the base pose or apply artificial lateral body forces. Roller sizes, count, friction, and bearing damping are modeling approximations rather than extracted manufacturer specifications. The CAD visual shell is still attached to the wheel hub; individual passive spin is represented by the hidden roller bodies. Each wheel retains a total of **1 kg**, split into a 0.76 kg hub and twelve 0.02 kg rollers, preserving the 15 kg mobile-base total. Imported CAD mass estimates were not substituted because their material assignments have not been validated. This is an approximate mecanum dynamics model, not a hardware-calibrated one.

`scripts/mecanum_geometry.py` generates these roller contacts in `models/wheels.xml`; `scripts/build_wheels.py` also applies them when rebuilding the CAD assets. The roller physics is native MuJoCo XML and requires no runtime contact callback.

Run `scripts/check_wheels.py` with the project Python to check all four ground contacts, fixed-versus-rotating parts, and stability. It saves `outputs/robot_cad_wheels.png` and `outputs/wheel_cad_detail.png`. Runtime assets are in `models/wheel_assets.xml`, `models/wheels.xml`, and `models/assets/meshes/wheels/`; Inventor is not needed to run the simulation. Intermediate CAD copies/exports are under `outputs/wheel_cad`. Rebuilding wheel assets uses `scripts/build_wheels.py` with NumPy and fast-simplification from `requirements-cad.txt`; the original `.cad_deps` bundle remains a fallback.

### Start the simulation

### Native Control panel

The MuJoCo viewer's **Control** panel controls the lift, wheels, and arm. Lift/arm sliders show requested destinations; a separate reference approaches each destination at the limits below. Slider targets persist while the robot moves.

- `lift_motor`: lift travel in metres, from 0 to 1.005. Requested lift height is **0.265 + slider value**; use **0.535 for 0.800 m**. This is the lift mounting-surface height; the aluminium plate adds 5 mm. A panel edit replaces the destination of any `--height` command and retains the **16 mm/s** reference limit in both directions. Large slider jumps cannot bypass that limit.
- `wheel_fl_motor`, `wheel_fr_motor`, `wheel_bl_motor`, `wheel_br_motor`: wheel angular speed in rad/s. Zero all four to stop, or press X in the viewer. Editing any wheel slider releases all four wheels from the keyboard mixer so independent commands persist.
- Pressing W/S/A/D/Q/E/X returns wheel control to the keyboard. Lift and arm slider control remain independent. `joint_1_actuator` through `joint_7_actuator` request arm joint angles in radians with position, reference speed/acceleration, and actuator torque limits.

For equal wheel-speed magnitude `s` (try `1` rad/s), the wheel slider patterns are:

| Motion | Front left | Front right | Back left | Back right |
| --- | --- | --- | --- | --- |
| Forward | +s | +s | +s | +s |
| Reverse | -s | -s | -s | -s |
| Strafe left | -s | +s | +s | -s |
| Strafe right | +s | -s | -s | +s |
| Turn left | -s | +s | -s | +s |
| Turn right | +s | -s | +s | -s |

Wheel panel values remain direct wheel velocity commands; the keyboard acceleration ramps apply while the keyboard has control. Lift and arm reference limits apply throughout `run_sim.py`, including panel control. `scripts/check_panel_controls.py` verifies slider persistence, lift/wheel response, stopping, and transfer between keyboard and panel. The runner batches physics at the model timestep and synchronizes the viewer around 60 times per second to avoid a redraw and operating-system sleep for every 1 ms step.

### Kinova Gen3 7-DOF limits

Values are from the official [Kinova Gen3 User Guide R07, printed pages 97-98, tables 39-44](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf). The arm uses angular joint trajectory acceleration limits, rather than the higher large-actuator acceleration hard limit.

| Joint | Position range | Reference speed limit | Reference acceleration limit | Actuator torque limit |
| --- | --- | --- | --- | --- |
| 1 | Continuous | 1.39 rad/s | 1 rad/s² | ±39 Nm |
| 2 | ±128.9° | 1.39 rad/s | 1 rad/s² | ±39 Nm |
| 3 | Continuous | 1.39 rad/s | 1 rad/s² | ±39 Nm |
| 4 | ±147.8° | 1.39 rad/s | 1 rad/s² | ±39 Nm |
| 5 | Continuous | 1.22 rad/s | 10 rad/s² | ±9 Nm |
| 6 | ±120.3° | 1.22 rad/s | 10 rad/s² | ±9 Nm |
| 7 | Continuous | 1.22 rad/s | 10 rad/s² | ±9 Nm |

Joint and actuator position ranges now agree exactly. The actuator force ranges use Kinova's robot torque soft limits, replacing the previous 105/52 Nm values. These torque and finite position limits are in the XML. Speed/acceleration limiting is in the runner's controller; loading the XML in a different application does not install that controller. Sudden target changes decelerate/reverse the reference smoothly. Non-finite arm/lift inputs retain the last valid target.

These are servo reference limits and actuator torque caps, not a reproduction of Kortex firmware or a collision-free trajectory planner. Actual joint velocity and acceleration also depend on contact, gravity, and tracking error; MuJoCo retains those dynamics without clipping or teleporting joint state. `python scripts/check_motion_limits.py` checks full lift-stroke timing, large panel jumps, arm reference limits during reversals, finite angle bounds, torque caps, and a coupled dynamics motion. The tested motion stays below the joint speed ratings; it does not certify every pose or external load.

### Launch commands

On any supported OS with `.venv` activated:

```bash
python scripts/launch_sim.py --start-height 0.8
```

Click the simulation window to focus it, then tap **W** forward, **S** reverse, **A** turn left, **D** turn right, **Q** strafe left, **E** strafe right, or **X** stop. Commands continue after releasing the key until another direction or X is pressed. Stop ramps down smoothly. Default commands are 0.10 m/s forward/reverse/sideways and 0.25 rad/s turning, with acceleration ramps. Motion is relative to the robot's heading, not the camera.

To begin moving without keyboard input, use `python scripts/run_sim.py --start-height 0.8 --drive strafe-left`. Optional `--drive-speed 0.15` and `--turn-speed 0.3` set the commands. Supported startup motions are `stop`, `forward`, `backward`, `left` (turn), `right` (turn), `strafe-left`, and `strafe-right`. The four wheel velocity servos have gain 10 and ±12 Nm torque limits; these are simulation tuning values, not validated AK80-9 specifications. `python scripts/check_drive.py` checks forward/reverse displacement, both turns, left/right strafing with low drift, stopping in all six directions, and body-relative strafing after a 90-degree rotation at a lift height of 0.8 m.

From this project folder in PowerShell:

```powershell
.\.venv\Scripts\python.exe scripts/run_sim.py --start-height 0.8
```

To raise from the collapsed state:

```powershell
.\.venv\Scripts\python.exe scripts/run_sim.py --height 1.27
```

`--start-height` sets a consistent initial pose of all joints. `--height` sets a target; the command approaches it at 16 mm/s in simulation time. With no arguments, the lift starts collapsed. A full commanded stroke takes 62.8125 simulation seconds. The position servo has some load-dependent tracking error; its target speed is not a validated prediction of the real actuator's loaded speed.

## Fidelity and assumptions

This is a reconstruction from the manufacturer's C2-60 drawing, not an exact manufacturer CAD model. Pivot spacing (660 mm when collapsed), arm sections, rail profiles, actuator attachment points, actuator dimensions, and individual inertias are estimated. The listed `DHLCT-60KG.STEP` download has not been verified as matching this exact model, so it was not used. Mounting holes are visual marks rather than drilled solid geometry; fasteners, wiring, controller, and internal motor gearing are simplified or omitted.

The estimated mass allocation is 4.5 kg lower frame, 3.5 kg top frame, 10 kg arms/carriage, and 2 kg electric actuator, totaling 20 kg. Four MuJoCo connect constraints close the two scissor crosses, upper fixed pivot, and actuator pin. The opposite upper rollers follow the rail through the closed geometry. The platform remains vertically guided by `lift_joint`.

The existing `lift_motor` interface is preserved as a **platform-space position drive**. The rendered actuator telescopes passively with the mechanism. Its real thrust, gearing, current, actuator stroke, and load curve are unverified; electrical and duty-cycle behavior are not modeled. This simulation does not certify the 60 kg rating. Gains and the ±2000 N platform force limit are simulation settings, not the rated payload. The frame has collision geometry; internal arms, rollers, and actuator have self-contact disabled. Use matching CAD, measured inertias, and actuator specifications for quantitative hardware predictions.

## Check and preview

```powershell
.\.venv\Scripts\python.exe scripts/check_scissor_lift.py --render
```

Checks 20 kg lift mass plus the separate platform mass, platform dimensions, gap-free rail/bar/plate interfaces, flush robot mounting, frame footprint, mounting-bar count and spacing, joint and command travel limits, geometric joint closure and mounting-surface height at five positions, three seconds of dynamics at each position, and five-second raising/lowering runs with the attached platform, robot, and mobile chassis. The script writes `outputs/scissor_lift_comparison.png` with the platform visible and the robot/chassis hidden, plus `outputs/upper_platform_detail.png` showing the assembled platform and robot base.

To regenerate the lift XML after changing geometry parameters:

```powershell
.\.venv\Scripts\python.exe scripts/build_scissor_lift.py
```

Model structure follows the [MuJoCo connect-constraint documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-connect).
