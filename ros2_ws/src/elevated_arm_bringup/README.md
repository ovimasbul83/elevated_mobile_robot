# Elevated arm bringup

ROS 2 Lyrical starter package for the Kinova Gen3 7-DOF arm identified in this
repository's MuJoCo model. It uses Kinova's description, hardware plugin and
joint trajectory controller rather than duplicating the manufacturer's model.

## Build

From the repository root, in a terminal outside the project's Python virtualenv:

```bash
source /opt/ros/lyrical/setup.bash
cd ros2_ws
colcon build --packages-select elevated_arm_bringup
source install/setup.bash
```

The package builds independently, but launching requires `kortex_bringup` and
its runtime dependencies. Install a version of the official
[ros2_kortex driver](https://github.com/Kinovarobotics/ros2_kortex) compatible
with your ROS distribution and source its workspace before this workspace.
Lyrical compatibility of the driver has not yet been verified here.

## Mock hardware

```bash
ros2 launch elevated_arm_bringup arm.launch.py
```

Mock hardware mirrors joint commands into joint states; it is not a physics
simulation. This launch does not connect to the existing MuJoCo simulation.
The default has no gripper. Set `gripper:=robotiq_2f_85` or
`gripper:=robotiq_2f_140` only when it matches your arm.

## Physical arm

Use the arm's actual network address:

```bash
ros2 launch elevated_arm_bringup arm.launch.py mode:=real robot_ip:=192.168.1.10
```

The address above is an example, not a discovered lab address. Real mode requires
an explicit address. This package sends no automatic movement commands.
Verify the arm calibration, joint limits, clear workspace and emergency stop
before executing trajectories. Joint trajectories do not provide collision
planning; add MoveIt for planned motion.

## Shared command interface

Both modes select the driver's `joint_trajectory_controller`. After launch:

```bash
ros2 control list_controllers
ros2 action list -t
ros2 topic echo /joint_states --once
```

Use the advertised `control_msgs/action/FollowJointTrajectory` action from the
same application in either mode. Check the driver's actual joint names and
action namespace before sending commands. Controller tuning remains owned by
the driver; MuJoCo actuator gains are not copied onto hardware.

This initial package covers the arm only. The lift, mobile base, MoveIt planning,
and a MuJoCo ROS bridge are not implemented by this launch.
