"""Arbitrate native MuJoCo Control sliders and keyboard/CLI commands."""
import numpy as np
from build_scissor_lift import MIN_HEIGHT, STROKE, NO_LOAD_SPEED
from joint_control import JointControl
from wheel_control import WheelControl


class ViewerControl:
    def __init__(self, model, data, target_height, speed=.10, turn_speed=.25, motion='stop'):
        self.lift_id = model.actuator('lift_motor').id
        self.lift_target = float(np.clip(target_height-MIN_HEIGHT, 0, STROKE))
        self.lift_command = float(data.ctrl[self.lift_id])
        self.arm = JointControl(model, data)
        self.drive = WheelControl(model, speed, turn_speed)
        self.drive.set_motion(motion)
        self.wheel_ids = list(self.drive.motors.values())
        self.wheels_from_panel = motion == 'stop'
        self.drive_revision = self.drive.revision

    def before_sync(self, data):
        """Show requested destinations in the panel, not moving servo references.

        Pair with after_sync before any physics step, even if sync raises.
        """
        self.applied_ctrl = data.ctrl.copy()
        data.ctrl[self.lift_id] = self.lift_target
        data.ctrl[self.arm.ids] = self.arm.target
        return data.ctrl.copy()

    def after_sync(self, previous_ctrl, data):
        """Capture panel edits, then restore rate-limited physics commands."""
        changed = ~np.isclose(previous_ctrl, data.ctrl, rtol=0, atol=1e-12)
        if changed[self.lift_id] and np.isfinite(data.ctrl[self.lift_id]):
            self.lift_target = float(np.clip(data.ctrl[self.lift_id], 0, STROKE))
        self.arm.set_target(data.ctrl[self.arm.ids])
        data.ctrl[self.lift_id] = self.applied_ctrl[self.lift_id]
        data.ctrl[self.arm.ids] = self.applied_ctrl[self.arm.ids]
        if changed[self.wheel_ids].any():
            self.wheels_from_panel = True
            with self.drive.lock:
                self.drive_revision = self.drive.revision
                # Seed keyboard ramps from the current panel wheel commands.
                fl, fr, bl, br = data.ctrl[self.wheel_ids]
                self.drive.linear = self.drive.radius*(fl+fr+bl+br)/4
                self.drive.lateral = self.drive.radius*(-fl+fr+bl-br)/4
                self.drive.yaw = self.drive.radius*(-fl+fr-bl+br)/(4*(self.drive.half_wheelbase+self.drive.half_track))

    def update(self, data, dt):
        limit = NO_LOAD_SPEED*dt
        self.lift_command += max(-limit, min(limit, self.lift_target-self.lift_command))
        data.ctrl[self.lift_id] = self.lift_command
        self.arm.update(data, dt)
        with self.drive.lock:
            revision = self.drive.revision
        if revision != self.drive_revision:
            self.wheels_from_panel = False
            self.drive_revision = revision
        if not self.wheels_from_panel:
            self.drive.update(data, dt)
