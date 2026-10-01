"""Gen3 7-DOF joint trajectory limits, SI units.

Kinova User-Guide-Gen3-R07.pdf, printed pp. 97-98, tables 39-44:
https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf
Acceleration uses the angular joystick/joint trajectory limits (table 43).
These limit the servo reference; physical joint dynamics remain in MuJoCo.
"""
import numpy as np

POSITION_LIMITS = np.deg2rad([np.inf, 128.9, np.inf, 147.8, np.inf, 120.3, np.inf])
SPEED_LIMITS = np.array([1.39]*4 + [1.22]*3)
ACCELERATION_LIMITS = np.array([1.0]*4 + [10.0]*3)
TORQUE_LIMITS = np.array([39.0]*4 + [9.0]*3)


class JointControl:
    def __init__(self, model, data):
        self.ids = [model.actuator(f'joint_{i}_actuator').id for i in range(1, 8)]
        self.command = data.ctrl[self.ids].copy()
        self.target = self.command.copy()
        self.velocity = np.zeros(7)

    def set_target(self, values):
        # Reject invalid inputs individually, preserving the last valid target.
        values = np.where(np.isfinite(values), values, self.target)
        self.target = np.clip(values, -POSITION_LIMITS, POSITION_LIMITS)

    def update(self, data, dt):
        error = self.target-self.command
        # Discrete braking envelope: v*dt + v^2/(2*a) <= distance.
        # Retargeting may cross the new target while braking, but does not
        # instantly reverse velocity or jump the position reference.
        a_dt = ACCELERATION_LIMITS*dt
        braking_speed = np.sqrt(a_dt*a_dt + 2*ACCELERATION_LIMITS*np.abs(error))-a_dt
        desired_velocity = np.sign(error)*np.minimum(SPEED_LIMITS, braking_speed)
        self.velocity += np.clip(desired_velocity-self.velocity, -a_dt, a_dt)
        self.command += self.velocity*dt
        data.ctrl[self.ids] = self.command
