"""Consistent initialization of every joint in the closed scissor mechanism."""
import math
import mujoco
from build_scissor_lift import MIN_HEIGHT, MAX_HEIGHT, AXLE_Z, TOP_OFFSET, LENGTH, THETA, SPAN, HALF_RISE


def set_lift_height(model, data, height):
    """Set an initial pose; use the lift_motor control for subsequent motion."""
    if not MIN_HEIGHT <= height <= MAX_HEIGHT:
        raise ValueError(f"Height must be between {MIN_HEIGHT} and {MAX_HEIGHT} metres")
    rise = (height - AXLE_Z - TOP_OFFSET) / 2
    theta = math.asin(rise / LENGTH)
    span = LENGTH * math.cos(theta)
    delta = theta - THETA
    angle = math.atan2(1.05*rise, .55*span)
    initial_angle = math.atan2(1.05*HALF_RISE, .55*SPAN)
    positions = {
        "lift_joint": height-MIN_HEIGHT, "base_slider": SPAN-span,
        "lower_a_hinge": delta, "lower_b_hinge": delta,
        "upper_a_hinge": -2*delta, "upper_b_hinge": -2*delta,
        "actuator_pitch": delta-angle+initial_angle,
        "actuator_extension": math.hypot(.55*span, 1.05*rise)-math.hypot(.55*SPAN, 1.05*HALF_RISE),
    }
    for name, value in positions.items():
        joint = model.joint(name)
        data.qpos[joint.qposadr[0]] = value
        data.qvel[joint.dofadr[0]] = 0
    data.ctrl[model.actuator("lift_motor").id] = height-MIN_HEIGHT
    mujoco.mj_forward(model, data)
