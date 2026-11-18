"""Uncalibrated passive PTFE retention approximation, authored once at reset.

This module never reads/writes runtime state or follows a target angle. The
bounded velocity-only drive dissipates energy; it does not model exact static
friction or establish physical calibration. See PASSIVE-RETENTION.md.
"""
import math

# Author-selected engineering approximation, not manufacturer measurements.
DAMPING_NMS_RAD = .020
TORQUE_CAP_NM = .010


def torque_nm(angular_velocity_rad_s):
    """Reference algebra for CPU audits; native PhysX applies the actual drive."""
    velocity = float(angular_velocity_rad_s)
    if not math.isfinite(velocity):
        raise ValueError('nonfinite_angular_velocity')
    return -max(-TORQUE_CAP_NM, min(TORQUE_CAP_NM, DAMPING_NMS_RAD*velocity))


def usd_damping():
    """USD angular drives use torque seconds/degree, not torque seconds/radian."""
    return DAMPING_NMS_RAD * math.pi / 180


def author_retention(joint):
    """Author a finite force-type angular drag; no positional spring or motor."""
    from pxr import UsdPhysics
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'angular')
    drive.CreateTypeAttr('force')
    drive.CreateStiffnessAttr(0.)
    drive.CreateDampingAttr(usd_damping())
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateMaxForceAttr(TORQUE_CAP_NM)
    # No target-position authoring. The fallback position is inert at k=0.
    return drive
