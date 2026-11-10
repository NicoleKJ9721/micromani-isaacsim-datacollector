"""Gravity calibration from public free-air wrench and robot orientation samples.

Reaction wrench convention: supported weight points upward in world. Rotations
map sensor coordinates to world. Never fit to contact or accelerating samples.
This removes a gravity model, not sensor noise, friction, or inertial forces.
"""
import numpy as np


def _cross(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def fit_gravity(rotations, wrenches):
    rotations = np.asarray(rotations, dtype=float)
    wrenches = np.asarray(wrenches, dtype=float)
    if rotations.shape != (len(wrenches), 3, 3) or wrenches.shape != (len(rotations), 6) or len(rotations) < 3:
        raise ValueError('At least three paired orientation/wrench samples required')
    if not np.isfinite(rotations).all() or not np.isfinite(wrenches).all():
        raise ValueError('Nonfinite calibration sample')
    if not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-5) or not np.allclose(np.linalg.det(rotations), 1, atol=1e-5):
        raise ValueError('Sensor orientations must be proper rotations')
    gravity = rotations.transpose(0, 2, 1) @ np.array([0., 0., 9.81])
    mass = float(np.sum(gravity * wrenches[:, :3]) / np.sum(gravity**2))
    if mass <= 0:
        raise ValueError('Wrench sign convention inconsistent with supported weight')
    matrix = np.vstack([-_cross(mass * g) for g in gravity])
    condition = float(np.linalg.cond(matrix))
    if condition > 100:
        raise ValueError('Insufficient orientation excitation for COM calibration')
    com = np.linalg.lstsq(matrix, wrenches[:, 3:].reshape(-1), rcond=1e-4)[0]
    return {'mass_kg': mass, 'com_sensor_frame_m': com.tolist(), 'condition_number': condition}


def expected_gravity(model, sensor_to_world):
    rotation = np.asarray(sensor_to_world, dtype=float)
    force = rotation.T @ np.array([0., 0., 9.81 * model['mass_kg']])
    return np.r_[force, np.cross(model['com_sensor_frame_m'], force)]


def compensated_wrench(model, sensor_to_world, measured):
    return np.asarray(measured, dtype=float) - expected_gravity(model, sensor_to_world)
