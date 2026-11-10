"""Choose physics precision from robot clearance and public finger contact only."""
import math

def select_physics_hz(finger_min_z, contacts, gripper_moving, precision_hz=960, free_hz=120, table_top=.2):
    values=[float(finger_min_z),*[float(v) for v in contacts]]
    if not values or not all(math.isfinite(v) for v in values):
        return precision_hz
    if gripper_moving or finger_min_z <= table_top+.015 or any(v>.0001 for v in contacts):
        return precision_hz
    return free_hz
