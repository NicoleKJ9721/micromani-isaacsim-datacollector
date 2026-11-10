"""Shared schema for MicroMani Isaac Sim and LeRobot data collection."""

from __future__ import annotations

import math
from typing import Iterable, Mapping


ARTICULATION_ROOT = "/micromanipulator/root_joint"
STAGE_PATH = "/isaac-sim/.local/share/ov/data/micromani/micromanipulator_scene.usd"

CONTROL_ORDER = (
    "joint1_left",
    "joint2_left",
    "joint3_left",
    "joint4_left",
    "joint5_left",
    "joint6_left",
    "left_gripper",
    "joint1_right",
    "joint2_right",
    "joint3_right",
    "joint4_right",
    "joint5_right",
    "joint6_right",
    "right_gripper",
)

RAW_DOF_NAMES = (
    "joint1_left",
    "joint2_left",
    "joint3_left",
    "joint4_left",
    "joint5_left",
    "joint6_left",
    "joint_finger1_left",
    "joint_finger2_left",
    "joint1_right",
    "joint2_right",
    "joint3_right",
    "joint4_right",
    "joint5_right",
    "joint6_right",
    "joint_finger1_right",
    "joint_finger2_right",
)

GRIPPER_MAP = {
    "left_gripper": ("joint_finger1_left", "joint_finger2_left"),
    "right_gripper": ("joint_finger1_right", "joint_finger2_right"),
}

JOINT_LIMITS = {
    "joint1_left": (0.0, 0.075),
    "joint2_left": (0.0, 0.05),
    "joint3_left": (0.0, 0.075),
    "joint4_left": (-3.1416, 3.1416),
    "joint5_left": (-0.1396, 0.1396),
    "joint6_left": (-3.1416, 3.1416),
    "left_gripper": (0.0, 0.01),
    "joint1_right": (0.0, 0.075),
    "joint2_right": (0.0, 0.05),
    "joint3_right": (0.0, 0.075),
    "joint4_right": (-3.1416, 3.1416),
    "joint5_right": (-0.1396, 0.1396),
    "joint6_right": (-3.1416, 3.1416),
    "right_gripper": (0.0, 0.01),
}

# Retracted, staggered start: CAD cross-arm finger AABBs are >=17 mm apart.
DEFAULT_HOME_ACTION = (
    0.002, 0.005, 0.034, 0.0, 0.0, 0.0, 0.00225,
    0.002, 0.045, 0.034, 0.0, 0.0, 0.0, 0.001,
)

CAMERA_SPECS = {
    "left_wrist": {
        "prim_path": "/micromanipulator/link6_left/left_wrist_camera",
        "feature_key": "observation.images.left_wrist",
        "resolution": (640, 480),
        "fps": 30,
        "target_fov_m": (0.04, 0.03),
    },
    "right_wrist": {
        "prim_path": "/micromanipulator/link6_right/right_wrist_camera",
        "feature_key": "observation.images.right_wrist",
        "resolution": (640, 480),
        "fps": 30,
        "target_fov_m": (0.04, 0.03),
    },
    "global": {
        "prim_path": "/World/global_camera",
        "feature_key": "observation.images.global",
        "resolution": (640, 480),
        "fps": 30,
        "target_fov_m": (0.12, 0.09),
    },
}

FORCE_TORQUE_AXES = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
FORCE_TORQUE_FEATURE_KEY = "observation.force_torque"
FORCE_TORQUE_VECTOR_NAMES = tuple(
    f"{sensor}_{axis}"
    for sensor in ("left_wrist", "right_wrist")
    for axis in FORCE_TORQUE_AXES
)

HKVL36_DEFAULT_MODEL = "HKVL-36A"
HKVL36_MODELS = {
    "HKVL-36A": {
        "force_range_n": 30.0,
        "torque_range_nm": 1.0,
    },
    "HKVL-36B": {
        "force_range_n": 50.0,
        "torque_range_nm": 1.5,
    },
    "HKVL-36C": {
        "force_range_n": 80.0,
        "torque_range_nm": 1.5,
    },
}

HKVL36_COMMON = {
    "diameter_mm": 36.0,
    "height_mm": 21.0,
    "material": "aluminum alloy",
    "overload_percent_fs": 300.0,
    "repeatability_percent_fs": 0.1,
    "resolution_percent_fs": 0.03,
    "accuracy_percent_fs": 0.5,
    "mass_kg_upper_bound": 0.2,
    "spec_source": "https://www.hkvtech.cn/archives/125/",
    "spec_checked_date": "2026-11-09",
    "sample_frequency_hz_range": (150, 1000),
    "supply_voltage_vdc_range": (24, 24),
    "communication_interfaces": ("RS485", "CAN-FD", "EtherCAT"),
}

FORCE_TORQUE_SENSORS = {
    "left_wrist": {
        "model": HKVL36_DEFAULT_MODEL,
        "joint_name": "joint6_left",
        "joint_prim_path": "/micromanipulator/joints/joint6_left",
        "mount_link_path": "/micromanipulator/link6_left",
        "sensor_prim_path": "/micromanipulator/link6_left/HKVL36_FT_sensor",
        "mount": "joint6_left center",
        "frame": "child joint frame of joint6_left",
    },
    "right_wrist": {
        "model": HKVL36_DEFAULT_MODEL,
        "joint_name": "joint6_right",
        "joint_prim_path": "/micromanipulator/joints/joint6_right",
        "mount_link_path": "/micromanipulator/link6_right",
        "sensor_prim_path": "/micromanipulator/link6_right/HKVL36_FT_sensor",
        "mount": "joint6_right center",
        "frame": "child joint frame of joint6_right",
    },
}

DEFAULT_TASK = (
    "Use the bimanual micromanipulator to align both wrist views over the micro target "
    "and execute a synchronized open-close gripper motion."
)


def clip_control_action(action: Iterable[float]) -> list[float]:
    values = [float(v) for v in action]
    if len(values) != len(CONTROL_ORDER):
        raise ValueError(f"expected {len(CONTROL_ORDER)} control values, got {len(values)}")
    clipped = []
    for name, value in zip(CONTROL_ORDER, values):
        lo, hi = JOINT_LIMITS[name]
        clipped.append(min(max(value, lo), hi))
    return clipped


def control_action_to_raw_targets(action: Iterable[float]) -> dict[str, float]:
    targets = {}
    for name, value in zip(CONTROL_ORDER, clip_control_action(action)):
        if name in GRIPPER_MAP:
            for raw_name in GRIPPER_MAP[name]:
                targets[raw_name] = value
        else:
            targets[name] = value
    return targets


def raw_state_to_control_state(raw_state_by_name: Mapping[str, float]) -> list[float]:
    state = []
    for name in CONTROL_ORDER:
        if name in GRIPPER_MAP:
            raw_names = GRIPPER_MAP[name]
            state.append(sum(float(raw_state_by_name[r]) for r in raw_names) / len(raw_names))
        else:
            state.append(float(raw_state_by_name[name]))
    return state


def hkvl36_model_spec(model: str = HKVL36_DEFAULT_MODEL) -> dict:
    if model not in HKVL36_MODELS:
        raise ValueError(f"unknown HKVL-36 model: {model}")
    return {"model": model, **HKVL36_COMMON, **HKVL36_MODELS[model]}


def hkvl36_axis_ranges(model: str = HKVL36_DEFAULT_MODEL) -> list[float]:
    spec = hkvl36_model_spec(model)
    return [spec["force_range_n"]] * 3 + [spec["torque_range_nm"]] * 3


def quantize_hkvl36_wrench(wrench: Iterable[float], model: str = HKVL36_DEFAULT_MODEL) -> tuple[list[float], bool]:
    values = [float(v) for v in wrench]
    if len(values) != len(FORCE_TORQUE_AXES) or not all(map(math.isfinite, values)):
        raise ValueError(f"expected {len(FORCE_TORQUE_AXES)} finite wrench values")
    spec = hkvl36_model_spec(model)
    ranges = hkvl36_axis_ranges(model)
    resolution_scale = float(spec["resolution_percent_fs"]) / 100.0
    quantized = []
    saturated = False
    for value, full_scale in zip(values, ranges):
        if value < -full_scale or value > full_scale:
            saturated = True
        clipped = min(max(value, -full_scale), full_scale)
        step = full_scale * resolution_scale
        if step > 0 and abs(clipped) < full_scale:
            quantized.append(round(clipped / step) * step)
        else:
            quantized.append(clipped)
    return quantized, saturated


def force_torque_vector_from_readings(readings: Mapping[str, Mapping[str, Iterable[float]]]) -> list[float]:
    vector = []
    for sensor_name in ("left_wrist", "right_wrist"):
        sensor = readings[sensor_name]
        values = sensor.get("wrench")
        if values is None:
            values = sensor.get("values")
        if values is None:
            raise ValueError(f"missing wrench values for {sensor_name}")
        vector.extend(float(v) for v in values)
    if len(vector) != len(FORCE_TORQUE_VECTOR_NAMES):
        raise ValueError(f"expected {len(FORCE_TORQUE_VECTOR_NAMES)} force-torque values, got {len(vector)}")
    return vector


def build_lerobot_features(image_dtype: str = "image", include_force_torque: bool = True) -> dict:
    if image_dtype not in {"image", "video"}:
        raise ValueError("image_dtype must be 'image' or 'video'")
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(CONTROL_ORDER),),
            "names": list(CONTROL_ORDER),
        },
        "action": {
            "dtype": "float32",
            "shape": (len(CONTROL_ORDER),),
            "names": list(CONTROL_ORDER),
        },
    }
    if include_force_torque:
        features[FORCE_TORQUE_FEATURE_KEY] = {
            "dtype": "float32",
            "shape": (len(FORCE_TORQUE_VECTOR_NAMES),),
            "names": list(FORCE_TORQUE_VECTOR_NAMES),
        }
    for spec in CAMERA_SPECS.values():
        width, height = spec["resolution"]
        features[spec["feature_key"]] = {
            "dtype": image_dtype,
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        }
    return features
