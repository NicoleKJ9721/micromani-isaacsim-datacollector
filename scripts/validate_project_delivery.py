#!/usr/bin/env python3
"""Validate the MicroMani Isaac Sim delivery without requiring Isaac Sim."""

from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "isaacsim_tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from micromani_lerobot_schema import (  # noqa: E402
    CAMERA_SPECS,
    CONTROL_ORDER,
    DEFAULT_HOME_ACTION,
    FORCE_TORQUE_FEATURE_KEY,
    FORCE_TORQUE_SENSORS,
    FORCE_TORQUE_VECTOR_NAMES,
    GRIPPER_MAP,
    HKVL36_DEFAULT_MODEL,
    JOINT_LIMITS,
    RAW_DOF_NAMES,
    build_lerobot_features,
    control_action_to_raw_targets,
    hkvl36_model_spec,
    quantize_hkvl36_wrench,
    raw_state_to_control_state,
)
from micromani_pico_mapping import PicoTeleopMapper  # noqa: E402


failures: list[str] = []


def check(condition: bool, message: str) -> None:
    if not condition:
        failures.append(message)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def validate_schema() -> None:
    check(len(CONTROL_ORDER) == 14, "effective control order must be 14D")
    check(len(RAW_DOF_NAMES) == 16, "raw actuated DOFs must be 16")
    check(set(GRIPPER_MAP) == {"left_gripper", "right_gripper"}, "gripper map must expose left/right logical axes")

    action = [0.0] * len(CONTROL_ORDER)
    action[CONTROL_ORDER.index("left_gripper")] = 0.007
    action[CONTROL_ORDER.index("right_gripper")] = 0.006
    raw = control_action_to_raw_targets(action)
    check(raw["joint_finger1_left"] == raw["joint_finger2_left"] == 0.007, "left gripper raw targets mismatch")
    check(raw["joint_finger1_right"] == raw["joint_finger2_right"] == 0.006, "right gripper raw targets mismatch")

    raw_state = {name: float(i) for i, name in enumerate(RAW_DOF_NAMES)}
    state = raw_state_to_control_state(raw_state)
    check(len(state) == 14, "raw_state_to_control_state must return 14 values")
    check(state[CONTROL_ORDER.index("left_gripper")] == 6.5, "left gripper state must average both fingers")
    check(state[CONTROL_ORDER.index("right_gripper")] == 14.5, "right gripper state must average both fingers")

    features = build_lerobot_features()
    check(features["observation.state"]["shape"] == (14,), "LeRobot state feature must be 14D")
    check(features["action"]["shape"] == (14,), "LeRobot action feature must be 14D")
    check(features[FORCE_TORQUE_FEATURE_KEY]["shape"] == (12,), "force/torque feature must be 12D")
    check(len(FORCE_TORQUE_VECTOR_NAMES) == 12, "force/torque vector order must be 12D")
    check(set(CAMERA_SPECS) == {"left_wrist", "right_wrist", "global"}, "three cameras must be defined")
    for name, spec in CAMERA_SPECS.items():
        check(tuple(spec["resolution"]) == (640, 480), f"{name} camera must be 640x480")
        check(int(spec["fps"]) == 30, f"{name} camera must be 30 FPS")


def validate_hkvl36() -> None:
    spec = hkvl36_model_spec(HKVL36_DEFAULT_MODEL)
    check(spec["force_range_n"] == 30.0, "HKVL-36A force range must be +/-30 N")
    check(spec["torque_range_nm"] == 1.0, "HKVL-36A torque range must be +/-1 Nm")
    check(len(FORCE_TORQUE_SENSORS) == 2, "two HKVL-36 sensors must be configured")
    check(
        FORCE_TORQUE_SENSORS["left_wrist"]["joint_name"] == "joint6_left",
        "left force/torque sensor must be attached at joint6_left",
    )
    check(
        FORCE_TORQUE_SENSORS["right_wrist"]["joint_name"] == "joint6_right",
        "right force/torque sensor must be attached at joint6_right",
    )
    values, saturated = quantize_hkvl36_wrench([30.5, -30.5, 0.0, 1.2, -1.2, 0.0])
    check(saturated, "HKVL-36 quantization must flag saturation")
    check(values == [30.0, -30.0, 0.0, 1.0, -1.0, 0.0], "HKVL-36 quantization must clip to full scale")


def validate_urdf_grippers() -> None:
    urdf_path = ROOT / "urdf" / "微纳机器人URDF.urdf"
    root = ET.parse(urdf_path).getroot()
    axes = {}
    limits = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name and "finger" in name:
            axis = joint.find("axis")
            limit = joint.find("limit")
            axes[name] = tuple(float(v) for v in axis.attrib["xyz"].split())
            limits[name] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))

    pairs = [
        ("joint_finger1_left", "joint_finger2_left"),
        ("joint_finger1_right", "joint_finger2_right"),
    ]
    for first, second in pairs:
        check(first in axes and second in axes, f"missing URDF gripper pair {first}/{second}")
        dot = sum(a * b for a, b in zip(axes[first], axes[second]))
        check(dot < -0.99, f"{first}/{second} axes must be opposite for relative gripper motion")
        check(limits[first] == (0.0, 0.01) and limits[second] == (0.0, 0.01), f"{first}/{second} limits mismatch")


def validate_extension() -> None:
    ext_dir = ROOT / "exts" / "micromani.data_collector"
    config = tomllib.loads((ext_dir / "config" / "extension.toml").read_text(encoding="utf-8"))
    check(config["package"]["version"] == "0.1.0", "extension version must be 0.1.0")
    check(config["python"]["module"][0]["name"] == "micromani.data_collector", "extension module mismatch")
    source = (ext_dir / "micromani" / "data_collector" / "extension.py").read_text(encoding="utf-8")
    for label in (
        "Open Scene",
        "Save Stage",
        "Configure Cameras",
        "Configure HKVL-36",
        "Start Bridge",
        "Stop Bridge",
        "Start PICO Gateway",
        "Stop PICO Gateway",
        "Run E2E Smoke",
        "Open Dataset Dir",
        "Open Docs",
        "Refresh Status",
    ):
        check(label in source, f"extension missing button: {label}")


def validate_docs() -> None:
    guide = (ROOT / "docs" / "MICROMANI_ISAACSIM_PLUGIN_AND_DATA_COLLECTION.md").read_text(encoding="utf-8")
    for phrase in (
        "14 维",
        "HKVL-36A",
        "observation.force_torque",
        "Tools > MicroMani > MicroMani Data Collector",
        "PICO-4",
        "当前限制",
        "Run E2E Smoke",
    ):
        check(phrase in guide, f"guide missing required phrase: {phrase}")

    check((ROOT / "pico4_unity" / "MicroManiPicoTeleop.cs").exists(), "PICO-4 Unity sender script missing")
    check((ROOT / "isaacsim_tools" / "micromani_pico_gateway.py").exists(), "PICO-4 gateway script missing")


def validate_pico_mapping() -> None:
    mapper = PicoTeleopMapper()
    axes = mapper.map_packet({"axes": list(DEFAULT_HOME_ACTION)})
    check(len(axes) == 14, "direct PICO axes packet must map to 14D")
    packet = {
        "left": {"primary2DAxis": {"x": 0.5, "y": 0.5}, "trigger": 0.8, "grip": 0.2},
        "right": {"primary2DAxis": {"x": -0.5, "y": 0.25}, "trigger": 0.1, "grip": 0.7},
    }
    axes = mapper.map_packet(packet)
    check(len(axes) == 14, "OpenXR-style PICO packet must map to 14D")
    for name, value in zip(CONTROL_ORDER, axes):
        lo, hi = JOINT_LIMITS[name]
        check(lo <= value <= hi, f"PICO mapped value out of range for {name}")


def validate_existing_dataset() -> None:
    dataset = ROOT / "datasets" / "lerobot" / "micromani_ft_e2e_20260703_01"
    if not dataset.exists():
        return
    info = load_json(dataset / "meta" / "info.json")
    validation = load_json(dataset / "micromani_validation.json")
    check(info["codebase_version"] == "v3.0", "verified dataset must report LeRobot v3.0")
    check(validation["num_frames"] == 6, "verified dataset must contain six smoke-test frames")
    check(validation["sample_state_shape"] == [14], "verified dataset state shape mismatch")
    check(validation["sample_action_shape"] == [14], "verified dataset action shape mismatch")
    check(validation["sample_force_torque_shape"] == [12], "verified dataset force/torque shape mismatch")
    check((dataset / "data" / "chunk-000" / "file-000.parquet").exists(), "verified dataset parquet shard missing")
    check((dataset / "force_torque" / "episode_000000.csv").exists(), "force/torque CSV sidecar missing")
    check((dataset / "force_torque" / "episode_000000.jsonl").exists(), "force/torque JSONL sidecar missing")


def main() -> int:
    validate_schema()
    validate_hkvl36()
    validate_urdf_grippers()
    validate_extension()
    validate_pico_mapping()
    validate_docs()
    validate_existing_dataset()
    if failures:
        print("Delivery validation failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1
    print("Delivery validation passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
