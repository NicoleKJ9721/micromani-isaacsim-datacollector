#!/usr/bin/env python3
"""Configure HKVL-36 force/torque sensor metadata in the MicroMani USD scene."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from isaacsim import SimulationApp

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from micromani_lerobot_schema import (  # noqa: E402
    FORCE_TORQUE_AXES,
    FORCE_TORQUE_SENSORS,
    HKVL36_DEFAULT_MODEL,
    STAGE_PATH,
    hkvl36_axis_ranges,
    hkvl36_model_spec,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=STAGE_PATH)
    parser.add_argument("--model", default=HKVL36_DEFAULT_MODEL, choices=("HKVL-36A", "HKVL-36B", "HKVL-36C"))
    parser.add_argument("--sample-fps", type=int, default=30)
    parser.add_argument("--report", default=None)
    return parser.parse_args()


def set_sensor_metadata(prim, metadata: dict) -> None:
    for key, value in metadata.items():
        if not isinstance(value, (str, int, float, bool)):
            value = json.dumps(value, ensure_ascii=False)
        prim.SetCustomDataByKey(f"micromani_{key}", value)


def main() -> None:
    args = parse_args()
    kit = SimulationApp({"headless": True})

    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    context = omni.usd.get_context()
    open_ok = context.open_stage(args.stage)
    if open_ok is False:
        raise RuntimeError(f"failed to open stage: {args.stage}")
    for _ in range(30):
        kit.update()
    stage = context.get_stage()
    model = hkvl36_model_spec(args.model)
    axis_ranges = hkvl36_axis_ranges(args.model)
    configured = []

    for sensor_name, sensor_spec in FORCE_TORQUE_SENSORS.items():
        sensor_path = Sdf.Path(sensor_spec["sensor_prim_path"])
        sensor_xform = UsdGeom.Xform.Define(stage, sensor_path)
        sensor_prim = sensor_xform.GetPrim()
        sensor_xform.ClearXformOpOrder()
        sensor_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

        joint_prim = stage.GetPrimAtPath(sensor_spec["joint_prim_path"])
        if not joint_prim.IsValid():
            raise RuntimeError(f"missing joint prim for force-torque sensor: {sensor_spec['joint_prim_path']}")

        metadata = {
            "sensor_type": "six_axis_force_torque",
            "model": args.model,
            "vendor": "HKVTech",
            "install_location": sensor_spec["mount"],
            "joint_name": sensor_spec["joint_name"],
            "joint_prim_path": sensor_spec["joint_prim_path"],
            "mount_link_path": sensor_spec["mount_link_path"],
            "sensor_prim_path": sensor_spec["sensor_prim_path"],
            "data_source": "Articulation.get_measured_joint_forces(joint_index + 1)",
            "reference_frame": sensor_spec["frame"],
            "axes": list(FORCE_TORQUE_AXES),
            "axis_units": ["N", "N", "N", "N*m", "N*m", "N*m"],
            "axis_full_scale": axis_ranges,
            "sample_fps_configured": int(args.sample_fps),
            "hardware_spec": model,
        }
        set_sensor_metadata(sensor_prim, metadata)
        set_sensor_metadata(joint_prim, metadata)

        configured.append(
            {
                "name": sensor_name,
                "model": args.model,
                "joint_name": sensor_spec["joint_name"],
                "joint_prim_path": sensor_spec["joint_prim_path"],
                "sensor_prim_path": sensor_spec["sensor_prim_path"],
                "axis_full_scale": axis_ranges,
                "sample_fps_configured": int(args.sample_fps),
                "reference_frame": sensor_spec["frame"],
            }
        )

    stage.GetRootLayer().Save()
    report = {
        "stage": args.stage,
        "model": args.model,
        "axes": list(FORCE_TORQUE_AXES),
        "hardware_spec": model,
        "configured_sensors": configured,
    }
    report_path = Path(args.report) if args.report else Path(args.stage).with_name("hkvl36_force_torque_sensors.json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    kit.close()


if __name__ == "__main__":
    main()
