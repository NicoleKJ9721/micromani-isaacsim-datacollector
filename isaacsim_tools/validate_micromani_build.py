#!/usr/bin/env python3
"""Validate that the MicroMani USD is a live, controllable Isaac Sim robot build."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from micromani_collect_isaac import (  # noqa: E402
    apply_control_action,
    capture_rgb,
    get_control_state,
    get_dof_names,
    read_force_torque,
    write_png,
)
from micromani_lerobot_schema import (  # noqa: E402
    ARTICULATION_ROOT,
    CAMERA_SPECS,
    CONTROL_ORDER,
    DEFAULT_HOME_ACTION,
    FORCE_TORQUE_SENSORS,
    GRIPPER_MAP,
    RAW_DOF_NAMES,
    STAGE_PATH,
    clip_control_action,
    control_action_to_raw_targets,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=STAGE_PATH)
    parser.add_argument("--report", default="/isaac-sim/.local/share/ov/data/micromani/micromani_build_validation.json")
    parser.add_argument("--image-dir", default="/isaac-sim/.local/share/ov/data/micromani/build_validation_images")
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--gripper-settle-frames", type=int, default=120)
    return parser.parse_args()


def finite_list(values: list[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def summarize_stage(stage) -> dict:
    from pxr import UsdGeom

    joints = []
    collisions = []
    meshes = []
    cameras = []
    mimic = []
    for prim in stage.Traverse():
        type_name = prim.GetTypeName()
        path = str(prim.GetPath())
        if type_name.startswith("Physics") and type_name.endswith("Joint"):
            joints.append({"path": path, "type": type_name})
        if type_name == "Mesh" or prim.IsA(UsdGeom.Mesh):
            meshes.append(path)
        if type_name == "Camera":
            cameras.append(path)
        schemas = list(prim.GetAppliedSchemas())
        if "PhysicsCollisionAPI" in schemas or "CollisionAPI" in schemas:
            collisions.append(path)
        if any("Mimic" in schema for schema in schemas):
            mimic.append(path)
    return {
        "root_layer": stage.GetRootLayer().identifier,
        "sub_layers": list(stage.GetRootLayer().subLayerPaths),
        "joint_prim_count": len(joints),
        "joint_prims": joints,
        "collision_prim_count": len(collisions),
        "collision_prims": collisions,
        "mesh_prim_count": len(meshes),
        "camera_prim_count": len(cameras),
        "camera_prims": cameras,
        "mimic_prim_count": len(mimic),
        "mimic_prims": mimic,
    }


def camera_metadata(stage) -> dict:
    report = {}
    for name, spec in CAMERA_SPECS.items():
        prim = stage.GetPrimAtPath(spec["prim_path"])
        report[name] = {
            "prim_path": spec["prim_path"],
            "exists": prim.IsValid(),
            "resolution_expected": list(spec["resolution"]),
            "fps_expected": spec["fps"],
            "target_fov_m": list(spec["target_fov_m"]),
            "resolution_width_metadata": prim.GetCustomDataByKey("micromani_resolution_width") if prim else None,
            "resolution_height_metadata": prim.GetCustomDataByKey("micromani_resolution_height") if prim else None,
            "fps_metadata": prim.GetCustomDataByKey("micromani_fps") if prim else None,
        }
    return report


def sensor_metadata(stage) -> dict:
    report = {}
    for name, spec in FORCE_TORQUE_SENSORS.items():
        sensor_prim = stage.GetPrimAtPath(spec["sensor_prim_path"])
        joint_prim = stage.GetPrimAtPath(spec["joint_prim_path"])
        report[name] = {
            "sensor_prim_path": spec["sensor_prim_path"],
            "sensor_exists": sensor_prim.IsValid(),
            "joint_prim_path": spec["joint_prim_path"],
            "joint_exists": joint_prim.IsValid(),
            "model_metadata": sensor_prim.GetCustomDataByKey("micromani_model") if sensor_prim else None,
            "axis_full_scale_metadata": sensor_prim.GetCustomDataByKey("micromani_axis_full_scale")
            if sensor_prim
            else None,
        }
    return report


def raw_positions_by_name(robot, dof_names: list[str]) -> dict[str, float]:
    import numpy as np

    positions = np.asarray(robot.get_joint_positions(), dtype=float).reshape(-1)
    return {name: float(positions[i]) for i, name in enumerate(dof_names)}


def validate_gripper_mapping(robot, dof_index: dict[str, int], dof_names: list[str], world, settle_frames: int) -> dict:
    closed = list(DEFAULT_HOME_ACTION)
    closed[CONTROL_ORDER.index("left_gripper")] = 0.008
    closed[CONTROL_ORDER.index("right_gripper")] = 0.007
    apply_control_action(robot, dof_index, clip_control_action(closed))
    for _ in range(max(settle_frames, 0)):
        world.step(render=True)
    raw = raw_positions_by_name(robot, dof_names)
    expected = control_action_to_raw_targets(closed)
    gripper_report = {}
    for logical_name, raw_names in GRIPPER_MAP.items():
        values = [raw[name] for name in raw_names]
        targets = [expected[name] for name in raw_names]
        gripper_report[logical_name] = {
            "raw_joint_names": list(raw_names),
            "raw_joint_positions": values,
            "target_positions": targets,
            "settle_frames": settle_frames,
            "max_abs_error": max(abs(value - target) for value, target in zip(values, targets)),
        }
    return gripper_report


def validate_cameras(cameras: dict, world, image_dir: Path) -> dict:
    image_dir.mkdir(parents=True, exist_ok=True)
    report = {}
    for camera_name, camera in cameras.items():
        spec = CAMERA_SPECS[camera_name]
        width, height = spec["resolution"]
        rgb = capture_rgb(camera, world, width=width, height=height)
        path = image_dir / f"{camera_name}.png"
        write_png(path, rgb)
        report[camera_name] = {
            "path": str(path),
            "shape": list(rgb.shape),
            "min": int(rgb.min()),
            "max": int(rgb.max()),
            "mean": float(rgb.mean()),
            "non_empty": bool(rgb.size and int(rgb.max()) > int(rgb.min())),
        }
    return report


def main() -> int:
    args = parse_args()
    kit = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    started_at = time.time()
    result: dict = {"stage": args.stage, "started_at_unix": started_at}
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.sensors.camera import Camera

        open_ok = omni.usd.get_context().open_stage(args.stage)
        if open_ok is False:
            raise RuntimeError(f"failed to open stage: {args.stage}")
        for _ in range(90):
            kit.update()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("stage did not load")
        result["stage_summary"] = summarize_stage(stage)
        result["camera_metadata"] = camera_metadata(stage)
        result["force_torque_sensor_metadata"] = sensor_metadata(stage)

        world = World(stage_units_in_meters=1.0)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        world.reset()

        robot = Articulation(ARTICULATION_ROOT)
        robot.initialize()
        for _ in range(max(args.warmup_frames, 1)):
            world.step(render=True)

        dof_names = get_dof_names(robot)
        dof_index = {name: i for i, name in enumerate(dof_names)}
        result["articulation"] = {
            "root": ARTICULATION_ROOT,
            "root_prim_exists": stage.GetPrimAtPath(ARTICULATION_ROOT).IsValid(),
            "physics_handle_valid": bool(robot.is_physics_handle_valid()),
            "num_dof": len(dof_names),
            "dof_names": dof_names,
            "expected_raw_dofs": list(RAW_DOF_NAMES),
            "expected_effective_control_dofs": len(CONTROL_ORDER),
            "control_order": list(CONTROL_ORDER),
            "missing_expected_raw_dofs": [name for name in RAW_DOF_NAMES if name not in dof_index],
        }

        cameras = {}
        for camera_name, spec in CAMERA_SPECS.items():
            camera = Camera(
                prim_path=spec["prim_path"],
                name=f"{camera_name}_build_validation_camera",
                resolution=spec["resolution"],
            )
            camera.initialize()
            cameras[camera_name] = camera

        home = list(DEFAULT_HOME_ACTION)
        for _ in range(args.frames):
            apply_control_action(robot, dof_index, home)
            world.step(render=True)

        result["control_state_sample"] = get_control_state(robot, dof_names)
        result["gripper_command_validation"] = validate_gripper_mapping(
            robot, dof_index, dof_names, world, args.gripper_settle_frames
        )

        readings, vector = read_force_torque(robot, dof_index, model="HKVL-36A")
        result["force_torque_runtime_sample"] = {
            "vector": vector,
            "vector_len": len(vector),
            "finite": finite_list(vector),
            "readings": readings,
            "saturated_any": any(bool(item["saturated"]) for item in readings.values()),
        }
        result["camera_runtime_sample"] = validate_cameras(cameras, world, Path(args.image_dir))

        checks = {
            "stage_loaded": stage is not None,
            "articulation_root_exists": result["articulation"]["root_prim_exists"],
            "physics_handle_valid": result["articulation"]["physics_handle_valid"],
            "raw_dof_count_is_16": result["articulation"]["num_dof"] == len(RAW_DOF_NAMES),
            "all_expected_raw_dofs_present": not result["articulation"]["missing_expected_raw_dofs"],
            "effective_control_is_14": len(CONTROL_ORDER) == 14,
            "collision_prims_at_least_20": result["stage_summary"]["collision_prim_count"] >= 20,
            "mesh_prims_at_least_19": result["stage_summary"]["mesh_prim_count"] >= 19,
            "three_cameras_present": all(item["exists"] for item in result["camera_metadata"].values()),
            "three_cameras_render_non_empty": all(
                item["non_empty"] and item["shape"] == [480, 640, 3]
                for item in result["camera_runtime_sample"].values()
            ),
            "two_force_torque_sensor_prims_present": all(
                item["sensor_exists"] and item["joint_exists"]
                for item in result["force_torque_sensor_metadata"].values()
            ),
            "force_torque_runtime_is_12d_finite": result["force_torque_runtime_sample"]["vector_len"] == 12
            and result["force_torque_runtime_sample"]["finite"],
            "gripper_logical_axes_drive_two_raw_joints": all(
                item["max_abs_error"] < 1e-4 for item in result["gripper_command_validation"].values()
            ),
        }
        result["checks"] = checks
        result["build_success"] = all(checks.values())
        result["elapsed_s"] = time.time() - started_at
        timeline.stop()
    except Exception as exc:  # pragma: no cover - exercised inside Isaac Sim.
        result["build_success"] = False
        result["error"] = repr(exc)
    finally:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("MICROMANI_BUILD_VALIDATION", json.dumps(result, ensure_ascii=False), flush=True)
        kit.close()
    return 0 if result.get("build_success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
