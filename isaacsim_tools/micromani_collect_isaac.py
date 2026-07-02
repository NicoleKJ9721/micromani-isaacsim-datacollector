#!/usr/bin/env python3
"""Collect short LeRobot-style raw episodes from the MicroMani Isaac Sim scene.

Run inside the Isaac Sim container with:
    /isaac-sim/python.sh micromani_collect_isaac.py --frames 60
"""

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

from micromani_lerobot_schema import (  # noqa: E402
    ARTICULATION_ROOT,
    CAMERA_SPECS,
    CONTROL_ORDER,
    DEFAULT_HOME_ACTION,
    DEFAULT_TASK,
    FORCE_TORQUE_AXES,
    FORCE_TORQUE_SENSORS,
    FORCE_TORQUE_VECTOR_NAMES,
    RAW_DOF_NAMES,
    STAGE_PATH,
    clip_control_action,
    control_action_to_raw_targets,
    force_torque_vector_from_readings,
    hkvl36_model_spec,
    quantize_hkvl36_wrench,
    raw_state_to_control_state,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=STAGE_PATH)
    parser.add_argument("--out-dir", default="/isaac-sim/.local/share/ov/data/micromani/lerobot_raw")
    parser.add_argument("--session-name", default=None)
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=45)
    parser.add_argument("--closed-loop-gain", type=float, default=0.65)
    parser.add_argument("--force-torque-model", default="HKVL-36A", choices=("HKVL-36A", "HKVL-36B", "HKVL-36C"))
    parser.add_argument("--task", default=DEFAULT_TASK)
    return parser.parse_args()


def make_session_dir(out_dir: Path, session_name: str | None) -> Path:
    if session_name is None:
        session_name = time.strftime("micromani_%Y%m%d_%H%M%S")
    session_dir = out_dir / session_name
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def make_target_action(frame_index: int, frame_count: int) -> list[float]:
    phase = 2.0 * math.pi * frame_index / max(frame_count - 1, 1)
    base = list(DEFAULT_HOME_ACTION)
    offsets = [
        0.010 * math.sin(phase),
        0.006 * math.cos(phase),
        0.008 * math.sin(phase + 0.7),
        0.22 * math.sin(phase),
        0.055 * math.cos(phase + 0.4),
        0.35 * math.sin(phase + 1.2),
        0.003 * (0.5 + 0.5 * math.sin(phase)),
        -0.010 * math.sin(phase),
        0.006 * math.cos(phase + math.pi),
        0.008 * math.sin(phase + math.pi + 0.7),
        -0.22 * math.sin(phase),
        -0.055 * math.cos(phase + 0.4),
        -0.35 * math.sin(phase + 1.2),
        0.003 * (0.5 + 0.5 * math.sin(phase + math.pi)),
    ]
    return clip_control_action([b + o for b, o in zip(base, offsets)])


def blend_action(current_state: list[float], target_action: list[float], gain: float) -> list[float]:
    return clip_control_action(
        current + gain * (target - current) for current, target in zip(current_state, target_action)
    )


def as_uint8_rgb(rgba):
    import numpy as np

    arr = np.asarray(rgba)
    if arr.size == 0:
        raise ValueError("empty camera frame")
    if arr.ndim != 3 or arr.shape[2] < 3:
        raise ValueError(f"unexpected camera frame shape {arr.shape}")
    rgb = arr[:, :, :3]
    if rgb.dtype != np.uint8:
        high = float(rgb.max()) if rgb.size else 0.0
        if high <= 1.0:
            rgb = rgb * 255.0
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    return rgb


def capture_rgb(camera, world, width: int, height: int, retries: int = 8):
    for _ in range(retries):
        try:
            rgb = as_uint8_rgb(camera.get_rgba())
            if rgb.shape == (height, width, 3):
                return rgb
        except Exception:
            pass
        world.step(render=True)
    rgb = as_uint8_rgb(camera.get_rgba())
    if rgb.shape != (height, width, 3):
        raise ValueError(f"camera returned {rgb.shape}, expected {(height, width, 3)}")
    return rgb


def get_dof_names(robot) -> list[str]:
    for attr in ("dof_names", "joint_names"):
        value = getattr(robot, attr, None)
        if value is None:
            continue
        names = list(value() if callable(value) else value)
        if names:
            return names
    raise RuntimeError("could not read articulation DOF names")


def get_control_state(robot, dof_names: list[str]) -> list[float]:
    import numpy as np

    positions = robot.get_joint_positions()
    positions = np.asarray(positions, dtype=np.float32).reshape(-1)
    if len(positions) < len(dof_names):
        raise RuntimeError(f"got {len(positions)} joint positions for {len(dof_names)} DOFs")
    raw_by_name = {name: float(positions[i]) for i, name in enumerate(dof_names)}
    return raw_state_to_control_state(raw_by_name)


def read_force_torque(robot, dof_index: dict[str, int], model: str) -> tuple[dict, list[float]]:
    import numpy as np

    joint_rows = []
    sensor_names = []
    missing = []
    for sensor_name, spec in FORCE_TORQUE_SENSORS.items():
        joint_name = spec["joint_name"]
        if joint_name not in dof_index:
            missing.append(joint_name)
            continue
        joint_rows.append(dof_index[joint_name] + 1)
        sensor_names.append(sensor_name)
    if missing:
        raise RuntimeError(f"missing force-torque sensor joints in articulation: {missing}")

    measured = robot.get_measured_joint_forces(joint_indices=np.asarray(joint_rows, dtype=np.int32))
    if measured is None:
        raise RuntimeError("Isaac returned no measured joint forces")
    rows = np.asarray(measured, dtype=np.float64).reshape(-1, len(FORCE_TORQUE_AXES))
    if rows.shape[0] != len(sensor_names):
        raise RuntimeError(f"expected {len(sensor_names)} force-torque rows, got {rows.shape[0]}")

    readings = {}
    for sensor_name, raw_row in zip(sensor_names, rows):
        wrench, saturated = quantize_hkvl36_wrench(raw_row.tolist(), model=model)
        spec = FORCE_TORQUE_SENSORS[sensor_name]
        readings[sensor_name] = {
            "model": model,
            "joint_name": spec["joint_name"],
            "joint_row_index": int(dof_index[spec["joint_name"]] + 1),
            "frame": spec["frame"],
            "axes": list(FORCE_TORQUE_AXES),
            "wrench": wrench,
            "raw_wrench": [float(v) for v in raw_row],
            "saturated": bool(saturated),
        }
    return readings, force_torque_vector_from_readings(readings)


def apply_control_action(robot, dof_index: dict[str, int], action: list[float]) -> None:
    import numpy as np

    raw_targets = control_action_to_raw_targets(action)
    joint_indices = []
    joint_positions = []
    missing = []
    for name, value in raw_targets.items():
        if name not in dof_index:
            missing.append(name)
            continue
        joint_indices.append(dof_index[name])
        joint_positions.append(value)
    if missing:
        raise RuntimeError(f"missing DOFs in articulation: {missing}")
    robot.set_joint_positions(np.asarray(joint_positions, dtype=np.float32), joint_indices=np.asarray(joint_indices))


def write_png(path: Path, rgb) -> None:
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb).save(path)


def add_micro_target() -> None:
    import numpy as np
    from isaacsim.core.api.objects import VisualCuboid
    from pxr import Sdf
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    path = Sdf.Path("/World/micro_target")
    if stage.GetPrimAtPath(path).IsValid():
        return
    VisualCuboid(
        prim_path=str(path),
        name="micro_target",
        position=np.array([0.0, -0.038, 0.028]),
        scale=np.array([0.008, 0.008, 0.002]),
        size=1.0,
        color=np.array([0.0, 0.85, 1.0]),
    )


def init_force_torque_stats() -> dict:
    return {
        sensor_name: {
            "min": [float("inf")] * len(FORCE_TORQUE_AXES),
            "max": [float("-inf")] * len(FORCE_TORQUE_AXES),
            "max_abs": [0.0] * len(FORCE_TORQUE_AXES),
            "saturated_count": 0,
            "count": 0,
        }
        for sensor_name in FORCE_TORQUE_SENSORS
    }


def update_force_torque_stats(stats: dict, readings: dict) -> None:
    for sensor_name, reading in readings.items():
        sensor_stats = stats[sensor_name]
        values = [float(v) for v in reading["wrench"]]
        for i, value in enumerate(values):
            sensor_stats["min"][i] = min(sensor_stats["min"][i], value)
            sensor_stats["max"][i] = max(sensor_stats["max"][i], value)
            sensor_stats["max_abs"][i] = max(sensor_stats["max_abs"][i], abs(value))
        if reading["saturated"]:
            sensor_stats["saturated_count"] += 1
        sensor_stats["count"] += 1


def finalize_force_torque_stats(stats: dict) -> dict:
    finalized = {}
    for sensor_name, sensor_stats in stats.items():
        count = sensor_stats["count"]
        finalized[sensor_name] = {
            "axes": list(FORCE_TORQUE_AXES),
            "min": [None if count == 0 else float(v) for v in sensor_stats["min"]],
            "max": [None if count == 0 else float(v) for v in sensor_stats["max"]],
            "max_abs": [float(v) for v in sensor_stats["max_abs"]],
            "saturated_count": int(sensor_stats["saturated_count"]),
            "count": int(count),
        }
    return finalized


def main() -> None:
    args = parse_args()
    kit = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

    import numpy as np
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.sensors.camera import Camera

    session_dir = make_session_dir(Path(args.out_dir), args.session_name)
    episode_dir = session_dir / f"episode_{args.episode_index:06d}"
    image_root = episode_dir / "images"
    episode_dir.mkdir(parents=True, exist_ok=True)

    open_ok = omni.usd.get_context().open_stage(args.stage)
    if open_ok is False:
        raise RuntimeError(f"failed to open stage: {args.stage}")
    for _ in range(90):
        kit.update()

    add_micro_target()
    world = World(stage_units_in_meters=1.0)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    world.reset()

    robot = Articulation(ARTICULATION_ROOT)
    robot.initialize()
    for _ in range(20):
        world.step(render=True)
    if not robot.is_physics_handle_valid():
        raise RuntimeError(f"invalid physics handle for {ARTICULATION_ROOT}")
    dof_names = get_dof_names(robot)
    dof_index = {name: i for i, name in enumerate(dof_names)}

    cameras = {}
    for camera_name, spec in CAMERA_SPECS.items():
        width, height = spec["resolution"]
        camera = Camera(
            prim_path=spec["prim_path"],
            name=f"{camera_name}_camera",
            resolution=(width, height),
        )
        camera.initialize()
        cameras[camera_name] = camera

    home = list(DEFAULT_HOME_ACTION)
    for _ in range(max(args.warmup_frames, 1)):
        apply_control_action(robot, dof_index, home)
        world.step(render=True)

    frame_path = episode_dir / "frames.jsonl"
    force_torque_path = episode_dir / "force_torque.jsonl"
    image_stats = {name: {"min": 255, "max": 0, "mean_sum": 0.0, "count": 0} for name in CAMERA_SPECS}
    force_torque_stats = init_force_torque_stats()
    tracking_errors = []
    started_at = time.time()

    with frame_path.open("w", encoding="utf-8") as fp, force_torque_path.open("w", encoding="utf-8") as ft_fp:
        for frame_index in range(args.frames):
            obs_state = get_control_state(robot, dof_names)
            force_torque_readings, force_torque_vector = read_force_torque(
                robot, dof_index, model=args.force_torque_model
            )
            update_force_torque_stats(force_torque_stats, force_torque_readings)
            target_action = make_target_action(frame_index, args.frames)
            action = blend_action(obs_state, target_action, args.closed_loop_gain)
            tracking_errors.append(float(np.mean(np.abs(np.asarray(target_action) - np.asarray(obs_state)))))

            image_paths = {}
            for camera_name, camera in cameras.items():
                spec = CAMERA_SPECS[camera_name]
                width, height = spec["resolution"]
                rgb = capture_rgb(camera, world, width=width, height=height)
                rel_path = Path("images") / camera_name / f"{frame_index:06d}.png"
                write_png(episode_dir / rel_path, rgb)
                image_paths[camera_name] = str(rel_path)
                stats = image_stats[camera_name]
                stats["min"] = min(stats["min"], int(rgb.min()))
                stats["max"] = max(stats["max"], int(rgb.max()))
                stats["mean_sum"] += float(rgb.mean())
                stats["count"] += 1

            record = {
                "frame_index": frame_index,
                "timestamp": frame_index / args.fps,
                "observation_state": obs_state,
                "observation_force_torque": force_torque_vector,
                "force_torque": force_torque_readings,
                "target_action": target_action,
                "action": action,
                "images": image_paths,
                "task": args.task,
            }
            fp.write(json.dumps(record, ensure_ascii=False) + "\n")
            ft_fp.write(
                json.dumps(
                    {
                        "frame_index": frame_index,
                        "timestamp": frame_index / args.fps,
                        "force_torque": force_torque_readings,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )

            apply_control_action(robot, dof_index, action)
            world.step(render=True)

    for stats in image_stats.values():
        stats["mean"] = stats.pop("mean_sum") / max(stats["count"], 1)

    manifest = {
        "schema_version": "micromani_raw_v1",
        "created_at_unix": started_at,
        "stage": args.stage,
        "articulation_root": ARTICULATION_ROOT,
        "episode_index": args.episode_index,
        "fps": args.fps,
        "frames": args.frames,
        "task": args.task,
        "control_order": list(CONTROL_ORDER),
        "raw_dof_names_expected": list(RAW_DOF_NAMES),
        "raw_dof_names_observed": dof_names,
        "camera_specs": CAMERA_SPECS,
        "force_torque_feature": {
            "axes": list(FORCE_TORQUE_AXES),
            "vector_order": list(FORCE_TORQUE_VECTOR_NAMES),
            "sample_fps": args.fps,
            "source": "Articulation.get_measured_joint_forces(joint_index + 1)",
            "frame": "local child joint frame",
            "model": hkvl36_model_spec(args.force_torque_model),
            "sensors": FORCE_TORQUE_SENSORS,
        },
        "force_torque_stats": finalize_force_torque_stats(force_torque_stats),
        "episode_dir": episode_dir.name,
        "tracking_error_mean_abs": float(np.mean(tracking_errors)) if tracking_errors else None,
        "image_stats": image_stats,
    }
    (session_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"session_dir": str(session_dir), "manifest": manifest}, ensure_ascii=False, indent=2))

    timeline.stop()
    kit.close()


if __name__ == "__main__":
    main()
