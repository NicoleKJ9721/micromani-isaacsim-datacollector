#!/usr/bin/env python3
"""Convert MicroMani raw Isaac episodes into a LeRobot v3 dataset."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from micromani_lerobot_schema import (
    CAMERA_SPECS,
    DEFAULT_TASK,
    FORCE_TORQUE_FEATURE_KEY,
    FORCE_TORQUE_SENSORS,
    FORCE_TORQUE_VECTOR_NAMES,
    build_lerobot_features,
    force_torque_vector_from_readings,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-session", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--repo-id", default="micromani/isaac_micro_bimanual")
    parser.add_argument("--robot-type", default="micromani_isaac")
    parser.add_argument("--image-dtype", choices=("image", "video"), default="image")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def load_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def read_rgb(path: Path) -> np.ndarray:
    arr = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
    return arr


def row_force_torque_vector(row: dict) -> list[float]:
    if "observation_force_torque" in row:
        values = [float(v) for v in row["observation_force_torque"]]
    elif "force_torque" in row:
        values = force_torque_vector_from_readings(row["force_torque"])
    else:
        raise KeyError("force_torque")
    if len(values) != len(FORCE_TORQUE_VECTOR_NAMES):
        raise ValueError(f"force_torque has length {len(values)}, expected {len(FORCE_TORQUE_VECTOR_NAMES)}")
    return values


def raw_session_has_force_torque(rows: list[dict]) -> bool:
    return bool(rows) and all("observation_force_torque" in row or "force_torque" in row for row in rows)


def validate_raw_session(raw_session: Path, manifest: dict, rows: list[dict], has_force_torque: bool) -> None:
    expected_frames = int(manifest["frames"])
    if len(rows) != expected_frames:
        raise ValueError(f"raw frame count mismatch: manifest={expected_frames}, rows={len(rows)}")
    for index, row in enumerate(rows):
        if row["frame_index"] != index:
            raise ValueError(f"frame index mismatch at {index}: {row['frame_index']}")
        for key in ("observation_state", "action"):
            values = row[key]
            if len(values) != 14:
                raise ValueError(f"{key} at frame {index} has length {len(values)}, expected 14")
            if not np.isfinite(np.asarray(values, dtype=np.float32)).all():
                raise ValueError(f"{key} at frame {index} contains non-finite values")
        if has_force_torque:
            values = row_force_torque_vector(row)
            if not np.isfinite(np.asarray(values, dtype=np.float32)).all():
                raise ValueError(f"force_torque at frame {index} contains non-finite values")
        for camera_name in CAMERA_SPECS:
            rel = row["images"][camera_name]
            img_path = raw_session / manifest["episode_dir"] / rel
            if not img_path.exists():
                raise FileNotFoundError(img_path)


def write_force_torque_sidecars(output_root: Path, rows: list[dict], manifest: dict) -> dict:
    force_dir = output_root / "force_torque"
    force_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = force_dir / "episode_000000.jsonl"
    csv_path = force_dir / "episode_000000.csv"
    metadata_path = force_dir / "metadata.json"
    columns = ["frame_index", "timestamp", *FORCE_TORQUE_VECTOR_NAMES, "left_wrist_saturated", "right_wrist_saturated"]
    metadata_path.write_text(
        json.dumps(
            {
                "vector_order": list(FORCE_TORQUE_VECTOR_NAMES),
                "source_manifest": manifest.get("force_torque_feature", {}),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    with jsonl_path.open("w", encoding="utf-8") as jsonl_fp, csv_path.open("w", encoding="utf-8") as csv_fp:
        csv_fp.write(",".join(columns) + "\n")
        for row in rows:
            vector = row_force_torque_vector(row)
            force_torque = row.get("force_torque") or {}
            json_record = {
                "frame_index": row["frame_index"],
                "timestamp": row["timestamp"],
                "vector_order": list(FORCE_TORQUE_VECTOR_NAMES),
                "values": vector,
                "force_torque": force_torque,
            }
            jsonl_fp.write(json.dumps(json_record, ensure_ascii=False) + "\n")
            left_sat = bool(force_torque.get("left_wrist", {}).get("saturated", False))
            right_sat = bool(force_torque.get("right_wrist", {}).get("saturated", False))
            csv_values = [
                row["frame_index"],
                row["timestamp"],
                *vector,
                int(left_sat),
                int(right_sat),
            ]
            csv_fp.write(",".join(str(v) for v in csv_values) + "\n")

    return {"jsonl": str(jsonl_path), "csv": str(csv_path), "metadata": str(metadata_path), "columns": columns}


def convert() -> dict:
    args = parse_args()
    raw_session = Path(args.raw_session).expanduser().resolve()
    output_root = Path(args.output_root).expanduser().resolve()
    if (raw_session / "session.json").exists():
        from micromani_export_session import export_session
        report = export_session(raw_session, output_root, args.repo_id,
                                image_dtype=args.image_dtype, overwrite=args.overwrite)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return report
    manifest = json.loads((raw_session / "manifest.json").read_text(encoding="utf-8"))
    episode_dir = raw_session / manifest["episode_dir"]
    rows = load_jsonl(episode_dir / "frames.jsonl")
    has_force_torque = raw_session_has_force_torque(rows)
    validate_raw_session(raw_session, manifest, rows, has_force_torque=has_force_torque)

    if output_root.exists():
        if not args.overwrite:
            raise FileExistsError(f"{output_root} already exists; pass --overwrite to replace it")
        shutil.rmtree(output_root)

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = build_lerobot_features(args.image_dtype, include_force_torque=has_force_torque)
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=int(manifest["fps"]),
        root=output_root,
        robot_type=args.robot_type,
        features=features,
        use_videos=args.image_dtype == "video",
        image_writer_processes=0,
        image_writer_threads=max(1, len(CAMERA_SPECS) * 2),
        metadata_buffer_size=1,
    )

    for row in rows:
        frame = {
            "observation.state": np.asarray(row["observation_state"], dtype=np.float32),
            "action": np.asarray(row["action"], dtype=np.float32),
            "task": row.get("task") or manifest.get("task") or DEFAULT_TASK,
        }
        if has_force_torque:
            frame[FORCE_TORQUE_FEATURE_KEY] = np.asarray(row_force_torque_vector(row), dtype=np.float32)
        for camera_name, spec in CAMERA_SPECS.items():
            img_path = episode_dir / row["images"][camera_name]
            frame[spec["feature_key"]] = read_rgb(img_path)
        dataset.add_frame(frame)
    dataset.save_episode(parallel_encoding=False)
    dataset.finalize()
    force_torque_sidecars = write_force_torque_sidecars(output_root, rows, manifest) if has_force_torque else None

    loaded = LeRobotDataset(args.repo_id, root=output_root, episodes=[0], return_uint8=True)
    sample = loaded[0]
    force_torque_shape = list(sample[FORCE_TORQUE_FEATURE_KEY].shape) if has_force_torque else None
    validation = {
        "dataset_root": str(output_root),
        "repo_id": args.repo_id,
        "num_frames": len(loaded),
        "fps": int(manifest["fps"]),
        "feature_keys": sorted(loaded.features.keys()),
        "sample_state_shape": list(sample["observation.state"].shape),
        "sample_action_shape": list(sample["action"].shape),
        "sample_force_torque_shape": force_torque_shape,
        "force_torque_sidecars": force_torque_sidecars,
        "force_torque_sensors": list(FORCE_TORQUE_SENSORS.keys()) if has_force_torque else [],
        "camera_shapes": {
            spec["feature_key"]: list(sample[spec["feature_key"]].shape) for spec in CAMERA_SPECS.values()
        },
    }
    (output_root / "micromani_validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(validation, ensure_ascii=False, indent=2))
    return validation


if __name__ == "__main__":
    convert()
