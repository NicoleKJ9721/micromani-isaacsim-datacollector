#!/usr/bin/env python3
"""Re-apply MicroMani camera poses to an existing USD scene."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


STAGE_PATH = "/isaac-sim/.local/share/ov/data/micromani/micromanipulator_scene.usd"

CAMERA_POSES = {
    "/micromanipulator/link6_left/left_wrist_camera": {
        "eye": (0.045, -0.075, 0.09),
        "target": (0.045, -0.04, 0.032),
        "view_width_m": 0.04,
        "resolution": (640, 480),
        "fps": 30,
    },
    "/micromanipulator/link6_right/right_wrist_camera": {
        "eye": (-0.045, -0.075, 0.09),
        "target": (-0.045, -0.04, 0.032),
        "view_width_m": 0.04,
        "resolution": (640, 480),
        "fps": 30,
    },
    "/World/global_camera": {
        "eye": (0.0, -0.22, 0.26),
        "target": (0.0, -0.02, 0.06),
        "view_width_m": 0.15,
        "resolution": (640, 480),
        "fps": 30,
    },
}


def wrist_work_target(finger1_points, finger2_points, inward_x):
    import numpy as np

    centers = []
    for points in (finger1_points, finger2_points):
        distance_along_axis = inward_x * points[:, 0]
        tip = points[distance_along_axis >= distance_along_axis.max() - .001]
        centers.append((tip.min(axis=0) + tip.max(axis=0)) / 2)
    target = (centers[0] + centers[1]) / 2
    target[0] += inward_x * .001
    return tuple(target)


def vec_norm(v):
    length = math.sqrt(sum(x * x for x in v))
    if length == 0:
        raise ValueError("zero-length vector")
    return tuple(x / length for x in v)


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def camera_matrix(eye, target, up=(0.0, 0.0, 1.0)):
    from pxr import Gf

    forward = vec_norm(tuple(target[i] - eye[i] for i in range(3)))
    right = vec_norm(cross(forward, up))
    true_up = cross(right, forward)
    minus_forward = tuple(-x for x in forward)
    return Gf.Matrix4d(
        right[0],
        right[1],
        right[2],
        0.0,
        true_up[0],
        true_up[1],
        true_up[2],
        0.0,
        minus_forward[0],
        minus_forward[1],
        minus_forward[2],
        0.0,
        eye[0],
        eye[1],
        eye[2],
        1.0,
    )


def focal_length_for_view(width_m, distance_m, horizontal_aperture_mm=20.955):
    fov = 2.0 * math.atan(width_m / (2.0 * distance_m))
    return (horizontal_aperture_mm / 2.0) / math.tan(fov / 2.0)


def set_world_camera_pose(stage, path, eye, target):
    from pxr import Sdf, UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        cam = UsdGeom.Camera.Define(stage, Sdf.Path(path))
        prim = cam.GetPrim()
    xform = UsdGeom.Xformable(prim)
    desired_world = camera_matrix(eye, target)
    parent = prim.GetParent()
    parent_path = str(parent.GetPath()) if parent and parent.IsValid() else "/"
    if parent_path in {"/", "/World"}:
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(desired_world)
        return {"parent": parent_path, "translation_error_m": 0.0}

    cache = UsdGeom.XformCache()
    parent_world = cache.GetLocalToWorldTransform(parent)
    parent_inv = parent_world.GetInverse()
    candidates = [desired_world * parent_inv, parent_inv * desired_world]
    best = None
    best_error = float("inf")
    for candidate in candidates:
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(candidate)
        cache.Clear()
        actual = cache.GetLocalToWorldTransform(prim)
        actual_eye = actual.ExtractTranslation()
        error = math.sqrt(sum((actual_eye[i] - eye[i]) ** 2 for i in range(3)))
        if error < best_error:
            best_error = error
            best = candidate
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(best)
    return {"parent": parent_path, "translation_error_m": best_error}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=STAGE_PATH)
    parser.add_argument("--report", default=None)
    args = parser.parse_args()

    from isaacsim import SimulationApp

    kit = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    from pxr import Gf, Usd, UsdGeom

    try:
        stage = Usd.Stage.Open(args.stage)
        if stage is None:
            raise RuntimeError(f"failed to open stage {args.stage}")

        report = {}
        for path, spec in CAMERA_POSES.items():
            result = set_world_camera_pose(stage, path, spec["eye"], spec["target"])
            cam = UsdGeom.Camera.Get(stage, path)
            distance = math.sqrt(sum((spec["target"][i] - spec["eye"][i]) ** 2 for i in range(3)))
            cam.CreateHorizontalApertureAttr(20.955)
            cam.CreateVerticalApertureAttr(15.2908)
            cam.CreateFocalLengthAttr(focal_length_for_view(spec["view_width_m"], distance))
            cam.CreateFocusDistanceAttr(distance)
            cam.CreateFStopAttr(16.0)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.001, 5.0))
            prim = cam.GetPrim()
            prim.SetCustomDataByKey("micromani_resolution_width", int(spec["resolution"][0]))
            prim.SetCustomDataByKey("micromani_resolution_height", int(spec["resolution"][1]))
            prim.SetCustomDataByKey("micromani_fps", int(spec["fps"]))
            prim.SetCustomDataByKey("micromani_target_fov_width_m", float(spec["view_width_m"]))
            prim.SetCustomDataByKey("micromani_target_fov_height_m", float(spec["view_width_m"]))
            report[path] = {**result, "eye": list(spec["eye"]), "target": list(spec["target"])}

        stage.GetRootLayer().Save()
        if args.report:
            Path(args.report).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        kit.close()


if __name__ == "__main__":
    main()
