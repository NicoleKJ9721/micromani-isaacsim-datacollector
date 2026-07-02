import json
import math
import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from micromani_lerobot_schema import CONTROL_ORDER, GRIPPER_MAP, RAW_DOF_NAMES  # noqa: E402


ASSET_ROOT = Path(os.environ.get("MICROMANI_ASSET_ROOT", "/tmp/micromanipulator_isaac"))
URDF_PATH = Path(os.environ.get("MICROMANI_URDF", ASSET_ROOT / "urdf" / "micromanipulator_isaac.urdf"))
OUT_DIR = Path(os.environ.get("MICROMANI_OUT_DIR", "/isaac-sim/.local/share/ov/data/micromani"))
OUT_USD = Path(os.environ.get("MICROMANI_OUT_USD", OUT_DIR / "micromanipulator_scene.usd"))
REPORT_PATH = Path(os.environ.get("MICROMANI_REPORT", OUT_DIR / "micromanipulator_import_report.json"))


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
    """Return a USD transform whose camera -Z axis points at target."""
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


def add_camera(stage, path, eye, target, view_width_m, resolution, fps, parent_note):
    from pxr import Gf, Sdf, UsdGeom

    cam = UsdGeom.Camera.Define(stage, Sdf.Path(path))
    xform = UsdGeom.Xformable(cam.GetPrim())
    xform.ClearXformOpOrder()
    desired_world = camera_matrix(eye, target)
    parent = cam.GetPrim().GetParent()
    parent_path = str(parent.GetPath()) if parent and parent.IsValid() else "/"
    if parent_path not in {"/", "/World"}:
        cache = UsdGeom.XformCache()
        parent_world = cache.GetLocalToWorldTransform(parent)
        parent_inv = parent_world.GetInverse()
        candidates = [desired_world * parent_inv, parent_inv * desired_world]
        best_local = candidates[0]
        best_error = float("inf")
        for candidate in candidates:
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(candidate)
            cache.Clear()
            actual_world = cache.GetLocalToWorldTransform(cam.GetPrim())
            actual_eye = actual_world.ExtractTranslation()
            error = math.sqrt(
                (actual_eye[0] - eye[0]) ** 2 + (actual_eye[1] - eye[1]) ** 2 + (actual_eye[2] - eye[2]) ** 2
            )
            if error < best_error:
                best_error = error
                best_local = candidate
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(best_local)
    else:
        xform.AddTransformOp().Set(desired_world)
    distance = math.sqrt(sum((target[i] - eye[i]) ** 2 for i in range(3)))
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(15.2908)
    cam.CreateFocalLengthAttr(focal_length_for_view(view_width_m, distance))
    cam.CreateFocusDistanceAttr(distance)
    cam.CreateFStopAttr(16.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.001, 5.0))
    prim = cam.GetPrim()
    prim.SetCustomDataByKey("micromani_resolution_width", int(resolution[0]))
    prim.SetCustomDataByKey("micromani_resolution_height", int(resolution[1]))
    prim.SetCustomDataByKey("micromani_fps", int(fps))
    prim.SetCustomDataByKey("micromani_target_fov_width_m", float(view_width_m))
    prim.SetCustomDataByKey("micromani_target_fov_height_m", float(view_width_m))
    prim.SetCustomDataByKey("micromani_mount", parent_note)
    return {
        "path": path,
        "eye": list(eye),
        "target": list(target),
        "distance_m": distance,
        "target_fov_m": [view_width_m, view_width_m],
        "resolution": list(resolution),
        "fps": fps,
        "focal_length_mm": float(cam.GetFocalLengthAttr().Get()),
        "mount": parent_note,
    }


def main():
    kit = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

    import omni.kit.commands
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if OUT_USD.exists():
        OUT_USD.unlink()

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.distance_scale = 1.0
    import_config.parse_mimic = True
    import_config.self_collision = False
    import_config.collision_from_visuals = False
    if hasattr(import_config, "set_default_drive_type"):
        import_config.set_default_drive_type(1)

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(URDF_PATH),
        import_config=import_config,
        dest_path=str(OUT_USD),
        get_articulation_root=True,
    )
    if not status or not prim_path:
        raise RuntimeError(f"URDFParseAndImportFile failed: status={status}, prim_path={prim_path}")

    if OUT_USD.exists():
        OUT_USD.unlink()
    stage = Usd.Stage.CreateNew(str(OUT_USD))
    stage.GetRootLayer().subLayerPaths = [
        "configuration/micromanipulator_scene_base.usd",
        "configuration/micromanipulator_scene_physics.usd",
        "configuration/micromanipulator_scene_sensor.usd",
        "configuration/micromanipulator_scene_robot.usd",
    ]
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physx_scene = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateSolverTypeAttr("TGS")

    PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 1.0, Gf.Vec3f(0, 0, -0.002), Gf.Vec3f(0.25))
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
    light.CreateIntensityAttr(800)

    # Local wrist camera poses are parented under wrist links so they move with the wrists.
    cameras = []
    cameras.append(
        add_camera(
            stage,
            "/micromanipulator/link6_left/left_wrist_camera",
            (0.045, -0.075, 0.09),
            (0.045, -0.04, 0.032),
            0.04,
            (640, 480),
            30,
            "child of link6_left, 45deg downward toward left gripper",
        )
    )
    cameras.append(
        add_camera(
            stage,
            "/micromanipulator/link6_right/right_wrist_camera",
            (-0.045, -0.075, 0.09),
            (-0.045, -0.04, 0.032),
            0.04,
            (640, 480),
            30,
            "child of link6_right, 45deg downward toward right gripper",
        )
    )
    cameras.append(
        add_camera(
            stage,
            "/World/global_camera",
            (0.0, -0.22, 0.26),
            (0.0, -0.02, 0.06),
            0.15,
            (640, 480),
            30,
            "world camera above both arms, oblique downward",
        )
    )

    stage.SetDefaultPrim(stage.GetPrimAtPath("/micromanipulator"))
    try:
        stage.GetRootLayer().Save()
    except Exception:
        stage.GetRootLayer().Export(str(OUT_USD))
    omni.usd.get_context().open_stage(str(OUT_USD))
    for _ in range(20):
        kit.update()
    stage = omni.usd.get_context().get_stage()

    articulation_candidates = []
    joint_prims = []
    collision_prims = []
    mimic_prims = []
    for prim in stage.Traverse():
        prim_type = prim.GetTypeName()
        path = str(prim.GetPath())
        if hasattr(UsdPhysics, "ArticulationRootAPI") and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_candidates.append(path)
        if hasattr(PhysxSchema, "PhysxArticulationAPI") and prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            articulation_candidates.append(path)
        if prim_type.endswith("Joint"):
            joint_prims.append({"path": path, "type": prim_type})
            if prim.HasAPI(PhysxSchema.PhysxMimicJointAPI):
                mimic_prims.append(path)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_prims.append(path)

    articulation_candidates = sorted(set(articulation_candidates))
    articulation_root_path = articulation_candidates[0] if articulation_candidates else "/micromanipulator"
    physics_handle_valid = False
    articulation_error = None
    dof_names = []
    num_dof = None
    try:
        omni.timeline.get_timeline_interface().play()
        kit.update()
        art = Articulation(articulation_root_path)
        art.initialize()
        for _ in range(10):
            kit.update()
        physics_handle_valid = bool(art.is_physics_handle_valid())
        for attr in ("dof_names", "joint_names"):
            try:
                value = getattr(art, attr)
                dof_names = list(value() if callable(value) else value)
                if dof_names:
                    break
            except Exception:
                pass
        for attr in ("num_dof", "num_joints"):
            try:
                value = getattr(art, attr)
                num_dof = int(value() if callable(value) else value)
                break
            except Exception:
                pass
    except Exception as exc:
        articulation_error = str(exc)
    finally:
        omni.timeline.get_timeline_interface().stop()

    report = {
        "asset_root": str(ASSET_ROOT),
        "urdf_path": str(URDF_PATH),
        "usd_path": str(OUT_USD),
        "import_returned_prim_path": str(prim_path),
        "articulation_root": articulation_root_path,
        "articulation_candidates": articulation_candidates,
        "physics_handle_valid": physics_handle_valid,
        "articulation_error": articulation_error,
        "num_dof": num_dof,
        "dof_names": dof_names,
        "expected_raw_actuated_dofs": len(RAW_DOF_NAMES),
        "expected_effective_control_dofs": len(CONTROL_ORDER),
        "control_order": list(CONTROL_ORDER),
        "gripper_map": {key: list(value) for key, value in GRIPPER_MAP.items()},
        "joint_prim_count": len(joint_prims),
        "joint_prims": joint_prims,
        "collision_prim_count": len(collision_prims),
        "collision_prims": collision_prims,
        "mimic_prim_count": len(mimic_prims),
        "mimic_prims": mimic_prims,
        "cameras": cameras,
    }
    REPORT_PATH.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))

    kit.close()


if __name__ == "__main__":
    main()
