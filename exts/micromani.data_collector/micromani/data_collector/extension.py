from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import threading
import time
import weakref
from pathlib import Path

import carb
import omni.ext
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.menu import make_menu_item_description
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items


EXTENSION_NAME = "MicroMani Data Collector"
DEFAULT_STAGE = "/isaac-sim/.local/share/ov/data/micromani/micromanipulator_scene.usd"
DEFAULT_REPO_ID = "micromani/isaac_micro_bimanual_ft"


def _project_root_from_file() -> Path:
    return Path(__file__).resolve().parents[4]


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        self._project_root = Path(os.environ.get("MICROMANI_PROJECT_ROOT", _project_root_from_file())).resolve()
        self._tools_dir = self._project_root / "isaacsim_tools"
        self._bridge_module = None
        self._process = None
        self._process_log = None
        self._pico_process = None
        self._pico_log = None
        self._status_label = None

        self._project_root_model = ui.SimpleStringModel(str(self._project_root))
        self._stage_model = ui.SimpleStringModel(os.environ.get("MICROMANI_STAGE", DEFAULT_STAGE))
        self._session_model = ui.SimpleStringModel("")
        self._frames_model = ui.SimpleIntModel(6)
        self._fps_model = ui.SimpleIntModel(30)
        self._repo_model = ui.SimpleStringModel(DEFAULT_REPO_ID)
        self._status_model = ui.SimpleStringModel("Ready.")

        self._window = ui.Window(
            title=EXTENSION_NAME,
            width=520,
            height=560,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._on_window)

        menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._toggle_window())
        ]
        self._menu_items = [MenuItemDescription(name="MicroMani", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Tools")

    def on_shutdown(self):
        self._stop_bridge()
        self._stop_pico_gateway()
        if self._process and self._process.poll() is None:
            self._process.terminate()
        remove_menu_items(self._menu_items, "Tools")
        self._window = None

    def _toggle_window(self):
        self._window.visible = not self._window.visible

    def _on_window(self, _visible):
        if self._window.visible:
            self._build_ui()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label("Project Root")
                ui.StringField(model=self._project_root_model)
                ui.Label("Scene USD")
                ui.StringField(model=self._stage_model)
                ui.Label("Session Name")
                ui.StringField(model=self._session_model)

                with ui.HStack(spacing=8):
                    with ui.VStack():
                        ui.Label("Frames")
                        ui.IntField(model=self._frames_model)
                    with ui.VStack():
                        ui.Label("FPS")
                        ui.IntField(model=self._fps_model)

                ui.Label("LeRobot Repo ID")
                ui.StringField(model=self._repo_model)

                with ui.HStack(spacing=8):
                    ui.Button("Open Scene", clicked_fn=self._open_scene)
                    ui.Button("Save Stage", clicked_fn=self._save_stage)

                with ui.HStack(spacing=8):
                    ui.Button("Configure Cameras", clicked_fn=self._configure_cameras)
                    ui.Button("Configure HKVL-36", clicked_fn=self._configure_force_torque)

                with ui.HStack(spacing=8):
                    ui.Button("Start Bridge", clicked_fn=self._start_bridge)
                    ui.Button("Stop Bridge", clicked_fn=self._stop_bridge)

                with ui.HStack(spacing=8):
                    ui.Button("Start PICO Gateway", clicked_fn=self._start_pico_gateway)
                    ui.Button("Stop PICO Gateway", clicked_fn=self._stop_pico_gateway)

                with ui.HStack(spacing=8):
                    ui.Button("Run E2E Smoke", clicked_fn=self._run_smoke_pipeline)
                    ui.Button("Open Dataset Dir", clicked_fn=self._open_dataset_dir)

                with ui.HStack(spacing=8):
                    ui.Button("Open Docs", clicked_fn=self._open_docs)
                    ui.Button("Refresh Status", clicked_fn=self._refresh_status)

                ui.Spacer(height=8)
                ui.Label("Status")
                self._status_label = ui.Label(self._status_model.as_string, word_wrap=True)

    def _set_status(self, message: str):
        self._status_model.set_value(message)
        if self._status_label is not None:
            self._status_label.text = message
        carb.log_info(f"[MicroMani] {message}")

    def _root(self) -> Path:
        return Path(self._project_root_model.as_string).expanduser().resolve()

    def _stage_path(self) -> str:
        return self._stage_model.as_string.strip()

    def _ensure_tools_on_path(self):
        tools_dir = self._root() / "isaacsim_tools"
        if str(tools_dir) not in sys.path:
            sys.path.insert(0, str(tools_dir))
        self._tools_dir = tools_dir

    def _open_scene(self):
        path = self._stage_path()
        if not path:
            self._set_status("Scene path is empty.")
            return
        ok = omni.usd.get_context().open_stage(path)
        self._set_status(f"Open scene requested: {path}, ok={ok}")

    def _save_stage(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._set_status("No stage is open.")
            return
        stage.GetRootLayer().Save()
        self._set_status("Stage saved.")

    def _configure_cameras(self):
        self._ensure_tools_on_path()
        try:
            from micromani_configure_cameras import CAMERA_POSES, focal_length_for_view, set_world_camera_pose
            from pxr import Gf, UsdGeom

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                self._open_scene()
                stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise RuntimeError("no open stage")

            report = {}
            for path, spec in CAMERA_POSES.items():
                result = set_world_camera_pose(stage, path, spec["eye"], spec["target"])
                cam = UsdGeom.Camera.Get(stage, path)
                distance = sum((spec["target"][i] - spec["eye"][i]) ** 2 for i in range(3)) ** 0.5
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
                report[path] = result
            stage.GetRootLayer().Save()
            report_path = self._root() / "outputs" / "micromani_camera_config_report.json"
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            self._set_status(f"Configured cameras. Report: {report_path}")
        except Exception as exc:
            self._set_status(f"Camera configuration failed: {exc}")

    def _configure_force_torque(self):
        self._ensure_tools_on_path()
        try:
            from micromani_lerobot_schema import (
                FORCE_TORQUE_AXES,
                FORCE_TORQUE_SENSORS,
                HKVL36_DEFAULT_MODEL,
                hkvl36_axis_ranges,
                hkvl36_model_spec,
            )
            from pxr import Gf, Sdf, UsdGeom

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                self._open_scene()
                stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise RuntimeError("no open stage")

            model = hkvl36_model_spec(HKVL36_DEFAULT_MODEL)
            axis_ranges = hkvl36_axis_ranges(HKVL36_DEFAULT_MODEL)
            configured = []
            for sensor_name, spec in FORCE_TORQUE_SENSORS.items():
                sensor_xform = UsdGeom.Xform.Define(stage, Sdf.Path(spec["sensor_prim_path"]))
                sensor_prim = sensor_xform.GetPrim()
                sensor_xform.ClearXformOpOrder()
                sensor_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
                joint_prim = stage.GetPrimAtPath(spec["joint_prim_path"])
                if not joint_prim.IsValid():
                    raise RuntimeError(f"missing joint prim: {spec['joint_prim_path']}")

                metadata = {
                    "sensor_type": "six_axis_force_torque",
                    "model": HKVL36_DEFAULT_MODEL,
                    "vendor": "HKVTech",
                    "install_location": spec["mount"],
                    "joint_name": spec["joint_name"],
                    "joint_prim_path": spec["joint_prim_path"],
                    "mount_link_path": spec["mount_link_path"],
                    "sensor_prim_path": spec["sensor_prim_path"],
                    "data_source": "Articulation.get_measured_joint_forces(joint_index + 1)",
                    "reference_frame": spec["frame"],
                    "axes": list(FORCE_TORQUE_AXES),
                    "axis_units": ["N", "N", "N", "N*m", "N*m", "N*m"],
                    "axis_full_scale": axis_ranges,
                    "sample_fps_configured": int(self._fps_model.as_int),
                    "hardware_spec": model,
                }
                for prim in (sensor_prim, joint_prim):
                    for key, value in metadata.items():
                        if not isinstance(value, (str, int, float, bool)):
                            value = json.dumps(value, ensure_ascii=False)
                        prim.SetCustomDataByKey(f"micromani_{key}", value)
                configured.append({"name": sensor_name, **spec})
            stage.GetRootLayer().Save()

            report = {"model": HKVL36_DEFAULT_MODEL, "hardware_spec": model, "configured_sensors": configured}
            report_path = self._root() / "outputs" / "hkvl36_force_torque_sensors.json"
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            self._set_status(f"Configured HKVL-36 sensors. Report: {report_path}")
        except Exception as exc:
            self._set_status(f"HKVL-36 configuration failed: {exc}")

    def _start_bridge(self):
        self._ensure_tools_on_path()
        if self._bridge_module is not None:
            self._set_status("Control bridge is already loaded in this Isaac process.")
            return
        try:
            bridge_path = self._tools_dir / "micromani_control_bridge.py"
            spec = importlib.util.spec_from_file_location("micromani_control_bridge_runtime", bridge_path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            self._bridge_module = module
            omni.timeline.get_timeline_interface().play()
            self._set_status("Control bridge started on UDP 49010 and TCP 49011.")
        except Exception as exc:
            self._set_status(f"Control bridge failed: {exc}")

    def _stop_bridge(self):
        if self._bridge_module is None:
            self._set_status("Control bridge is not running from this extension.")
            return
        try:
            bridge = getattr(self._bridge_module, "BRIDGE", None)
            if bridge is not None and hasattr(bridge, "stop"):
                bridge.stop()
            self._bridge_module = None
            self._set_status("Control bridge stop requested.")
        except Exception as exc:
            self._set_status(f"Control bridge stop failed: {exc}")

    def _run_smoke_pipeline(self):
        root = self._root()
        script = root / "isaacsim_tools" / "run_micromani_lerobot_pipeline.sh"
        if not script.exists():
            self._set_status(f"Missing pipeline script: {script}")
            return
        if self._process and self._process.poll() is None:
            self._set_status("A pipeline process is already running.")
            return
        session = self._session_model.as_string.strip() or time.strftime("micromani_ui_%Y%m%d_%H%M%S")
        env = os.environ.copy()
        env["MICROMANI_PROJECT_ROOT"] = str(root)
        env["MICROMANI_FRAMES"] = str(max(1, self._frames_model.as_int))
        env["MICROMANI_FPS"] = str(max(1, self._fps_model.as_int))
        env["MICROMANI_SESSION_NAME"] = session
        env["MICROMANI_REPO_ID"] = self._repo_model.as_string.strip() or DEFAULT_REPO_ID
        env["MICROMANI_LOCAL_LEROBOT_DIR"] = str(root / "datasets" / "lerobot" / session)
        log_path = root / "outputs" / f"{session}_pipeline.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._process_log = log_path
        log_fp = log_path.open("w", encoding="utf-8")
        self._process = subprocess.Popen([str(script)], cwd=str(root), env=env, stdout=log_fp, stderr=subprocess.STDOUT)
        threading.Thread(target=self._watch_process, args=(session, log_fp), daemon=True).start()
        self._set_status(f"Started E2E smoke pipeline: {session}. Log: {log_path}")

    def _start_pico_gateway(self):
        root = self._root()
        script = root / "isaacsim_tools" / "micromani_pico_gateway.py"
        if not script.exists():
            self._set_status(f"Missing PICO gateway script: {script}")
            return
        if self._pico_process and self._pico_process.poll() is None:
            self._set_status("PICO gateway is already running.")
            return
        log_path = root / "outputs" / "micromani_pico_gateway.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._pico_log = log_path
        log_fp = log_path.open("w", encoding="utf-8")
        self._pico_process = subprocess.Popen(
            [sys.executable, str(script)],
            cwd=str(root),
            stdout=log_fp,
            stderr=subprocess.STDOUT,
        )
        threading.Thread(target=self._watch_pico_gateway, args=(log_fp,), daemon=True).start()
        self._set_status(f"Started PICO gateway on UDP 49100. Log: {log_path}")

    def _watch_pico_gateway(self, log_fp):
        code = self._pico_process.wait()
        log_fp.close()
        self._set_status(f"PICO gateway exited with code {code}. Log: {self._pico_log}")

    def _stop_pico_gateway(self):
        if self._pico_process is None or self._pico_process.poll() is not None:
            return
        self._pico_process.terminate()
        try:
            self._pico_process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._pico_process.kill()
        self._set_status("PICO gateway stop requested.")

    def _watch_process(self, session: str, log_fp):
        code = self._process.wait()
        log_fp.close()
        self._set_status(f"Pipeline finished for {session} with exit code {code}. Log: {self._process_log}")

    def _open_dataset_dir(self):
        self._open_path(self._root() / "datasets")

    def _open_docs(self):
        self._open_path(self._root() / "docs" / "MICROMANI_ISAACSIM_PLUGIN_AND_DATA_COLLECTION.md")

    def _open_path(self, path: Path):
        path = path.expanduser().resolve()
        if not path.exists():
            self._set_status(f"Path does not exist: {path}")
            return
        try:
            if sys.platform == "darwin":
                subprocess.Popen(["open", str(path)])
            elif os.name == "nt":
                os.startfile(str(path))
            else:
                subprocess.Popen(["xdg-open", str(path)])
            self._set_status(f"Opened: {path}")
        except Exception as exc:
            self._set_status(f"Open path failed: {exc}")

    def _refresh_status(self):
        root = self._root()
        files = [
            root / "outputs" / "micromani_pipeline_report.json",
            root / "outputs" / "micromani_replay_report.json",
            root / "docs" / "cloud_isaacsim_progress.md",
        ]
        existing = [str(path) for path in files if path.exists()]
        self._set_status("Existing reports:\n" + "\n".join(existing))
