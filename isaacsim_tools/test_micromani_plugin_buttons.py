#!/usr/bin/env python3
"""Exercise the MicroMani Isaac Sim extension button handlers in headless Kit."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp


DEFAULT_STAGE = "/isaac-sim/.local/share/ov/data/micromani/micromanipulator_scene.usd"
DEFAULT_EXTENSION_PATH = "/isaac-sim/.local/share/ov/data/micromani/exts/micromani.data_collector"


def main() -> int:
    kit = SimulationApp({"headless": True})

    stage_path = os.environ.get("MICROMANI_STAGE", DEFAULT_STAGE)
    extension_path = Path(os.environ.get("MICROMANI_EXTENSION_PATH", DEFAULT_EXTENSION_PATH))
    report_path = Path(
        os.environ.get(
            "MICROMANI_PLUGIN_REPORT",
            "/isaac-sim/.local/share/ov/data/micromani/plugin_button_report.json",
        )
    )
    if str(extension_path) not in sys.path:
        sys.path.insert(0, str(extension_path))

    from micromani.data_collector.extension import Extension  # noqa: PLC0415

    extension = Extension()
    result = {"stage": stage_path, "extension_path": str(extension_path)}
    try:
        extension.on_startup("micromani.data_collector-button-test")
        extension._toggle_window()
        for _ in range(5):
            kit.update()
        result["ui_visible"] = bool(extension._window.visible)
        result["ui_status"] = extension._status_model.as_string

        extension._stage_model.set_value(stage_path)
        extension._open_scene()
        for _ in range(20):
            kit.update()
        result["open_scene_status"] = extension._status_model.as_string

        extension._configure_cameras()
        result["configure_cameras_status"] = extension._status_model.as_string
        extension._configure_force_torque()
        result["configure_ft_status"] = extension._status_model.as_string
        extension._refresh_status()
        result["refresh_status"] = extension._status_model.as_string

        extension._start_pico_gateway()
        time.sleep(1.0)
        result["pico_running"] = extension._pico_process is not None and extension._pico_process.poll() is None
        extension._stop_pico_gateway()
        time.sleep(0.5)
        result["pico_stopped"] = extension._pico_process is None or extension._pico_process.poll() is not None
        result["ok"] = True
    except Exception as exc:  # pragma: no cover - exercised inside Isaac Sim.
        result["ok"] = False
        result["error"] = repr(exc)
    finally:
        try:
            extension.on_shutdown()
        except Exception as exc:  # pragma: no cover - exercised inside Isaac Sim.
            result["shutdown_error"] = repr(exc)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("MICROMANI_PLUGIN_BUTTON_REPORT", json.dumps(result, ensure_ascii=False), flush=True)
        kit.close()

    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
