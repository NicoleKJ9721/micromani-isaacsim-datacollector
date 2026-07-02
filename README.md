# MicroManiURDF Isaac Sim Data Collector

This repository contains the MicroMani bimanual micromanipulator URDF assets, Isaac Sim setup scripts, a lightweight Isaac Sim extension, and a LeRobot-compatible virtual operation data collection pipeline.

## What Is Included

- URDF and mesh assets for the bimanual micromanipulator.
- Isaac Sim import/configuration scripts.
- Three camera configuration: left wrist, right wrist, and global camera.
- HKVL-36A six-axis force/torque sensor configuration at both `joint6` centers.
- UDP/TCP control bridge for 14-axis external control.
- PICO-4/OpenXR UDP gateway and Unity sender stub for VR teleoperation.
- LeRobot v3 dataset conversion with visual, state, action, and force/torque observations.
- Isaac Sim extension: `micromani.data_collector`.

## Quick Start

Launch Isaac Sim with the extension search path:

```bash
/path/to/isaac-sim.sh --ext-folder /path/to/MicroManiURDF/exts --enable micromani.data_collector
```

Then open `Tools > MicroMani > MicroMani Data Collector`.

See the full guide:

- [Plugin, Configuration, and Data Collection Guide](docs/MICROMANI_ISAACSIM_PLUGIN_AND_DATA_COLLECTION.md)
- [Quality Assessment](docs/QUALITY_ASSESSMENT.md)
- [Cloud Isaac Sim Progress](docs/cloud_isaacsim_progress.md)
- [LeRobot Collection Plan](docs/micromani_lerobot_collection_plan.md)
- [Functional Test Report 2026-07-03](docs/FUNCTIONAL_TEST_REPORT_20260703.md)

Run local delivery checks:

```bash
scripts/run_local_checks.sh
```

## Current Verified Build And Data Retest

- Build report: `outputs/micromani_build_validation_20260703.json`, `build_success=true`
- Data session: `micromani_build_verify_20260703_01`
- Scope: open current primary USD -> initialize articulation physics -> command grippers -> render cameras -> read force/torque -> collect raw data -> convert LeRobot -> replay actions
- Frames: 6
- FPS: 30
- `observation.state`: 14D
- `action`: 14D
- `observation.force_torque`: 12D
- Cameras: three `3x480x640` tensors after LeRobot reload
- Replay: 6 frames sent, 6 ACKs, no replay errors
- Extension: cloud Isaac Sim 5.1 headless button-path smoke test passed
- PICO gateway: local UDP forwarding and cloud plugin start/stop smoke tests passed

## PICO-4 Teleoperation Stub

- Unity sender: `pico4_unity/MicroManiPicoTeleop.cs`
- Gateway: `isaacsim_tools/micromani_pico_gateway.py`
- Default flow: PICO-4 Unity app -> UDP `49100` -> gateway -> Isaac bridge UDP `49010`
