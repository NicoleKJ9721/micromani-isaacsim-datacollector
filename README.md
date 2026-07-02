# MicroManiURDF Isaac Sim Data Collector

This repository contains the MicroMani bimanual micromanipulator URDF assets, Isaac Sim setup scripts, a lightweight Isaac Sim extension, and a LeRobot-compatible virtual operation data collection pipeline.

## What Is Included

- URDF and mesh assets for the bimanual micromanipulator.
- Isaac Sim import/configuration scripts.
- Three camera configuration: left wrist, right wrist, and global camera.
- HKVL-36A six-axis force/torque sensor configuration at both `joint6` centers.
- UDP/TCP control bridge for 14-axis external control.
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
- [Cloud Isaac Sim Progress](docs/cloud_isaacsim_progress.md)
- [LeRobot Collection Plan](docs/micromani_lerobot_collection_plan.md)

## Current Verified Smoke Test

- Session: `micromani_ft_e2e_20260703_01`
- Frames: 6
- FPS: 30
- `observation.state`: 14D
- `action`: 14D
- `observation.force_torque`: 12D
- Cameras: three `3x480x640` tensors after LeRobot reload
- Replay: 6 frames sent, 6 ACKs, no replay errors
