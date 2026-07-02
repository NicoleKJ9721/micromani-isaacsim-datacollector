# MicroMani Data Collector Extension

This Isaac Sim extension provides a compact UI for the MicroMani bimanual micromanipulator workflow.

Use it from Isaac Sim with:

```bash
./isaac-sim.sh --ext-folder /path/to/MicroManiURDF/exts --enable micromani.data_collector
```

The window is available from `Tools > MicroMani > MicroMani Data Collector`.

Main controls:

- Open Scene
- Configure Cameras
- Configure HKVL-36 force/torque sensors
- Start / Stop UDP and TCP control bridge
- Run an end-to-end smoke data collection pipeline
- Open dataset and documentation directories
