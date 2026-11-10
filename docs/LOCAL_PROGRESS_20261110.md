# Local simulation status — 2026-11-10

This update includes the configurable 1200 × 800 mm breadboard, outward arm offsets,
hardware appearance references, three-camera monitor, public sensor/control HTTP
API, coupled per-arm gripper controls, bounded joint trajectories, recording/export,
and assembly contact configuration. See SIM_API.md, CAMERA_MONITOR.md,
PLATFORM_LAYOUT.md and ASSEMBLY_PHYSICS.md for interfaces and limitations.

The runtime STOP implementation holds measured arm positions while retaining the
already-applied gripper references. It does not continue toward an unapplied future
gripper target. The API advertises this behavior through `stop_semantics`.

## Verification and limitations

- Local delivery checks and all 40 discovered offline simulation tests passed.
- Local left-gripper experiment established table-supported bilateral contact:
  approximately 6.47–6.62 mN over 61 fresh observations spanning 2 simulated seconds.
  This does not establish successful lifting or assembly.
- Complete cavity/sleeve assembly and repeatability have **not** been achieved.
- Camera sampling is 30 Hz in simulation time, not a guarantee of real-time display.
  A local measurement was about 3.1 FPS at 0.10× real time; a recent profiling window
  averaged 293 ms physics, 9 ms control and 13 ms rendering per camera group.
- TPU friction values are estimates, not hardware calibration. The baseline uses
  rigid contact; TPU deformation has not been validated.
- Generated USD scenes, recordings, experiment output and machine-specific runtime
  state are excluded from Git. Experimental Astra controllers in the external task
  workspace are not packaged as a production autonomous assembly controller here.
- Material-table and part CAD assets supplied outside this repository must be
  provided separately when reconstructing the local assembly scene.
