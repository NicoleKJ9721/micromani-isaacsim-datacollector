# MicroMani Isaac Sim + LeRobot Data Collection Plan

## Goal

Build a closed-loop embodied-operation data collection workflow for the virtual bimanual micromanipulator in Isaac Sim, while keeping the dataset compatible with LeRobot v3 conventions.

## Data Contract

- Observation state: `observation.state`, 14 float32 values.
- Action: `action`, 14 float32 values.
- Cameras:
  - `observation.images.left_wrist`, 640x480 RGB, 30 FPS.
  - `observation.images.right_wrist`, 640x480 RGB, 30 FPS.
  - `observation.images.global`, 640x480 RGB, 30 FPS.
- Force/torque:
  - `observation.force_torque`, 12 float32 values.
  - Order: left wrist `Fx,Fy,Fz,Mx,My,Mz`, then right wrist `Fx,Fy,Fz,Mx,My,Mz`.
  - Sidecar files: `force_torque/episode_000000.jsonl`, `force_torque/episode_000000.csv`, and `force_torque/metadata.json`.
- Effective control axes:
  - 6 left arm axes + 1 merged left gripper axis.
  - 6 right arm axes + 1 merged right gripper axis.
- Isaac internal actuation still expands the two merged gripper axes to the four raw finger joints.

## HKVL-36 Force/Torque Sensors

- Model used in simulation: `HKVL-36A`.
- Mounts: `joint6_left` center and `joint6_right` center.
- Isaac data source: `Articulation.get_measured_joint_forces(joint_index + 1)`.
- Reference frame: local child joint frame of each joint6.
- Full scale: `Fx/Fy/Fz = +/-30 N`, `Mx/My/Mz = +/-1 Nm`.
- Resolution model: `0.03% FS` quantization.
- Accuracy and repeatability are recorded as metadata: `0.5% FS` accuracy, `0.1% FS` repeatability.
- Hardware metadata also records 36 mm diameter, 21 mm height, 0.11 kg mass, 300% FS overload, 150-1000 Hz customizable sample frequency, and RS485/CAN-FD/EtherCAT interfaces.

## Reused LeRobot Pattern

The workflow follows LeRobot's record loop:

1. Read robot observation.
2. Generate or receive an action.
3. Send the action to the robot or simulator.
4. Save the observation/action pair and task label into a LeRobot dataset.

For stability, Isaac Sim only records raw episode files. The local Mac converts those raw files into a LeRobot v3 dataset using `LeRobotDataset.create`, `add_frame`, and `save_episode`.

## Test Plan

1. Isaac camera test: verify all three camera streams return non-empty RGB images.
2. Isaac control test: verify 14D control expands to the 16 raw Isaac DOFs and stays inside URDF limits.
3. Force/torque sensor test: verify both joint6 sensors produce 6D finite readings and stay inside HKVL-36A range.
4. Raw episode test: collect a short scripted closed-loop episode with visual, state, action, and force/torque data.
5. LeRobot conversion test: convert raw episode to LeRobot v3 and reload it locally.
6. Replay test: replay LeRobot `action` frames through the TCP bridge into Isaac.
7. Joint test: run the whole pipeline from capture to replay and write reports under `outputs/`.

## Current Scripts

- `isaacsim_tools/micromani_lerobot_schema.py`: shared axis order, gripper mapping, camera specs, and LeRobot features.
- `isaacsim_tools/micromani_collect_isaac.py`: Isaac-side raw episode collection.
- `isaacsim_tools/micromani_configure_ft_sensors.py`: write HKVL-36 metadata into the USD scene.
- `isaacsim_tools/micromani_convert_to_lerobot.py`: local raw-to-LeRobot conversion and validation.
- `isaacsim_tools/micromani_replay_lerobot.py`: LeRobot action replay to the Isaac TCP bridge.
- `isaacsim_tools/micromani_configure_cameras.py`: re-apply camera poses to the USD scene.
- `isaacsim_tools/run_micromani_lerobot_pipeline.sh`: end-to-end smoke pipeline.

## Verified Result

Final smoke session: `micromani_ft_e2e_20260703_01`.

- Raw episode: `datasets/raw/micromani_ft_e2e_20260703_01`.
- LeRobot dataset: `datasets/lerobot/micromani_ft_e2e_20260703_01`.
- Frames: 6.
- FPS: 30.
- `observation.state`: 14 values.
- `action`: 14 values.
- `observation.force_torque`: 12 values.
- Cameras reload as `3x480x640` tensors.
- Force/torque sidecars: JSONL, CSV, and metadata files.
- Saturation counts: 0 for both left and right HKVL-36A sensors in the final smoke session.
- Replay to Isaac control bridge: 6 frames sent, 6 acks, no replay errors.

## Notes Before Scaling

- The TCP bridge is reliable from the instance localhost. Public `49011` accepts a socket but did not return bridge acknowledgements, so the replay script and pipeline use an SSH local tunnel to `127.0.0.1:49011`.
- Wrist cameras are now mounted under `link6_left` and `link6_right` with world-targeted poses converted to link-local transforms. They capture the gripper region, but the images are dark and partially occluded because of current robot material and close camera placement. Improve lighting/materials and fine-tune extrinsics before large-scale dataset collection.
- The current action source is a scripted closed-loop policy. The future PICO-4 path should feed the same 14D `axes` packet used by the bridge and dataset schema.
- The force/torque stream is synchronized at 30 Hz with the current control/camera loop. HKVL-36 hardware supports higher sampling, but the current dataset stores the policy-usable synchronized force observation.
