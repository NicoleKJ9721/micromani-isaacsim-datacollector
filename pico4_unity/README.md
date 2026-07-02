# PICO-4 Unity Teleoperation Stub

This folder contains a minimal Unity/OpenXR sender for PICO-4 teleoperation.

## Network Flow

```text
PICO-4 Unity app
  -> UDP 49100
MicroMani PICO gateway
  -> UDP 49010 or TCP 49011
Isaac Sim MicroMani control bridge
  -> 14D robot action
```

## Unity Setup

1. Enable Unity XR/OpenXR for the PICO-4 project.
2. Add `MicroManiPicoTeleop.cs` to a GameObject in the scene.
3. Set `gatewayHost` to the Isaac Sim workstation or cloud instance address.
4. Keep `gatewayPort=49100` and `sendRateHz=30` unless you change the gateway.

The script sends generic OpenXR controller values:

- left/right `primary2DAxis`
- left/right `trigger`
- left/right `grip`
- primary button as reset
- left secondary button as open grippers
- right secondary button as close grippers

The gateway maps these values into the MicroMani 14D control order.
