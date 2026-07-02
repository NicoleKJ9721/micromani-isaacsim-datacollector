"""Map PICO/OpenXR teleoperation packets to MicroMani 14D control actions."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping

from micromani_lerobot_schema import CONTROL_ORDER, DEFAULT_HOME_ACTION, JOINT_LIMITS, clip_control_action


def _as_float(value, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        result = float(value)
        if math.isfinite(result):
            return result
    except Exception:
        pass
    return default


def _axis2(value) -> tuple[float, float]:
    if isinstance(value, Mapping):
        return _as_float(value.get("x")), _as_float(value.get("y"))
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return _as_float(value[0]), _as_float(value[1])
    return 0.0, 0.0


def _button(message: Mapping, name: str) -> bool:
    buttons = message.get("buttons") or {}
    return bool(buttons.get(name) or message.get(name))


@dataclass
class PicoTeleopMapper:
    """Stateful incremental mapper for VR-controller teleoperation.

    The mapper accepts either a direct ``{"axes": [...]}`` packet or a generic
    OpenXR-style packet with ``left`` and ``right`` controller dictionaries.
    """

    linear_step_m: float = 0.0012
    rotation_step_rad: float = 0.035
    gripper_step_m: float = 0.0008
    action: list[float] = field(default_factory=lambda: list(DEFAULT_HOME_ACTION))

    def reset(self) -> list[float]:
        self.action = list(DEFAULT_HOME_ACTION)
        return list(self.action)

    def map_packet(self, packet: Mapping) -> list[float]:
        if "axes" in packet:
            self.action = clip_control_action(packet["axes"])
            return list(self.action)

        if _button(packet, "reset") or _button(packet, "primaryButton"):
            return self.reset()

        left = packet.get("left") or {}
        right = packet.get("right") or {}
        next_action = list(self.action)
        self._apply_hand(next_action, "left", left)
        self._apply_hand(next_action, "right", right)

        if _button(packet, "openGrippers"):
            next_action[CONTROL_ORDER.index("left_gripper")] = JOINT_LIMITS["left_gripper"][1]
            next_action[CONTROL_ORDER.index("right_gripper")] = JOINT_LIMITS["right_gripper"][1]
        if _button(packet, "closeGrippers"):
            next_action[CONTROL_ORDER.index("left_gripper")] = JOINT_LIMITS["left_gripper"][0]
            next_action[CONTROL_ORDER.index("right_gripper")] = JOINT_LIMITS["right_gripper"][0]

        self.action = clip_control_action(next_action)
        return list(self.action)

    def _apply_hand(self, action: list[float], side: str, hand: Mapping) -> None:
        prefix = "left" if side == "left" else "right"
        stick_x, stick_y = _axis2(hand.get("thumbstick") or hand.get("primary2DAxis"))
        secondary_x, secondary_y = _axis2(hand.get("secondary2DAxis"))
        trigger = _as_float(hand.get("trigger") or hand.get("triggerButton"))
        grip = _as_float(hand.get("grip") or hand.get("gripButton"))

        joint_names = [
            f"joint1_{prefix}",
            f"joint2_{prefix}",
            f"joint3_{prefix}",
            f"joint4_{prefix}",
            f"joint5_{prefix}",
            f"joint6_{prefix}",
            f"{prefix}_gripper",
        ]
        idx = {name: CONTROL_ORDER.index(name) for name in joint_names}

        action[idx[f"joint1_{prefix}"]] += stick_y * self.linear_step_m
        action[idx[f"joint2_{prefix}"]] += stick_x * self.linear_step_m
        action[idx[f"joint3_{prefix}"]] += (trigger - grip) * self.linear_step_m
        action[idx[f"joint4_{prefix}"]] += secondary_x * self.rotation_step_rad
        action[idx[f"joint5_{prefix}"]] += secondary_y * self.rotation_step_rad * 0.5
        action[idx[f"joint6_{prefix}"]] += (trigger - 0.5) * self.rotation_step_rad * 0.5
        action[idx[f"{prefix}_gripper"]] += (trigger - grip) * self.gripper_step_m


def map_pico_packet(packet: Mapping, mapper: PicoTeleopMapper | None = None) -> list[float]:
    mapper = mapper or PicoTeleopMapper()
    return mapper.map_packet(packet)
