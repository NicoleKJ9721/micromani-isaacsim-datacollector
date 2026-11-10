"""Contact-phase guard over public sensor messages, never simulator truth.

Thresholds are exploratory simulation settings, not TPU allowable loads.
Wrist residual must be computed from a previously fitted free-air calibration;
raw gravity-containing wrist force is not a contact estimate.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class ContactLimits:
    bilateral_min_n: float = .003
    finger_max_n: float = .05
    unilateral_max_n: float = .02
    wrist_residual_max_n: float = .10
    stable_samples: int = 5
    # Optional contact-presence threshold, independent of the force goal.
    unilateral_contact_min_n: float | None = None

class ContactGuard:
    def __init__(self, side, limits=None, holding_other=False):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        self.side = side
        self.holding_other = holding_other
        self.other_loss_samples = 0
        self.limits = limits or ContactLimits()
        threshold=self.limits.unilateral_contact_min_n
        if threshold is not None and not (0 < threshold <= self.limits.bilateral_min_n):
            raise ValueError('unilateral contact threshold must be positive and no greater than confirmation target')
        self.last_sequence = None
        self.last_time = None
        self.consecutive = 0
        self.fault = None

    def evaluate(self, state, wrist_residual_n):
        """Return stop reason or bilateral confirmation; duplicates never confirm."""
        def stop(reason):
            self.consecutive = 0
            self.fault = reason
            return {'stop': True, 'reason': reason, 'bilateral_confirmed': False}
        if self.fault is not None:
            return stop(self.fault)
        if state.get('stale', True):
            return stop('stale_sensor_state')
        try:
            seq = state['sequence']
            if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
                return stop('invalid_sequence')
            stamp = float(state['simulation_time'])
            force = [float(v) for v in state['grip'][self.side]['finger_contact_n']]
            wrist = float(wrist_residual_n)
            if len(force) != 2 or not all(math.isfinite(v) and v >= 0 for v in force + [wrist, stamp]):
                return stop('invalid_sensor_value')
            if any(w['saturated'] for w in state['force_torque'].values()):
                return stop('saturated_wrist_sensor')
        except (KeyError, TypeError, ValueError):
            return stop('missing_or_invalid_sensor_value')
        other = 'right' if self.side == 'left' else 'left'
        try:
            other_forces = state['grip'][other]['finger_contact_n']
            if len(other_forces) != 2 or any(not math.isfinite(v) or v < 0 for v in other_forces):
                return stop('invalid_other_gripper_sensor')
            if self.holding_other:
                if max(other_forces) > .05:
                    return stop('held_other_gripper_force_limit')
            elif max(other_forces) > .001:
                return stop('unexpected_other_gripper_contact')
        except (KeyError, TypeError, ValueError):
            return stop('missing_other_gripper_sensor')
        lim = self.limits
        if max(force) > lim.finger_max_n:
            return stop('finger_force_limit')
        contact_min=lim.bilateral_min_n if lim.unilateral_contact_min_n is None else lim.unilateral_contact_min_n
        if min(force) < contact_min and max(force) > lim.unilateral_max_n:
            return stop('unilateral_force_limit')
        if wrist > lim.wrist_residual_max_n:
            return stop('wrist_residual_limit')
        if self.last_sequence is not None:
            if seq < self.last_sequence or stamp < self.last_time:
                return stop('sensor_clock_regressed')
            if seq == self.last_sequence or stamp == self.last_time:
                return {'stop': False, 'reason': 'duplicate_sample', 'bilateral_confirmed': False}
        self.last_sequence, self.last_time = seq, stamp
        if self.holding_other:
            self.other_loss_samples = self.other_loss_samples + 1 if min(other_forces) < .001 else 0
            if self.other_loss_samples >= 3:
                return stop('lost_other_gripper_contact')
        self.consecutive = self.consecutive + 1 if min(force) >= lim.bilateral_min_n else 0
        return {'stop': False, 'reason': 'fresh_sample',
                'bilateral_confirmed': self.consecutive >= lim.stable_samples,
                'consecutive_samples': self.consecutive}
