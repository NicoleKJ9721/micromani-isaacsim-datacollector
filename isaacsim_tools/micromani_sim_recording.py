"""仿真录制契约与 episode 生命周期；不依赖 Isaac Kit。"""
from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from pathlib import Path

from micromani_lerobot_schema import CONTROL_ORDER, JOINT_LIMITS

VERSION = 'micromani.sim.operator_sides.v2'
CAMERAS = {'global': 'global', 'left_wrist': 'wrist_left', 'right_wrist': 'wrist_right'}


def vector(values, size):
    result = [float(v) for v in values]
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f'expected {size} finite values')
    return result


def profile_metadata(operator_left_model_side='right', zero_gap_mm=(0., 0.)):
    if operator_left_model_side not in ('left', 'right'):
        raise ValueError('operator left model side must be left or right')
    gaps = vector(zero_gap_mm, 2)
    if min(gaps) < 0:
        raise ValueError('zero gaps must be nonnegative')
    other = 'left' if operator_left_model_side == 'right' else 'right'
    return {
        'version': VERSION,
        'referenceContract': 'appstation.dual_arm.operator_sides.v2',
        'source': 'isaac_sim',
        'modelSideForOperatorSide': {'left': operator_left_model_side, 'right': other},
        'units': {'translation': 'um', 'rotation': 'mdeg', 'gripper': 'mm', 'force': 'SI'},
        'motionSemantics': 'URDF joint coordinates; not calibrated hardware Cartesian coordinates',
        'stateOrder': [f'operator_{side}.{axis}' for side in ('left', 'right')
                       for axis in ('joint1_um', 'joint2_um', 'joint3_um', 'joint4_mdeg',
                                    'joint5_mdeg', 'joint6_mdeg', 'gripper_gap_mm')],
        'gripper': {'zeroGapMmModelLeftRight': gaps, 'gapFormula': 'zero_gap_mm + 2000 * finger_joint_m'},
        'force': {'source': 'simulated_joint_reaction', 'frame': 'joint6 child joint frame',
                  'sideOrder': 'operator', 'calibration': None},
        'cameras': 'physical model roles; not swapped with numeric operator sides',
        'pulses': 'not available; omitted',
        'gripForce': {'unit': 'N', 'order': 'operator_left_then_right',
                      'source': 'simulated_finger_contact', 'definition': 'mean_of_two_finger_contact_magnitudes',
                      'missing': 'unavailable; never filled with synthetic zeros'},
        'clock': 'simulation time; synchronous render at physics step boundary',
    }


def validate_contract(contract):
    if not isinstance(contract, dict):
        raise ValueError('missing simulation data contract')
    try:
        expected = profile_metadata(contract['modelSideForOperatorSide']['left'],
                                    contract['gripper']['zeroGapMmModelLeftRight'])
    except (KeyError, TypeError) as exc:
        raise ValueError('invalid simulation data contract') from exc
    if contract != expected:
        raise ValueError('unknown or incompatible simulation data contract')


def to_dataset(values, contract):
    validate_contract(contract)
    values = vector(values, 14)
    result = []
    for side in ('left', 'right'):
        i = 0 if contract['modelSideForOperatorSide'][side] == 'left' else 7
        gap = contract['gripper']['zeroGapMmModelLeftRight'][i // 7]
        result.extend(v * 1e6 for v in values[i:i+3])
        result.extend(math.degrees(v) * 1000 for v in values[i+3:i+6])
        result.append(gap + 2000 * values[i+6])
    return result


def from_dataset(values, contract):
    validate_contract(contract)
    values = vector(values, 14)
    result = [0.] * 14
    for j, side in enumerate(('left', 'right')):
        i = 0 if contract['modelSideForOperatorSide'][side] == 'left' else 7
        gap = contract['gripper']['zeroGapMmModelLeftRight'][i // 7]
        v = values[j*7:j*7+7]
        result[i:i+7] = [*(x / 1e6 for x in v[:3]),
                         *(math.radians(x / 1000) for x in v[3:6]), (v[6]-gap)/2000]
    # LeRobot float32 在单位往返后可能略微越过边界；只容纳舍入误差。
    for i, name in enumerate(CONTROL_ORDER):
        lo, hi = JOINT_LIMITS[name]
        if lo - 2e-7 <= result[i] <= hi + 2e-7:
            result[i] = min(hi, max(lo, result[i]))
    validate_action(result)
    return result


def validate_action(values):
    values = vector(values, 14)
    for name, value in zip(CONTROL_ORDER, values):
        lo, hi = JOINT_LIMITS[name]
        if not lo - 1e-9 <= value <= hi + 1e-9:
            raise ValueError(f'{name} outside joint limits')
    return values


class SimSession:
    """保存/丢弃后必须复位；失败帧不能保存，旧会话不自动续写。"""
    def __init__(self, root, contract, fps=30):
        validate_contract(contract)
        if fps <= 0 or int(fps) != fps:
            raise ValueError('fps must be a positive integer')
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.contract = deepcopy(contract)
        self.fps = int(fps)
        self.phase = 'idle'
        self.episodes = []
        self.frames = []
        self._write_manifest()

    def _write_manifest(self):
        manifest = {'schema_version': VERSION, 'dataContract': self.contract, 'fps': self.fps,
                    'phase': self.phase, 'episodes': self.episodes}
        tmp = self.root / 'manifest.json.tmp'
        tmp.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
        tmp.replace(self.root / 'manifest.json')

    def start(self, task):
        if self.phase != 'idle' or not isinstance(task, str) or not task.strip():
            raise ValueError('start requires idle phase and a task')
        self.started_monotonic = time.monotonic()
        self.task = task.strip()
        self.frames = []
        self.phase = 'recording'
        self._write_manifest()

    def add(self, state, action, force, images, sim_time, action_time, grip_force=None):
        if self.phase != 'recording':
            raise ValueError('not recording')
        try:
            state = vector(state, 14)
            action = validate_action(action)
            force = vector(force, 12)
            if not all(math.isfinite(t) for t in (sim_time, action_time)):
                raise ValueError('invalid source time')
            if not 0 <= sim_time - action_time <= 0.25:
                raise ValueError('missing, future or stale action')
            if self.frames and not math.isclose(sim_time-self.frames[-1]['sim_time'], 1/self.fps, abs_tol=1e-6):
                raise ValueError('non-contiguous simulation frames')
            if set(images) != set(CAMERAS):
                raise ValueError('three camera frames required')
            for path in images.values():
                image_path = (self.root / path).resolve()
                if not image_path.is_relative_to(self.root.resolve()) or not image_path.is_file():
                    raise ValueError('missing image or image outside session')
            row = {'frame_index': len(self.frames), 'timestamp': len(self.frames)/self.fps,
                   'sim_time': sim_time, 'action_time': action_time,
                   'observation.state': to_dataset(state, self.contract),
                   'action': to_dataset(action, self.contract), 'images': dict(images), 'task': self.task}
            for side in ('left', 'right'):
                i = 0 if self.contract['modelSideForOperatorSide'][side] == 'left' else 6
                row[f'observation.force_{side}'] = force[i:i+6]
            if grip_force is not None:
                grip = vector(grip_force, 2)
                if min(grip) < 0: raise ValueError('negative gripping force')
                row['observation.grip_force'] = [grip[0 if self.contract['modelSideForOperatorSide'][side] == 'left' else 1] for side in ('left','right')]
            self.frames.append(row)
        except Exception:
            self.phase = 'interrupted'
            self._write_manifest()
            raise

    def save(self, outcome='unreviewed'):
        if self.phase != 'recording' or not self.frames:
            raise ValueError('only a nonempty valid recording can be saved')
        if outcome not in ('unreviewed', 'success', 'failure'):
            raise ValueError('invalid outcome')
        index = len(self.episodes)
        path = self.root / f'episode_{index:06d}.jsonl'
        path.write_text(''.join(json.dumps(row, ensure_ascii=False)+'\n' for row in self.frames), encoding='utf-8')
        self.episodes.append({'index': index, 'frames': len(self.frames), 'path': path.name,
                              'task': self.task, 'outcome': outcome, 'source': 'isaac_sim',
                              'simulationDurationS': len(self.frames)/self.fps,
                              'wallElapsedS': time.monotonic()-self.started_monotonic,
                              'quality': {'contiguousSimulationFrames': True, 'realTimeFpsVerified': False}})
        self.frames = []
        self.phase = 'reset_pending'
        self._write_manifest()

    def discard(self):
        if self.phase not in ('recording', 'interrupted'):
            raise ValueError('no recording to discard')
        self.frames = []
        self.phase = 'reset_pending'
        self._write_manifest()

    def reset(self):
        if self.phase != 'reset_pending':
            raise ValueError('reset is only allowed between episodes')
        self.phase = 'idle'
        self._write_manifest()

    def finish(self):
        if self.phase not in ('idle', 'reset_pending'):
            raise ValueError('save or discard current episode first')
        self.phase = 'finished'
        self._write_manifest()
