"""仿真数据边界：参照 AppStation，显式声明单位、侧序和标定差异。"""
from __future__ import annotations
import copy
import math
from micromani_lerobot_schema import CONTROL_ORDER, JOINT_LIMITS

LEGACY_VERSION = 'micromani.sim.operator_sides.v1'
VERSION = 'micromani.sim.operator_sides.v3'
CAMERA_KEYS = {'left_wrist': 'observation.images.wrist_left',
               'right_wrist': 'observation.images.wrist_right', 'global': 'observation.images.global'}
TRANSLATION_NAMES = ('x_um', 'y_um', 'z_um')
ROTATION_NAMES = {
    LEGACY_VERSION: ('roll_mdeg', 'pitch_mdeg', 'yaw_mdeg'),
    VERSION: ('rx_mdeg', 'rz_mdeg', 'ry_mdeg'),
}


def finite_vector(values, length):
    out = [float(v) for v in values]
    if len(out) != length or not all(math.isfinite(v) for v in out):
        raise ValueError(f'expected {length} finite values')
    return out


def validate_action(values):
    out = finite_vector(values, 14)
    for name, v in zip(CONTROL_ORDER, out):
        lo, hi = JOINT_LIMITS[name]
        if not lo <= v <= hi:
            raise ValueError(f'{name}: {v} outside [{lo}, {hi}]')
    return out


class SimContract:
    def __init__(self, config, *, version=VERSION):
        if version not in ROTATION_NAMES:
            raise ValueError(f'unsupported simulation contract {version!r}')
        self.version = version
        axes = (*TRANSLATION_NAMES, *ROTATION_NAMES[version], 'gripper_gap_mm')
        self.state_names = [f'{side}_{axis}' for side in ('left', 'right') for axis in axes]
        self.config = copy.deepcopy(config)
        try:
            side = config['operator_left_model_side']
            if side not in ('left', 'right'): raise ValueError('confirm operator/model side mapping')
            for s in ('left', 'right'):
                gap = config['gripper_zero_gap_mm'][s]
                if gap is None or not math.isfinite(float(gap)) or float(gap) < 0:
                    raise ValueError('calibrate gripper zero gap in mm')
                if config['gripper_open_sign'][s] not in (-1,1): raise ValueError('invalid gripper sign')
                signs = finite_vector(config['motion_sign'][s],6)
                if any(v not in (-1,1) for v in signs): raise ValueError('invalid motion sign')
                finite_vector(config['motion_zero_si'][s],6)
        except (KeyError, TypeError) as exc:
            raise ValueError('incomplete simulation calibration') from exc
        self.sides = [side, 'left' if side == 'right' else 'right']

    def metadata(self):
        return {'version': self.version, 'referenceContract': 'appstation.dual_arm.operator_sides.v2',
                'source': 'isaac_sim', 'sideOrder': 'operator_left_then_operator_right',
                'stateOrder': list(self.state_names), 'actionOrder': list(self.state_names),
                'units': {'translation':'um','rotation':'mdeg','gripper':'mm','force':'SI'},
                'calibration': copy.deepcopy(self.config),
                'actionSemantics': 'absolute_joint_target_for_next_simulation_step',
                'motionSemantics': 'joint_axes_relative_to_declared_zero_not_end_effector_pose',
                'forceSource': 'simulated_joint_reaction',
                'forceModel': {'name':'HKVL-36A','forceRangeN':30.,'torqueRangeNm':1.,'resolutionPercentFS':0.03,
                               'electronicNoiseSimulated':False,'hardwareCalibration':None}, 'forceFrame': 'model_joint6_child_frame',
                'cameraRoles': 'model_roles_not_swapped_with_numeric_sides',
                'clock': 'simulation_time', 'encoderPulses': 'unavailable',
                'gripForceSource': 'optional_simulated_finger_contact_mean_N',
                'hardwareCompatible': False}

    @classmethod
    def from_metadata(cls, metadata):
        contract = cls(metadata['calibration'], version=metadata['version'])
        contract.validate_metadata(metadata)
        return contract

    def validate_metadata(self, metadata):
        if metadata != self.metadata(): raise ValueError('simulation data contract mismatch')

    def to_dataset(self, values):
        raw = finite_vector(values,14); out=[]
        for s in self.sides:
            offset=0 if s=='left' else 7
            for i in range(6):
                q=(raw[offset+i]-self.config['motion_zero_si'][s][i])*self.config['motion_sign'][s][i]
                out.append(q*1e6 if i<3 else math.degrees(q)*1000)
            gap=float(self.config['gripper_zero_gap_mm'][s])+self.config['gripper_open_sign'][s]*raw[offset+6]*2000
            if gap<0: raise ValueError('negative calibrated gripper gap')
            out.append(gap)
        return out

    def to_model(self, values):
        raw=finite_vector(values,14); out=[0.]*14
        for side_index,s in enumerate(self.sides):
            offset=0 if s=='left' else 7
            for i in range(6):
                q=raw[side_index*7+i]
                q=q/1e6 if i<3 else math.radians(q/1000)
                out[offset+i]=q*self.config['motion_sign'][s][i]+self.config['motion_zero_si'][s][i]
            out[offset+6]=(raw[side_index*7+6]-self.config['gripper_zero_gap_mm'][s])/2000*self.config['gripper_open_sign'][s]
        # 浮点单位往返只允许边界舍入误差，不截断真正越界的指令。
        for i,name in enumerate(CONTROL_ORDER):
            lo,hi=JOINT_LIMITS[name]
            tolerance=2e-7*max(abs(lo),abs(hi),1e-3)
            if abs(out[i]-lo)<tolerance: out[i]=lo
            if abs(out[i]-hi)<tolerance: out[i]=hi
        return validate_action(out)

    def force_to_dataset(self, values):
        values=finite_vector(values,12)
        chunks={'left':values[:6],'right':values[6:]}
        return chunks[self.sides[0]],chunks[self.sides[1]]

    def features(self, image_dtype='image', include_grip=False):
        if image_dtype not in ('image','video'): raise ValueError('invalid image dtype')
        features={k:{'dtype':'float32','shape':(14,),'names':list(self.state_names)} for k in ('observation.state','action')}
        for side in ('left','right'):
            features[f'observation.force_{side}']={'dtype':'float32','shape':(6,),'names':['fx','fy','fz','mx','my','mz']}
        features.update({k:{'dtype':image_dtype,'shape':(480,640,3),'names':['height','width','channels']} for k in CAMERA_KEYS.values()})
        if include_grip:
            features['observation.grip_force']={'dtype':'float32','shape':(2,),'names':['operator_left_N','operator_right_N']}
        return features
