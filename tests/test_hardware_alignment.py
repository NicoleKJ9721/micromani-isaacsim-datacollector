"""Regressions for hardware labels and the simulated contact observations."""
import json
import math
import tempfile
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaacsim_tools'))
from micromani_grip_sensing import read_grip_sensors
import micromani_configure_cameras as camera_config
from micromani_sim_contract import SimContract, LEGACY_VERSION, validate_action
from micromani_lerobot_schema import CONTROL_ORDER, GRIPPER_MAP, JOINT_LIMITS, quantize_hkvl36_wrench
from micromani_episode import EpisodeSession
from test_sim_recording import config


class HardwareAlignmentTests(unittest.TestCase):
    def test_rotation_labels_follow_model_joint_axes_and_legacy_remains_readable(self):
        urdf = Path(__file__).resolve().parents[1] / 'urdf/微纳机器人URDF.urdf'
        joints = {joint.attrib['name']: joint for joint in ElementTree.parse(urdf).getroot().findall('joint')}
        contract = SimContract(config())
        for side in ('left', 'right'):
            names = CONTROL_ORDER[3:6] if side == 'left' else CONTROL_ORDER[10:13]
            roles = []
            for name in names:
                axis = [abs(float(value)) for value in joints[name].find('axis').attrib['xyz'].split()]
                roles.append('xyz'[axis.index(1.0)])
            self.assertEqual(roles, ['x', 'z', 'y'])
            self.assertEqual([name for name in contract.metadata()['stateOrder'] if name.startswith(side) and name.endswith('mdeg')],
                             [f'{side}_r{axis}_mdeg' for axis in roles])
        legacy = SimContract(config(), version=LEGACY_VERSION)
        self.assertEqual(legacy.metadata()['stateOrder'][4:6], ['left_pitch_mdeg', 'left_yaw_mdeg'])
        raw = [.0]*14
        raw[4], raw[5] = .1, -.2
        self.assertEqual(legacy.to_dataset(raw), contract.to_dataset(raw))
        self.assertAlmostEqual(contract.to_dataset(raw)[11], math.degrees(.1)*1000)

    def test_motion_limits_match_source_urdf(self):
        urdf = Path(__file__).resolve().parents[1] / 'urdf/微纳机器人URDF.urdf'
        joints = {joint.attrib['name']: joint for joint in ElementTree.parse(urdf).getroot().findall('joint')}
        for name, limits in JOINT_LIMITS.items():
            for dof in GRIPPER_MAP.get(name, (name,)):
                limit = joints[dof].find('limit')
                self.assertEqual(limits, tuple(float(limit.attrib[key]) for key in ('lower', 'upper')))

    def test_contact_samples_are_read_once_per_finger(self):
        reads = []
        values = {'left1': 2., 'left2': 4., 'right1': 1., 'right2': 0.}

        def read(path, *, use_latest_data):
            self.assertTrue(use_latest_data)
            reads.append(path)
            return SimpleNamespace(is_valid=True, value=values[path])

        sensors = {(side, i): SimpleNamespace(prim_path=f'{side}{i}',
                   _contact_sensor_interface=SimpleNamespace(get_sensor_reading=read))
                   for side in ('left', 'right') for i in (1, 2)}
        result = read_grip_sensors(sensors)
        self.assertCountEqual(reads, values.keys())
        self.assertEqual(result['left']['grip_force_n'], 3.)
        self.assertTrue(result['left']['bilateral_contact'])
        self.assertFalse(result['right']['bilateral_contact'])

    def test_hkvl36a_quantization_saturation_and_nonfinite_rejection(self):
        wrench, saturated = quantize_hkvl36_wrench([.01, -.01, 31., .0002, -.0002, -1.1])
        self.assertTrue(saturated)
        for measured, expected in zip(wrench, [.009, -.009, 30., .0003, -.0003, -1.]):
            self.assertAlmostEqual(measured, expected)
        for bad in (math.nan, math.inf, -math.inf):
            with self.assertRaisesRegex(ValueError, 'finite wrench'):
                quantize_hkvl36_wrench([bad, 0., 0., 0., 0., 0.])

    def test_all_logical_axes_accept_bounds_and_reject_overtravel(self):
        for i, name in enumerate(CONTROL_ORDER):
            for limit, direction in zip(JOINT_LIMITS[name], (-1, 1)):
                values = [0.]*14
                values[i] = limit
                self.assertEqual(validate_action(values)[i], limit)
                values[i] += direction * 1e-5
                with self.assertRaisesRegex(ValueError, name):
                    validate_action(values)

    def test_contact_provenance_distinguishes_one_finger_from_both(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'session'
            session = EpisodeSession(root, SimContract(config()), fps=30, task='test', stage='fixture')
            session.start()
            cameras = ('left_wrist', 'right_wrist', 'global')
            session.append(sim_time=1., state=[0.]*14, action=[0.]*14, force=[0.]*12,
                           images={name: b'fixture' for name in cameras},
                           source_times={name: 1. for name in ('state', 'force', *cameras)},
                           action_time=1., action_source='fixture', finger_contact_n=[2., 0., 3., 5.])
            row = json.loads((root / 'episode_000000/frames.jsonl').read_text())
            self.assertEqual(row['observation_grip_force'], [1., 4.])
            self.assertEqual(row['finger_contact_n'], [2., 0., 3., 5.])
            self.assertEqual(row['grip_bilateral_contact'], [False, True])

    def test_wrist_work_target_is_between_inward_fingertips(self):
        import numpy as np
        first = np.array([[-.030, -.010, .001], [.004, -.004, 0.], [.004, -.002, .002]])
        second = np.array([[-.030, .010, .003], [.004, .002, 0.], [.004, .004, .002]])
        np.testing.assert_allclose(camera_config.wrist_work_target(first, second, 1), [.005, 0., .001])
        first[:, 0] *= -1
        second[:, 0] *= -1
        np.testing.assert_allclose(camera_config.wrist_work_target(first, second, -1), [-.005, 0., .001])
