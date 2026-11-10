import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'isaacsim_tools'))
from micromani_collect_isaac import read_force_torque

class ForceIndexTests(unittest.TestCase):
    def test_fixed_joint_indices_not_dof_order(self):
        class Robot:
            _metadata=SimpleNamespace(joint_indices={'joint6_left':8,'joint6_right':17})
            def get_measured_joint_forces(self, joint_indices):
                np.testing.assert_array_equal(joint_indices,[9,18])
                return np.zeros((1,2,6))
        readings,values=read_force_torque(Robot(),{'joint6_left':5,'joint6_right':13},'HKVL-36A')
        self.assertEqual(readings['left_wrist']['joint_row_index'],9)
        self.assertEqual(values,[0.]*12)

    def test_missing_metadata_fails_instead_of_guessing(self):
        with self.assertRaises(RuntimeError): read_force_torque(object(),{},'HKVL-36A')
