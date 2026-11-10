import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaacsim_tools'))
from micromani_wrench_compensation import fit_gravity, expected_gravity, compensated_wrench


class WrenchCompensationTest(unittest.TestCase):
    def test_unexcited_orientations_rejected(self):
        with self.assertRaisesRegex(ValueError, 'orientation excitation'):
            fit_gravity([np.eye(3)] * 4, [[0, 0, 1., .03, .02, 0]] * 4)

    def test_weight_is_removed_but_contact_preserved(self):
        rotations = []
        for angle in [-.5, 0., .6]:
            c, s = np.cos(angle), np.sin(angle)
            rotations.append(np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]))
        original = {'mass_kg': .13, 'com_sensor_frame_m': [.01, .06, -.04]}
        wrenches = [expected_gravity(original, r) for r in rotations]
        fitted = fit_gravity(rotations, wrenches)
        contact = np.array([.02, -.01, 0, 0, .001, 0])
        np.testing.assert_allclose(compensated_wrench(fitted, rotations[1], wrenches[1] + contact), contact, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
