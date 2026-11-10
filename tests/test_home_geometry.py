import sys, unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'isaacsim_tools'))
from micromani_home_geometry import bounds
from micromani_lerobot_schema import DEFAULT_HOME_ACTION,CONTROL_ORDER,JOINT_LIMITS

class HomeTests(unittest.TestCase):
    def test_default_home_has_cross_arm_clearance(self):
        for name,value in zip(CONTROL_ORDER,DEFAULT_HOME_ACTION):
            lo,hi=JOINT_LIMITS[name]
            self.assertTrue(lo<=value<=hi)
        b=bounds(DEFAULT_HOME_ACTION)
        for a in b:
            if not a.endswith('_left'):continue
            for c in b:
                if not c.endswith('_right'):continue
                gap=np.maximum(b[c][0]-b[a][1],b[a][0]-b[c][1])
                distance=float(np.linalg.norm(np.maximum(gap,0)))
                self.assertGreaterEqual(distance,.005,(a,c,distance))
                if 'finger' in a and 'finger' in c:
                    self.assertGreaterEqual(distance,.010,(a,c,distance))
