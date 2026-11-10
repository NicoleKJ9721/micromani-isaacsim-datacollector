import json
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'isaacsim_tools'))
from micromani_breadboard import build_mesh

class BreadboardTests(unittest.TestCase):
    def test_surface_covers_rectangle_without_gaps(self):
        c=json.loads((ROOT/'config/platform_layout.json').read_text())['breadboard']
        v,f,holes=build_mesh(c)
        self.assertEqual(holes,1457)
        np.testing.assert_allclose(v.min(0),[-c['length_m']/2,-c['width_m']/2,0],atol=1e-10)
        np.testing.assert_allclose(v.max(0),[c['length_m']/2,c['width_m']/2,c['thickness_m']],atol=1e-10)
        t=v[f];cross=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0])
        # Top surface plus blind-hole floors must cover exactly the rectangle.
        self.assertAlmostEqual(np.maximum(cross[:,2],0).sum()/2,c['length_m']*c['width_m'],places=10)
    def test_invalid_depth_is_rejected(self):
        c=json.loads((ROOT/'config/platform_layout.json').read_text())['breadboard']
        c['hole_depth_m']=c['thickness_m']
        with self.assertRaises(ValueError):build_mesh(c)
