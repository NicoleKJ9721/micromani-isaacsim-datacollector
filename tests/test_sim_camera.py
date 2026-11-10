import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaacsim_tools'))
from micromani_record_session import camera_sample

class CameraTests(unittest.TestCase):
    def test_isaac_51_rgb_frame_with_dictionary_render_id(self):
        rgba=np.zeros((480,640,4),dtype=np.uint8)
        rgba[:,:,0]=123
        frame={'rgb':rgba,'rendering_time':2.,'rendering_frame':
               {'referenceTimeNumerator':120,'referenceTimeDenominator':60}}
        rgb,stamp=camera_sample(SimpleNamespace(get_current_frame=lambda:frame))
        self.assertEqual(rgb.shape,(480,640,3))
        self.assertEqual(int(rgb[0,0,0]),123)
        self.assertEqual(stamp,2.)

    def test_missing_or_invalid_camera_data_rejected(self):
        for frame in ({'rendering_time':0.}, {'rendering_time':float('nan')},
                      {'rendering_time':0.,'rgb':None}):
            with self.assertRaises((ValueError,KeyError)):
                camera_sample(SimpleNamespace(get_current_frame=lambda:frame))

class SynchronizationTests(unittest.TestCase):
    def test_render_latency_drained_without_physics_step(self):
        from micromani_record_session import synchronize_cameras
        image=np.zeros((480,640,3),dtype=np.uint8)
        stamps=iter([1.8,1.9,2.0])
        frame={'rgb':image,'rendering_time':next(stamps)}
        calls=[]
        def render():
            calls.append(True)
            frame['rendering_time']=next(stamps)
        world=SimpleNamespace(current_time=2.0,render=render)
        synchronize_cameras(world,{'camera':SimpleNamespace(get_current_frame=lambda:frame)})
        self.assertEqual(len(calls),2)
        self.assertEqual(world.current_time,2.0)
        self.assertEqual(frame['rendering_time'],2.0)

    def test_stale_or_future_frame_fails_without_relabeling(self):
        from micromani_record_session import synchronize_cameras
        for stamp in (1.0,3.0):
            frame={'rgb':np.zeros((480,640,3),dtype=np.uint8),'rendering_time':stamp}
            world=SimpleNamespace(current_time=2.0,render=lambda:None)
            with self.assertRaisesRegex(ValueError,'synchronization timeout'):
                synchronize_cameras(world,{'camera':SimpleNamespace(get_current_frame=lambda:frame)},max_renders=2)
            self.assertEqual(frame['rendering_time'],stamp)
