import json,sys,time,unittest,urllib.request,urllib.error
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'isaacsim_tools'))
from micromani_camera_preview import CameraPreview

class FakeCamera:
    stamp=1.
    def get_current_frame(self):return {'rendering_time':self.stamp,'rgb':np.zeros((480,640,3),dtype=np.uint8)}

class PreviewTests(unittest.TestCase):
    def test_latest_frame_only_and_no_fabricated_updates(self):
        preview=CameraPreview(0,max_fps=100000)
        url=f'http://127.0.0.1:{preview.server.server_port}/frame'
        try:
            with self.assertRaises(urllib.error.HTTPError) as e:urllib.request.urlopen(url)
            self.assertEqual(e.exception.code,503)
            cams={n:FakeCamera() for n in ('left_wrist','right_wrist','global')}
            preview.publish(cams,1.)
            a=json.load(urllib.request.urlopen(url));self.assertEqual(a['sequence'],1)
            self.assertEqual(set(a['cameras']),set(cams))
            preview.publish(cams,1.)
            self.assertEqual(json.load(urllib.request.urlopen(url))['sequence'],1)
            for c in cams.values():c.stamp=2.
            preview.publish(cams,2.)
            b=json.load(urllib.request.urlopen(url));self.assertEqual(b['sequence'],2)
            self.assertTrue(all(c['render_time']==2. for c in b['cameras'].values()))
        finally:preview.close()
