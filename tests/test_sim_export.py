"""真实 LeRobot 文件往返测试；数据为合成夹具，不代表物理验收。"""
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'isaacsim_tools'))
from micromani_episode import EpisodeSession
from micromani_sim_contract import SimContract,CAMERA_KEYS
from micromani_export_session import export_session
from test_sim_recording import config

class ExportTests(unittest.TestCase):
    def test_real_lerobot_roundtrip_and_preserved_output_on_bad_input(self):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'raw'; out=Path(tmp)/'export'
            c=SimContract(config()); s=EpisodeSession(root,c,fps=30,task='synthetic fixture',stage='test')
            buf=io.BytesIO(); Image.new('RGB',(640,480),(10,20,30)).save(buf,format='PNG')
            for index in range(3):
                s.start()
                for i in range(2):
                    t=index+i/30
                    q=[.01,.02,.03,.1,.02,.3,.004]*2
                    s.append(sim_time=t,state=q,action=q,force=list(range(12)),images={k:buf.getvalue() for k in CAMERA_KEYS},
                      source_times={k:t for k in ('state','force',*CAMERA_KEYS)},action_time=t,action_source='synthetic_fixture')
                if index==1: s.discard()
                else: s.save('success' if index==0 else 'failure')
            s.finish()
            before={str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()}
            report=export_session(root,out,'micromani/test')
            self.assertEqual(report['frames'],4); self.assertEqual(report['episodes'],2)
            d=LeRobotDataset('micromani/test',root=out,return_uint8=True)
            np.testing.assert_allclose(np.asarray(d[0]['observation.force_left']),list(range(6,12)))
            self.assertNotIn('observation.pulses',d.features)
            np.testing.assert_allclose(np.asarray(d[0]['observation.state']),c.to_dataset(q),rtol=1e-6)
            info=json.loads((out/'meta/micromani_info.json').read_text())
            self.assertEqual([e['index'] for e in info['episodes']],[0,2])
            self.assertEqual(before,{str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()})
            saved=(out/'meta/info.json').read_bytes()
            (root/'episode_000000/images/global/000000.png').write_bytes(b'broken')
            with self.assertRaises(Exception): export_session(root,out,'micromani/test',overwrite=True)
            self.assertEqual(saved,(out/'meta/info.json').read_bytes())

if __name__=='__main__': unittest.main()

class ReplayTests(unittest.TestCase):
    def test_contract_required_and_unknown_never_replayed(self):
        from micromani_replay_lerobot import load_replay_contract
        with tempfile.TemporaryDirectory() as tmp:
            meta=Path(tmp)/'meta'; meta.mkdir(); info=meta/'info.json'
            info.write_text('{}')
            with self.assertRaises(ValueError): load_replay_contract(tmp)
            self.assertIsNone(load_replay_contract(tmp,True))
            info.write_text(json.dumps({'dataContract':{'version':'unknown'}}))
            with self.assertRaises(ValueError): load_replay_contract(tmp,True)
            c=SimContract(config()); info.write_text(json.dumps({'dataContract':c.metadata()}))
            self.assertEqual(load_replay_contract(tmp).metadata(),c.metadata())

    def test_float32_boundary_roundtrip(self):
        from micromani_lerobot_schema import JOINT_LIMITS,CONTROL_ORDER
        c=SimContract(config())
        q=[JOINT_LIMITS[n][1] for n in CONTROL_ORDER]
        decoded=c.to_model(np.asarray(c.to_dataset(q),dtype=np.float32))
        np.testing.assert_allclose(decoded,q,atol=1e-7)
