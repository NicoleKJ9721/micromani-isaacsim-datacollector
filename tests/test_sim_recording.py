"""离线回归：使用明确标注的合成观测，不连接硬件。"""
import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaacsim_tools'))
from micromani_sim_contract import SimContract, validate_action
from micromani_episode import EpisodeSession


def config():
    return {'operator_left_model_side': 'right',
            'gripper_zero_gap_mm': {'left': 2., 'right': 3.},
            'gripper_open_sign': {'left': 1, 'right': 1},
            'motion_sign': {'left': [1]*6, 'right': [1]*6},
            'motion_zero_si': {'left': [0]*6, 'right': [0]*6}}


class ContractTests(unittest.TestCase):
    def test_units_side_gap_round_trip(self):
        c = SimContract(config())
        raw = [.01, .02, .03, .1, .02, -.2, .004,
               .04, .03, .02, -.3, .04, .5, .006]
        out = c.to_dataset(raw)
        self.assertAlmostEqual(out[0], 40000)
        self.assertAlmostEqual(out[3], math.degrees(-.3)*1000)
        self.assertAlmostEqual(out[6], 15)
        self.assertAlmostEqual(out[13], 10)
        for a, b in zip(raw, c.to_model(out)):
            self.assertAlmostEqual(a, b)
        self.assertEqual(c.force_to_dataset(list(range(12))), (list(range(6,12)), list(range(6))))

    def test_missing_calibration_rejected(self):
        cfg = config(); cfg['gripper_zero_gap_mm']['left'] = None
        with self.assertRaises(ValueError): SimContract(cfg)
        cfg = config(); cfg['operator_left_model_side'] = None
        with self.assertRaises(ValueError): SimContract(cfg)

    def test_invalid_and_out_of_range_commands_rejected(self):
        for value in (float('nan'), float('inf'), -.01, .08):
            x = [0.]*14; x[0] = value
            with self.assertRaises(ValueError): validate_action(x)
        with self.assertRaises(ValueError): validate_action([0]*12)

    def test_metadata_mismatch_rejected(self):
        c = SimContract(config()); m = c.metadata()
        self.assertNotIn('observation.pulses', c.features())
        self.assertNotEqual(m['version'], 'appstation.dual_arm.operator_sides.v2')
        m['units']['rotation'] = 'rad'
        with self.assertRaises(ValueError): c.validate_metadata(m)


class EpisodeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)/'session'
        self.session = EpisodeSession(self.root, SimContract(config()), fps=30, task='test', stage='fixture')

    def tearDown(self): self.tmp.cleanup()

    def frame(self, t=1.):
        return dict(sim_time=t, state=[0.]*14, action=[0.]*14, force=[0.]*12,
                    images={k:b'fixture' for k in ('left_wrist','right_wrist','global')},
                    source_times={k:t for k in ('state','force','left_wrist','right_wrist','global')},
                    action_time=t, action_source='synthetic_test')

    def test_save_discard_resume_and_frozen_metadata(self):
        self.session.start()
        self.session.append(**self.frame())
        self.session.save('success')
        self.session.start(); self.session.append(**self.frame(2)); self.session.discard()
        self.session.finish()
        resumed = EpisodeSession(self.root, SimContract(config()), fps=30, task='test', stage='fixture', resume=True)
        resumed.start(); resumed.append(**self.frame(3)); resumed.save('failure')
        data=json.loads((self.root/'session.json').read_text())
        self.assertEqual([e['index'] for e in data['episodes']], [0,1,2])
        self.assertEqual([e['status'] for e in data['episodes']], ['saved','discarded','saved'])
        self.assertEqual(data['episodes'][0]['outcome'], 'success')
        cfg=config(); cfg['gripper_zero_gap_mm']['left']=4
        with self.assertRaises(ValueError):
            EpisodeSession(self.root, SimContract(cfg), fps=30, task='test', stage='fixture', resume=True)

    def test_stale_future_gap_and_empty_save_rejected(self):
        self.session.start()
        with self.assertRaises(ValueError): self.session.save('success')
        for key in ('source_times','action_time'):
            f=self.frame()
            if key=='source_times': f[key]['force']-=.1
            else: f[key]+=.1
            with self.assertRaises(ValueError): self.session.append(**f)
        self.session.append(**self.frame())
        with self.assertRaises(ValueError): self.session.append(**self.frame(1.2))
        self.assertEqual(self.session.frame_count,1)

    def test_resume_draft_requires_discard(self):
        self.session.start(); self.session.append(**self.frame())
        resumed=EpisodeSession(self.root, SimContract(config()), fps=30, task='test', stage='fixture', resume=True)
        with self.assertRaises(RuntimeError): resumed.start()
        resumed.discard(); resumed.start()


if __name__ == '__main__': unittest.main()
