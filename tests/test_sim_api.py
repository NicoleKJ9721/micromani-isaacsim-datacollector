import json,sys,time,unittest
from pathlib import Path
from urllib.error import HTTPError
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'isaacsim_tools'))
from micromani_api import SimulationAPI,validate_command
from micromani_client import MicroManiClient
from micromani_lerobot_schema import DEFAULT_HOME_ACTION,control_action_to_raw_targets

class APITests(unittest.TestCase):
    def setUp(self):
        self.api=SimulationAPI(port=0)
        self.client=MicroManiClient(f'http://127.0.0.1:{self.api.server.server_port}')
    def tearDown(self):self.api.close()
    def test_transport_validation_and_symmetric_gripper(self):
        self.assertEqual(len(self.client.schema()['control_order']),14)
        for body in ({'joints':{'joint_finger1_left':.001}}, {'joints':{'left_gripper':.011}},
                     {'axes':[0]*13}, {'joints':{'left_gripper':'nan'}}, {'joints':{}},
                     {'axes':[0]*14,'joints':{'left_gripper':0}}):
            with self.assertRaises(HTTPError) as cm:self.client._request('/v1/commands',body)
            self.assertEqual(cm.exception.code,400)
        reply=self.client.set_gripper('left',4.5)
        self.assertEqual(reply['status'],'accepted');seen=[]
        a,cid=self.api.apply_next(DEFAULT_HOME_ACTION,seen.append,3.)
        self.assertEqual(cid,reply['id']);self.assertEqual(self.client.wait(cid)['status'],'applied')
        raw=control_action_to_raw_targets(a)
        self.assertEqual(raw['joint_finger1_left'],.00225)
        self.assertEqual(raw['joint_finger2_left'],.00225)
        self.assertEqual(a[7:],list(DEFAULT_HOME_ACTION)[7:])
    def test_partial_commands_merge_at_apply_time(self):
        self.client.set_joints({'joint1_left':.003})
        self.client.set_joints({'joint2_left':.006})
        a,_=self.api.apply_next(DEFAULT_HOME_ACTION,lambda _:None,1.)
        b,_=self.api.apply_next(a,lambda _:None,2.)
        self.assertEqual(b[:2],[.003,.006])
    def test_expiry_and_failure_not_reported_as_applied(self):
        r=self.client.set_gripper('right',2.,ttl_s=.05);time.sleep(.06)
        a,c=self.api.apply_next(DEFAULT_HOME_ACTION,lambda _:self.fail('expired applied'),1.)
        self.assertIsNone(c);self.assertEqual(self.client.command(r['id'])['status'],'expired')
        r=self.client.set_gripper('left',3.)
        def fail(_):raise RuntimeError('invalid physics handle')
        a,c=self.api.apply_next(DEFAULT_HOME_ACTION,fail,1.)
        self.assertIsNone(c);self.assertEqual(self.client.command(r['id'])['status'],'failed')
    def test_stop_cancels_queue_and_requires_ack(self):
        r=self.client.set_gripper('left',3.)
        self.assertEqual(self.client.stop()['status'],'stop_requested')
        self.assertEqual(self.client.command(r['id'])['status'],'cancelled')
        with self.assertRaises(HTTPError):self.client.set_gripper('left',4.)
        self.assertTrue(self.api.consume_stop());self.assertFalse(self.api.consume_stop())
        self.client.set_gripper('left',4.)
    def test_queue_bounds_and_sensor_freshness(self):
        with self.assertRaises(HTTPError) as cm:self.client.state()
        self.assertEqual(cm.exception.code,503)
        for _ in range(32):self.client.set_gripper('right',2.)
        with self.assertRaises(HTTPError) as cm:self.client.set_gripper('right',2.)
        self.assertEqual(cm.exception.code,429)
        self.api.publish(sim_time=1.,state=DEFAULT_HOME_ACTION,action=DEFAULT_HOME_ACTION,force_torque={'test':1},grip={})
        state=self.client.state();self.assertFalse(state['stale']);self.assertEqual(state['gripper_opening_mm']['left'],4.5)
        with self.api.lock:self.api.snapshot['published_at']-=2
        self.assertTrue(self.client.state()['stale'])
        with self.assertRaises(HTTPError) as cm:self.client.cameras()
        self.assertEqual(cm.exception.code,503)

if __name__=='__main__':unittest.main()
