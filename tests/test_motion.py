import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'isaacsim_tools'))
from micromani_motion import JointMotion,drive_targets
from micromani_lerobot_schema import DEFAULT_HOME_ACTION
class MotionTests(unittest.TestCase):
 def test_bounded_speed_acceleration_and_completion(self):
  m=JointMotion(DEFAULT_HOME_ACTION);t=list(DEFAULT_HOME_ACTION);t[0]+=.003;t[3]+=.1;t[6]+=.001
  m.submit(t);dt=1/960;prev=list(m.reference);vel=[0]*14
  for _ in range(math.ceil(m.duration/dt)+2):
   cur=m.step(dt,prev);v=[(b-a)/dt for a,b in zip(prev,cur)];acc=[(b-a)/dt for a,b in zip(vel,v)]
   self.assertTrue(all(abs(x)<=lim*1.001 for x,lim in zip(v,m.velocity)))
   self.assertTrue(all(abs(x)<=lim*1.001 for x,lim in zip(acc,m.acceleration)))
   prev=cur;vel=v
  self.assertEqual(m.status,'reached');self.assertEqual(m.reference,t)
 def test_busy_stop_and_blocked_motion(self):
  m=JointMotion(DEFAULT_HOME_ACTION);t=list(DEFAULT_HOME_ACTION);t[6]=0.;m.submit(t)
  with self.assertRaises(RuntimeError):m.submit(t)
  m.step(m.duration+2.1,DEFAULT_HOME_ACTION)
  self.assertEqual(m.status,'contact_or_tracking_error')
  m.stop(DEFAULT_HOME_ACTION);self.assertEqual(m.status,'stopped')
  self.assertEqual(m.reference,list(DEFAULT_HOME_ACTION))
 def test_stop_retains_grip_preload_without_continuing_closure(self):
  m=JointMotion(DEFAULT_HOME_ACTION)
  target=list(DEFAULT_HOME_ACTION);target[6]=.001;target[13]=.002
  m.submit(target);m.step(m.duration*.5,DEFAULT_HOME_ACTION)
  reference=list(m.reference);measured=list(reference)
  measured[0]+=.00001;measured[6]+=.000004;measured[13]+=.000003
  m.stop(measured,preserve_grippers=True)
  self.assertEqual(m.status,'stopped')
  self.assertEqual(m.reference[0],measured[0])
  for i in (6,13):
   self.assertEqual(m.reference[i],reference[i])
   self.assertNotEqual(m.reference[i],target[i])
   self.assertNotEqual(m.reference[i],measured[i])
  self.assertEqual(m.target,m.reference)
 def test_driver_does_not_teleport_and_couples_fingers(self):
  from micromani_lerobot_schema import RAW_DOF_NAMES
  class Robot:
   def set_joint_position_targets(self,x,joint_indices):self.x=x;self.i=joint_indices
   def get_generalized_gravity_forces(self,joint_indices):
    import numpy as np
    assert len(joint_indices)==16
    return np.array([[.001*(i+1) for i in joint_indices]])
   def set_joint_efforts(self,x,joint_indices):self.efforts=x;self.effort_indices=joint_indices
   def set_joint_positions(self,*args,**kwargs):raise AssertionError('teleport')
   def set_joint_velocities(self,*args,**kwargs):raise AssertionError('velocity reset')
  r=Robot();drive_targets(r,{k:i for i,k in enumerate(RAW_DOF_NAMES)},DEFAULT_HOME_ACTION)
  self.assertEqual(r.efforts.shape,(1,16));self.assertEqual(list(r.effort_indices),list(range(16)));self.assertAlmostEqual(float(r.efforts[0,15]),.016,places=7);self.assertEqual(r.x.shape,(1,16));self.assertEqual(r.x[0,6],r.x[0,7]);self.assertEqual(r.x[0,14],r.x[0,15])

class MotionClientTests(unittest.TestCase):
 def test_wait_ignores_old_reached_snapshot(self):
  from micromani_client import MicroManiClient
  c=MicroManiClient();c._last_command_id='new'
  samples=iter([
    {'stale':False,'motion':{'status':'reached','command_id':'old'}},
    {'stale':False,'motion':{'status':'moving','command_id':'new'}},
    {'stale':False,'motion':{'status':'reached','command_id':'new'}}])
  c.state=lambda:next(samples);c.command=lambda _: {'status':'applied'}
  self.assertEqual(c.wait_motion(timeout=1)['motion']['command_id'],'new')
 def test_rejected_motion_does_not_wait_forever(self):
  from micromani_client import MicroManiClient
  c=MicroManiClient();c._last_command_id='failed';c.command=lambda _: {'status':'failed'}
  with self.assertRaises(RuntimeError):c.wait_motion(timeout=1)
