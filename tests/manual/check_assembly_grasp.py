"""Offline QA with actual robot fingers; fixture placement is not an agent observation."""
from pathlib import Path
import os
ROOT=Path(__file__).resolve().parents[2]
QA=Path(os.environ.get('MICROMANI_QA_OUTPUT',str(ROOT/'outputs/physics_qa')))
(QA/'work').mkdir(parents=True,exist_ok=True)
(QA/'outputs/assembly').mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
try:
 import sys,json,os,numpy as np,omni.usd
 from pathlib import Path
 from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
 from isaacsim.core.api import World
 from isaacsim.core.prims import Articulation,RigidPrim
 from isaacsim.core.api.objects import FixedCuboid
 sys.path.insert(0,str(ROOT / 'isaacsim_tools'))
 from micromani_lerobot_schema import DEFAULT_HOME_ACTION,ARTICULATION_ROOT
 from micromani_collect_isaac import get_dof_names,get_control_state,apply_control_action
 from micromani_motion import JointMotion,drive_targets
 from micromani_grip_sensing import create_finger_sensors,read_grip_sensors
 omni.usd.get_context().open_stage(str(ROOT / 'outputs/assembly_physics/micromani_assembly.usda'))
 stage=omni.usd.get_context().get_stage()
 if 'GRIP_FORCE_LIMIT' in os.environ:
  for p in stage.Traverse():
   if 'joint_finger' in p.GetName() and p.IsA(UsdPhysics.PrismaticJoint):UsdPhysics.DriveAPI.Get(p,'linear').GetMaxForceAttr().Set(float(os.environ['GRIP_FORCE_LIMIT']))
 asset=os.environ.get('FIXTURE_PART','gold_cavity');side='left' if asset=='gold_cavity' else 'right'
 home=list(DEFAULT_HOME_ACTION)
 if side=='right':home[13]=.0027
 else:home[6]=.0015
 scale=float(os.environ.get('TPU_FRICTION_SCALE','1'))
 if 'TPU_STIFFNESS' in os.environ:PhysxSchema.PhysxMaterialAPI(stage.GetPrimAtPath('/World/PhysicsMaterials/TPU_fingers')).GetCompliantContactStiffnessAttr().Set(float(os.environ['TPU_STIFFNESS']))
 mat=UsdPhysics.MaterialAPI(stage.GetPrimAtPath('/World/PhysicsMaterials/TPU_fingers'))
 mat.GetStaticFrictionAttr().Set(mat.GetStaticFrictionAttr().Get()*scale);mat.GetDynamicFrictionAttr().Set(mat.GetDynamicFrictionAttr().Get()*scale)
 for p in list(stage.GetPrimAtPath('/World/TableParts').GetChildren()):
  if p.GetName()!=asset+'_1':p.SetActive(False)
 gold=stage.GetPrimAtPath('/World/TableParts/'+asset+'_1')
 if side=='left':x=-.02515021-.00135;y=-.11765635;z=.315328-.00000845852;bottom=.00249154144;rotation=Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,1,0),90).GetQuat())
 else:x=.02515021+.00135;y=-.07765635;z=.314078-.00055;bottom=.0015;rotation=Gf.Quatf(1)
 xf=UsdGeom.Xformable(gold)
 for op in xf.GetOrderedXformOps():
  if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(0,0,1))
  if op.GetOpType()==UsdGeom.XformOp.TypeOrient:op.Set(rotation)
 pad=FixedCuboid('/World/TestPedestal',position=np.array([x,y,z-bottom-.0005]),scale=np.array([.003,.0008,.001]),size=1.)
 cp=PhysxSchema.PhysxCollisionAPI.Apply(pad.prim);cp.CreateContactOffsetAttr(2e-6);cp.CreateRestOffsetAttr(0.)
 world=World(stage_units_in_meters=1.,physics_dt=1/960,rendering_dt=1/30)
 sensors=create_finger_sensors(dt=1/30)
 for check_side in ('left','right'):
  for i in (1,2):
   assert not stage.GetPrimAtPath(f'/micromanipulator/link_finger{i}_{check_side}').HasAPI(UsdPhysics.CollisionAPI),'sensor recreated a parent hull'
 world.reset()
 for sensor in sensors.values():sensor.initialize()
 robot=Articulation(ARTICULATION_ROOT);robot.initialize();names=get_dof_names(robot);idx={n:i for i,n in enumerate(names)}
 apply_control_action(robot,idx,home);motion=JointMotion(home)
 body=RigidPrim('/World/TableParts/'+asset+'_1');body.initialize()
 # Keep the test object away during articulation initialization. Reset's first
 # physics step uses the imported joint states, before runtime home is applied.
 for _ in range(96):drive_targets(robot,idx,home);world.step(render=False)
 body.set_world_poses(positions=np.array([[x,y,z]],np.float32),orientations=np.array([[rotation.GetReal(),*rotation.GetImaginary()]],np.float32))
 body.set_velocities(np.zeros((1,6),np.float32))
 report={'asset':asset,'friction_scale':scale}
 def run(seconds):
  peak=np.zeros(2)
  for i in range(round(seconds*960)):
   q=motion.step(1/960,get_control_state(robot,names));drive_targets(robot,idx,q);world.step(render=False)
   if i%32==0:peak=np.maximum(peak,read_grip_sensors(sensors)[side]['finger_contact_n'])
  return dict(position=body.get_world_poses()[0][0].tolist(),grip=read_grip_sensors(sensors)[side],peak=peak.tolist(),motion=motion.status)
 report['settled']=run(.5)
 print('JOINTS',get_control_state(robot,names),flush=True)
 cache=UsdGeom.XformCache()
 for finger in (1,2):
  link=stage.GetPrimAtPath(f'/micromanipulator/link_finger{finger}_{side}')
  pose=np.asarray(robot._physics_view.get_link_transforms())[0,robot.get_body_index(link.GetName())]
  t=Gf.Transform();t.SetTranslation(Gf.Vec3d(*map(float,pose[:3])));q=pose[3:];t.SetRotation(Gf.Rotation(Gf.Quatd(float(q[3]),Gf.Vec3d(*map(float,q[:3])))))
  pts=[]
  for p in Usd.PrimRange(link):
   if p.IsA(UsdGeom.Mesh) and '/visuals/' in str(p.GetPath()):
    mat=cache.GetLocalToWorldTransform(p)*cache.GetLocalToWorldTransform(link).GetInverse()*t.GetMatrix()
    pts.extend(mat.Transform(Gf.Vec3d(*map(float,v))) for v in UsdGeom.Mesh(p).GetPointsAttr().Get())
  pts=np.asarray(pts);tip=pts[:,0].max() if side=='left' else pts[:,0].min();near=pts[abs(pts[:,0]-tip)<.00125]
  print('TIP',finger,near.min(0),near.max(0),flush=True)
 print('SETTLED',report['settled'],flush=True)
 target=list(home);target[6 if side=='left' else 13]=.00125 if side=='left' else .0024;motion.submit(target)
 report['closed']=run(motion.duration+2.2);print('CLOSED',report['closed'],flush=True)
 target[2 if side=='left' else 9]-=.002;motion.submit(target);report['lifted']=run(motion.duration+2.2);print('LIFTED',report['lifted'],flush=True)
 report['lift_m']=report['lifted']['position'][2]-report['closed']['position'][2]
 report['slip_error_m']=(np.array(report['lifted']['position'])-np.array(report['closed']['position'])-[0,0,.002]).tolist()
 report['precision_50um_passed']=bool(max(abs(np.array(report['slip_error_m'])))<50e-6)
 report['acceptance']='coarse retention: displacement error below 0.2mm, plus bilateral final contact; not precision alignment'
 report['passed']=bool(max(abs(np.array(report['slip_error_m'])))<.0002 and report['lifted']['grip']['bilateral_contact'])
 Path(str(QA / ('outputs/assembly/grasp_'+asset+'_'+str(scale)+'.json'))).write_text(json.dumps(report,indent=2))
 print('GRASP_RESULT',report['passed'],flush=True)
 assert report['passed'],report
except Exception:
 import traceback;traceback.print_exc();raise
finally:app.close()
