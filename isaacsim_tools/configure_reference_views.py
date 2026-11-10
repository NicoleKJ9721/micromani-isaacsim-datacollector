from pathlib import Path
import sys,json,shutil,math
import argparse
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'isaacsim_tools'))
parser=argparse.ArgumentParser(description='Apply reference-image camera directions and calibrated geometric home gaps.')
parser.add_argument('--stage',type=Path,required=True)
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--config',type=Path,default=root/'config/camera_reference_layout.json')
args=parser.parse_args();cfg=json.loads(args.config.read_text())
out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting'})
try:
 import numpy as np,omni.usd
 from pxr import Usd,UsdGeom,UsdPhysics,Gf
 from PIL import Image
 from isaacsim.core.api import World
 from isaacsim.core.prims import Articulation
 from isaacsim.sensors.camera import Camera
 from micromani_lerobot_schema import CAMERA_SPECS,FORCE_TORQUE_SENSORS,DEFAULT_HOME_ACTION,ARTICULATION_ROOT
 from micromani_collect_isaac import get_dof_names,apply_control_action,read_force_torque
 from micromani_configure_cameras import wrist_work_target,camera_matrix,focal_length_for_view
 from micromani_lerobot_schema import control_action_to_raw_targets
 from micromani_record_session import synchronize_cameras,camera_sample
 stagepath=args.stage.resolve()
 omni.usd.get_context().open_stage(str(stagepath))
 for _ in range(10):app.update()
 stage=omni.usd.get_context().get_stage()
 dest=out
 for p in stagepath.parent.glob('*.png'):shutil.copy2(p,dest/p.name)
 stage.Flatten().Export(str(dest/'micromani_equipped.usda'))
 omni.usd.get_context().open_stage(str(dest/'micromani_equipped.usda'))
 for _ in range(10):app.update()
 stage=omni.usd.get_context().get_stage()
 world=World(stage_units_in_meters=1,physics_dt=1/240,rendering_dt=1/30);world.reset()
 robot=Articulation(ARTICULATION_ROOT);robot.initialize();idx={n:i for i,n in enumerate(get_dof_names(robot))};apply_control_action(robot,idx,list(DEFAULT_HOME_ACTION))
 cams={n:Camera(s['prim_path'],resolution=(640,480),frequency=30) for n,s in CAMERA_SPECS.items()}
 for c in cams.values():c.initialize();c.add_rgb_to_frame()
 for _ in range(240):world.step(render=False)
 for _ in range(8):world.step(render=True)
 cache=UsdGeom.XformCache()
 def bodymatrix(name):
  pose=np.asarray(robot._physics_view.get_link_transforms())[0,robot.get_body_index(name)];t=Gf.Transform();t.SetTranslation(Gf.Vec3d(*map(float,pose[:3])));q=pose[3:];t.SetRotation(Gf.Rotation(Gf.Quatd(float(q[3]),Gf.Vec3d(*map(float,q[:3])))));return t.GetMatrix()
 def matrix(path):
  prim=stage.GetPrimAtPath(path);p=prim
  while str(p.GetPath()) not in ('/','/micromanipulator'):
   if p.GetName() in robot.body_names:
    return cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(p).GetInverse()*bodymatrix(p.GetName())
   p=p.GetParent()
  return cache.GetLocalToWorldTransform(prim)
 def points(path):
  result=[]
  for p in Usd.PrimRange(stage.GetPrimAtPath(path)):
   if p.IsA(UsdGeom.Mesh):result.extend(matrix(str(p.GetPath())).Transform(Gf.Vec3d(*map(float,v))) for v in UsdGeom.Mesh(p).GetPointsAttr().Get())
  return np.asarray(result)
 report={'cameras':{},'home':list(DEFAULT_HOME_ACTION),'gaps_mm':{}}
 targets={}
 for side in ('left','right'):
  inward=1 if side=='left' else -1;fingerpoints=[points(f'/micromanipulator/link_finger{i}_{side}') for i in (1,2)]
  target=np.array(wrist_work_target(*fingerpoints,inward));targets[side]=target
  tips=[p[inward*p[:,0]>max(inward*p[:,0])-.001] for p in fingerpoints]
  ranges=sorted([(p[:,1].min(),p[:,1].max()) for p in tips]);report['gaps_mm'][side]=(ranges[1][0]-ranges[0][1])*1000
  reference=cfg['cameras'][side+'_wrist'];eye=target+np.array(reference['eye'])-np.array(reference['target'])
  target=target+np.array([inward*reference.get('aim_forward_m',0),0,0])
  path=CAMERA_SPECS[side+'_wrist']['prim_path'];local=camera_matrix(eye,target)*bodymatrix('link6_'+side).GetInverse()
  q=local.ExtractRotationQuat();cams[side+'_wrist'].set_local_pose(translation=np.array(local.ExtractTranslation()),orientation=np.array([q.GetReal(),*q.GetImaginary()]),camera_axes='usd')
  cam=UsdGeom.Camera.Get(stage,path);cam.GetFocalLengthAttr().Set(focal_length_for_view(.04,math.dist(eye,target),5.184))
  cam.GetPrim().SetCustomDataByKey('micromani_target_fov_width_m',.04);cam.GetPrim().SetCustomDataByKey('micromani_target_fov_height_m',.03)
  report['cameras'][side+'_wrist']={'eye':eye.tolist(),'target':target.tolist(),'view_width_m':.04}
 target=(targets['left']+targets['right'])/2;eye=target+np.array(cfg['cameras']['global']['eye'])-np.array(cfg['cameras']['global']['target']);m=camera_matrix(eye,target)
 q=m.ExtractRotationQuat();cams['global'].set_world_pose(position=eye,orientation=np.array([q.GetReal(),*q.GetImaginary()]),camera_axes='usd')
 gc=UsdGeom.Camera.Get(stage,CAMERA_SPECS['global']['prim_path']);gc.GetFocalLengthAttr().Set(focal_length_for_view(cfg['cameras']['global']['view_width_m'],math.dist(eye,target),5.184));gc.GetPrim().SetCustomDataByKey('micromani_target_fov_width_m',cfg['cameras']['global']['view_width_m']);gc.GetPrim().SetCustomDataByKey('micromani_target_fov_height_m',cfg['cameras']['global']['view_width_m']*.75)
 report['cameras']['global']={'eye':eye.tolist(),'target':target.tolist(),'view_width_m':cfg['cameras']['global']['view_width_m']}
 for _ in range(12):world.step(render=True)
 synchronize_cameras(world,cams)
 for name,c in cams.items():
  rgb,stamp=camera_sample(c);Image.fromarray(rgb).save(out/(name+'.png'));report['cameras'][name]['render_time']=stamp
 poses=np.asarray(robot._physics_view.get_link_transforms())[0].copy();names=list(robot.body_names);world.stop();cache.Clear()
 for name,pose in zip(names,poses):
  prim=stage.GetPrimAtPath('/micromanipulator/'+name)
  if not prim:continue
  t=Gf.Transform();t.SetTranslation(Gf.Vec3d(*map(float,pose[:3])));q=pose[3:];t.SetRotation(Gf.Rotation(Gf.Quatd(float(q[3]),Gf.Vec3d(*map(float,q[:3])))))
  local=t.GetMatrix()*cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse();xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder()
  for attr in list(prim.GetAttributes()):
   if attr.GetName().startswith('xformOp:'):prim.RemoveProperty(attr.GetName())
  xf.AddTranslateOp().Set(local.ExtractTranslation());xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(local.ExtractRotationQuat()))
 for name,value in control_action_to_raw_targets(DEFAULT_HOME_ACTION).items():
  for prim in stage.Traverse():
   if prim.GetName()==name:
    linear=prim.IsA(UsdPhysics.PrismaticJoint);UsdPhysics.DriveAPI.Get(prim,'linear' if linear else 'angular').GetTargetPositionAttr().Set(value if linear else math.degrees(value))
 stage.GetRootLayer().Save();(out/'report.json').write_text(json.dumps(report,indent=2));print('CAMERAS_DONE',report,flush=True)
except Exception:
 import traceback;traceback.print_exc()
finally:app.close()
