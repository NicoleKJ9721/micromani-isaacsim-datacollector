"""Offline physics QA, privileged fixture setup. Never exposed to the assembly agent."""
from pathlib import Path
import os
ROOT=Path(__file__).resolve().parents[2]
QA=Path(os.environ.get('MICROMANI_QA_OUTPUT',str(ROOT/'outputs/physics_qa')))
(QA/'work').mkdir(parents=True,exist_ok=True)
(QA/'outputs/assembly').mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
try:
 import omni.usd,numpy as np,json
 from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
 from isaacsim.core.api import World
 from isaacsim.core.prims import RigidPrim
 from pathlib import Path
 src=Usd.Stage.Open(str(ROOT / 'outputs/assembly_physics/micromani_assembly.usda'))
 # Flatten reference shapes + physical settings, then retain only a single pair.
 flat=src.Flatten(); stage=Usd.Stage.Open(flat)
 for p in list(stage.GetPseudoRoot().GetChildren()):
  if p.GetName()!='World':stage.RemovePrim(p.GetPath())
 for p in list(stage.GetPrimAtPath('/World').GetChildren()):
  if p.GetName() not in ('TableParts','PhysicsMaterials','Looks'):stage.RemovePrim(p.GetPath())
 for p in list(stage.GetPrimAtPath('/World/TableParts').GetChildren()):
  if p.GetName() not in ('gold_cavity_1','sleeve_1'):stage.RemovePrim(p.GetPath())
 # All material references retained by flatten, rendering is irrelevant for this fixture.
 sleeve=stage.GetPrimAtPath('/World/TableParts/sleeve_1');gold=stage.GetPrimAtPath('/World/TableParts/gold_cavity_1')
 UsdPhysics.RigidBodyAPI(sleeve).CreateKinematicEnabledAttr(True)
 for p,pos,orient in [(sleeve,(0,0,0),Gf.Quatf(1)),(gold,(0,0,.004),Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,1,0),90).GetQuat()))]:
  xf=UsdGeom.Xformable(p);xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(*pos));xf.AddOrientOp().Set(orient)
 scene=UsdPhysics.Scene.Define(stage,'/physicsScene');scene.CreateGravityMagnitudeAttr(0.)
 ph=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());ph.CreateEnableGPUDynamicsAttr(True);ph.CreateBroadphaseTypeAttr('GPU');ph.CreateSolverTypeAttr('TGS')
 stage.Export(str(QA / 'work/contact_fixture.usda'))
 omni.usd.get_context().open_stage(str(QA / 'work/contact_fixture.usda'))
 world=World(stage_units_in_meters=1.,physics_dt=1/960,rendering_dt=1/30);world.get_physics_context().set_gravity(0.);world.reset()
 body=RigidPrim('/World/TableParts/gold_cavity_1');body.initialize()
 results=[]
 for offset in (0.,40e-6):
  body.set_world_poses(positions=np.array([[offset,0,.004]],np.float32),orientations=np.array([[.70710678,0,.70710678,0]],np.float32))
  body.set_velocities(np.array([[0,0,-.0005,0,0,0]],np.float32))
  zs=[];xs=[]
  for i in range(3840):
   world.step(render=False)
   if i%96==0:
    pose=body.get_world_poses()[0][0];zs.append(float(pose[2]));xs.append(float(pose[0]))
  results.append(dict(offset_m=offset,z_min_m=min(zs),final=body.get_world_poses()[0][0].tolist(),velocity=body.get_velocities()[0].tolist(),samples_z=zs,samples_x=xs))
  print('CASE',offset,results[-1]['z_min_m'],results[-1]['final'],flush=True)
 Path(str(QA / 'outputs/assembly/contact_fixture.json')).write_text(json.dumps(results,indent=2))
 assert results[0]['final'][2]<.003 and results[1]['final'][2]>.0035,results
except Exception:
 import traceback;traceback.print_exc();raise
finally:app.close()
