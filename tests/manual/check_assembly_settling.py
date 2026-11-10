from pathlib import Path
import os
ROOT=Path(__file__).resolve().parents[2]
QA=Path(os.environ.get('MICROMANI_QA_OUTPUT',str(ROOT/'outputs/physics_qa')))
(QA/'work').mkdir(parents=True,exist_ok=True)
(QA/'outputs/assembly').mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting'})
import sys,json,time
sys.path.insert(0,str(ROOT / 'isaacsim_tools'))
try:
 import omni.usd,numpy as np
 from pxr import UsdGeom,UsdPhysics,PhysxSchema
 from isaacsim.core.api import World
 from isaacsim.core.prims import Articulation,RigidPrim
 from micromani_lerobot_schema import ARTICULATION_ROOT,DEFAULT_HOME_ACTION
 from micromani_collect_isaac import get_dof_names,get_control_state,apply_control_action
 from micromani_motion import drive_targets,JointMotion
 omni.usd.get_context().open_stage(str(ROOT / 'outputs/assembly_physics/micromani_assembly.usda'))
 for _ in range(5):app.update()
 stage=omni.usd.get_context().get_stage()
 for p in stage.Traverse():
  if p.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI): print('SDF',p.GetPath(),PhysxSchema.PhysxSDFMeshCollisionAPI(p).GetSdfResolutionAttr().Get(),flush=True)
 world=World(stage_units_in_meters=1.,physics_dt=1/960,rendering_dt=1/30)
 print('RESET_BEGIN',flush=True);world.reset();print('RESET_DONE',flush=True)
 robot=Articulation(ARTICULATION_ROOT);robot.initialize();names=get_dof_names(robot);idx={n:i for i,n in enumerate(names)}
 apply_control_action(robot,idx,list(DEFAULT_HOME_ACTION))
 parts=RigidPrim('/World/TableParts/*');parts.initialize();initial=parts.get_world_poses()[0].copy()
 t=time.monotonic()
 for i in range(1920):
  drive_targets(robot,idx,DEFAULT_HOME_ACTION);world.step(render=False)
  if i%480==0: print('STEP',i,'parts',parts.get_world_poses()[0].tolist(),flush=True)
 final=parts.get_world_poses()[0];state=get_control_state(robot,names)
 result=dict(initial=initial.tolist(),final=final.tolist(),max_drift_m=float(abs(final-initial).max()),joint_error=(np.array(state)-DEFAULT_HOME_ACTION).tolist(),wall_s=time.monotonic()-t)
 print('RESULT',json.dumps(result),flush=True)
 open(str(QA / 'outputs/assembly_physics_probe.json'),'w').write(json.dumps(result,indent=2))
 assert result['max_drift_m']<50e-6,result
except Exception:
 import traceback;traceback.print_exc();raise
finally:app.close()
