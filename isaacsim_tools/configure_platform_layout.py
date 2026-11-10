from pathlib import Path
import sys,json,shutil
import argparse
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'isaacsim_tools'))
parser=argparse.ArgumentParser(description='Apply absolute arm layout and generate parametric breadboard; render and verify physics.')
parser.add_argument('--stage',type=Path,required=True)
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--config',type=Path,default=root/'config/platform_layout.json')
args=parser.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting'})
try:
 import omni.usd,numpy as np
 from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Gf,Sdf
 from PIL import Image
 from isaacsim.core.api import World
 from isaacsim.core.prims import Articulation
 from isaacsim.sensors.camera import Camera
 from micromani_breadboard import build_mesh,write_stl
 from micromani_lerobot_schema import DEFAULT_HOME_ACTION,ARTICULATION_ROOT
 from micromani_collect_isaac import apply_control_action,get_dof_names
 from micromani_configure_cameras import camera_matrix
 source=args.stage.parent;out=args.out;out.mkdir(parents=True,exist_ok=False)
 for p in source.glob('*.png'):shutil.copy(p,out/p.name)
 omni.usd.get_context().open_stage(str(args.stage))
 for _ in range(10):app.update()
 stage=omni.usd.get_context().get_stage();stage.Flatten().Export(str(out/'micromani_equipped.usda'))
 omni.usd.get_context().open_stage(str(out/'micromani_equipped.usda'))
 for _ in range(10):app.update()
 stage=omni.usd.get_context().get_stage();cfg=json.loads(args.config.read_text());offset=cfg['arm_outward_offset_m'];report={'offset_m':offset,'joint_anchors':{},'initial_bodies':{}}
 parent=stage.GetPrimAtPath('/micromanipulator')
 for side,sign in [('left',-1),('right',1)]:
    joints=[p for p in stage.Traverse() if p.GetName()=='joint0_'+side and p.IsA(UsdPhysics.FixedJoint)]
    if len(joints)!=1:raise RuntimeError('ambiguous base joint')
    j=UsdPhysics.FixedJoint(joints[0]);targets=j.GetBody0Rel().GetTargets()
    if not targets or not str(targets[0]).endswith('/base_link'):raise RuntimeError(f'unexpected joint parent {targets}')
    old=j.GetLocalPos0Attr().Get();delta=sign*(.201+offset)-old[0];j.GetLocalPos0Attr().Set(Gf.Vec3f(sign*(.201+offset),old[1],old[2]));report['joint_anchors'][side]=list(j.GetLocalPos0Attr().Get())
    for prim in parent.GetChildren():
      if prim.GetName().endswith('_'+side) and prim.GetName().startswith('link'):
        xf=UsdGeom.Xformable(prim);ops=xf.GetOrderedXformOps();op=next(o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeTranslate)
        old=op.Get();op.Set(type(old)(old[0]+delta,old[1],old[2]));report['initial_bodies'][prim.GetName()]=list(op.Get())
 base=stage.GetPrimAtPath('/micromanipulator/base_link')
 for child in base.GetChildren():
    if child.GetName()=='visuals':UsdGeom.Imageable(child).MakeInvisible()
 # Replace the old base collider with the full board envelope; no second floor.
 ground_paths=[p.GetPath() for p in stage.Traverse() if p.GetName().lower()=='groundplane']
 for path in ground_paths:stage.RemovePrim(path)
 report['removed_ground_planes']=[str(p) for p in ground_paths]
 old_collisions=base.GetPath().AppendChild('collisions')
 if stage.GetPrimAtPath(old_collisions):stage.RemovePrim(old_collisions)
 bc=cfg['breadboard']
 collider=UsdGeom.Cube.Define(stage,str(base.GetPath())+'/BreadboardCollision')
 collider.CreateSizeAttr(1)
 xf=UsdGeom.Xformable(collider);xf.ClearXformOpOrder()
 xf.AddTranslateOp().Set(Gf.Vec3d(0,0,bc['thickness_m']/2))
 xf.AddScaleOp().Set(Gf.Vec3f(bc['length_m'],bc['width_m'],bc['thickness_m']))
 collider.CreatePurposeAttr('guide');UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
 report['collision_dimensions_m']=[bc['length_m'],bc['width_m'],bc['thickness_m']]
 vertices,faces,holes=build_mesh(cfg['breadboard'])
 mesh=UsdGeom.Mesh.Define(stage,'/micromanipulator/base_link/BreadboardVisual');mesh.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in vertices]);mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.flatten().tolist());mesh.CreateSubdivisionSchemeAttr('none')
 mesh.CreateDoubleSidedAttr(False)
 material=UsdShade.Material.Define(stage,'/World/Looks/BreadboardSatinSteel');shader=UsdShade.Shader.Define(stage,str(material.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
 shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.52,.55,.58));shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(.8);shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.38);shader.CreateOutput('surface',Sdf.ValueTypeNames.Token);material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
 report['breadboard']={**cfg['breadboard'],'holes':holes,'bounds_m':[vertices.min(0).tolist(),vertices.max(0).tolist()]}
 # Inspection camera; training camera intrinsics/extrinsics stay intact.
 camera=UsdGeom.Camera.Define(stage,'/World/Breadboard_Inspection');camera.CreateFocalLengthAttr(30);camera.CreateHorizontalApertureAttr(20.955);camera.CreateClippingRangeAttr(Gf.Vec2f(.001,5));UsdGeom.Xformable(camera).MakeMatrixXform().Set(camera_matrix((.24,-.34,.36),(0,0,.005)))
 overview=UsdGeom.Camera.Define(stage,'/World/Platform_Overview')
 overview.CreateFocalLengthAttr(30);overview.CreateHorizontalApertureAttr(20.955);overview.CreateClippingRangeAttr(Gf.Vec2f(.01,20))
 UsdGeom.Xformable(overview).MakeMatrixXform().Set(camera_matrix((1.25,-1.65,1.5),(0,0,.05)))
 stage.GetRootLayer().Save();write_stl(out/'base_breadboard.STL',vertices,faces)
 world=World(stage_units_in_meters=1,physics_dt=1/30,rendering_dt=1/30);world.reset();robot=Articulation(ARTICULATION_ROOT);robot.initialize();idx={n:i for i,n in enumerate(get_dof_names(robot))}
 cameras={n:Camera(p,resolution=(1200,900)) for n,p in [('global','/World/Platform_Overview'),('breadboard','/World/Breadboard_Inspection')]}
 for c in cameras.values():c.initialize()
 for _ in range(60):apply_control_action(robot,idx,list(DEFAULT_HOME_ACTION));world.step(render=True)
 poses=np.asarray(robot._physics_view.get_link_transforms())[0];report['runtime_base_positions']={side:poses[robot.get_body_index('link0_'+side),:3].tolist() for side in ('left','right')}
 for side,position in report['runtime_base_positions'].items():
    expected=report['initial_bodies']['link0_'+side]
    if not np.allclose(position,expected,atol=1e-5):raise RuntimeError(f'base moved back: {side}')
 for n,c in cameras.items():Image.fromarray(np.asarray(c.get_rgba())[:,:,:3].astype('uint8')).save(out/(n+'.png'))
 report['passed']=True;(out/'layout_report.json').write_text(json.dumps(report,indent=2));print('LAYOUT_DONE',report,flush=True)
except BaseException:
 import traceback;traceback.print_exc();app._app.post_quit(1)
finally:app.close()
