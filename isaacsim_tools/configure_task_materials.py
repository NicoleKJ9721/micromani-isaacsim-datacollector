"""Prepare task assets and mount the static resin table on the breadboard."""
from pathlib import Path
import json, shutil
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Sdf,Gf
ROOT=Path(__file__).resolve().parents[1]
SRC=Path('/home/nkj/isaacsim_assets/step_parts_20261109.lQQzql')
OUT=ROOT/'outputs/material_table_20261109'
OUT.mkdir(parents=True,exist_ok=True)
SPECS={'operation_table':((.018,.020,.022),0.,.43),'gold_cavity':((.72,.46,.16),1.,.25),'sleeve':((.80,.82,.85),1.,.23)}
for name,(color,metal,rough) in SPECS.items():
 stage=Usd.Stage.Open(str(SRC/(name+'.usd')))
 stage.GetRootLayer().Export(str(OUT/(name+'.usd')))
 stage=Usd.Stage.Open(str(OUT/(name+'.usd')))
 root=stage.GetDefaultPrim();mat=UsdShade.Material.Define(stage,str(root.GetPath())+'/Looks/Material')
 shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
 for k,t,v in [('diffuseColor',Sdf.ValueTypeNames.Color3f,color),('metallic',Sdf.ValueTypeNames.Float,metal),('roughness',Sdf.ValueTypeNames.Float,rough),('ior',Sdf.ValueTypeNames.Float,1.5)]:shader.CreateInput(k,t).Set(v)
 mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
 for p in stage.Traverse():
  if p.IsA(UsdGeom.Mesh):
   UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
   UsdGeom.Mesh(p).CreateDisplayColorAttr([Gf.Vec3f(*color)])
   if name=='operation_table':
    UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('none')
 stage.GetRootLayer().Save()
base=ROOT/'outputs/reference_cameras_fine_20261109/micromani_equipped.usda'
for p in base.parent.glob('*.png'):shutil.copy2(p,OUT/p.name)
stage=Usd.Stage.Open(str(base));stage.Flatten().Export(str(OUT/'micromani_equipped.usda'))
stage=Usd.Stage.Open(str(OUT/'micromani_equipped.usda'))
# Static triangle collision geometry mounted at the fixed base world pose.
baseprim=stage.GetPrimAtPath('/micromanipulator/base_link');assert baseprim
bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeLocalBound(baseprim).ComputeAlignedRange()
center=(bbox.GetMin()+bbox.GetMax())*.5
p=UsdGeom.Xform.Define(stage,'/World/OperationTable')
p.GetPrim().GetReferences().AddReference('./operation_table.usd')
mount=Gf.Matrix4d(1);mount.SetTranslate(Gf.Vec3d(center[0],center[1],bbox.GetMax()[2]))
p.AddTransformOp().Set(mount * UsdGeom.XformCache().GetLocalToWorldTransform(baseprim))
p.GetPrim().SetCustomDataByKey('mount','Fixed to base_link; bottom on breadboard top; XY centered')
stage.GetRootLayer().Save()
report={'table_translation_m':list(p.GetOrderedXformOps()[0].Get().ExtractTranslation()),'materials':{n:{'linear_color':c,'metallic':m,'roughness':r} for n,(c,m,r) in SPECS.items()},'parts_placed_in_robot_scene':False}
(OUT/'materials_report.json').write_text(json.dumps(report,indent=2));print(report)
