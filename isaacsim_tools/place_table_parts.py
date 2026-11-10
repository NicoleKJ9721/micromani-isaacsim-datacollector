"""Reproducible random tabletop layout, metres; preserves source assets."""
from pathlib import Path
import shutil,json,random,math
import numpy as np
from pxr import Usd,UsdGeom,Gf
ROOT=Path(__file__).resolve().parents[1];src=ROOT/'outputs/material_table_20261109';out=ROOT/'outputs/table_parts_20261109';out.mkdir(exist_ok=True)
for name in ['operation_table.usd','gold_cavity.usd','sleeve.usd','micromani_equipped.usda']:
 shutil.copy2(src/name,out/name)
for p in src.glob('*.png'):shutil.copy2(p,out/p.name)
s=Usd.Stage.Open(str(out/'micromani_equipped.usda'));table=UsdGeom.Xformable(s.GetPrimAtPath('/World/OperationTable'))
m=table.GetOrderedXformOps()[0].Get();t=m.ExtractTranslation();m.SetTranslate(Gf.Vec3d(t[0],t[1]-.03,t[2]));table.GetOrderedXformOps()[0].Set(m)
table.GetPrim().SetCustomDataByKey('mount','Fixed on base_link; centered X, offset Y -30 mm')
cache=UsdGeom.XformCache();tris=[]
for p in Usd.PrimRange(table.GetPrim()):
 if p.IsA(UsdGeom.Mesh):
  mesh=UsdGeom.Mesh(p);v=np.array([cache.GetLocalToWorldTransform(p).Transform(Gf.Vec3d(*x)) for x in mesh.GetPointsAttr().Get()]);ix=np.array(mesh.GetFaceVertexIndicesAttr().Get());assert all(n==3 for n in mesh.GetFaceVertexCountsAttr().Get());tris.extend(v[ix.reshape(-1,3)])
tris=np.array(tris);tris=tris[tris[:,:,2].max(1)>.19]
a=tris[:,0,:2];b=tris[:,1,:2]-a;c=tris[:,2,:2]-a;det=b[:,0]*c[:,1]-b[:,1]*c[:,0];valid=abs(det)>1e-16
a,b,c,det,tris=a[valid],b[valid],c[valid],det[valid],tris[valid]
def height(x,y):
 d=np.array([x,y])-a;u=(d[:,0]*c[:,1]-d[:,1]*c[:,0])/det;v=(b[:,0]*d[:,1]-b[:,1]*d[:,0])/det;ok=(u>=-1e-8)&(v>=-1e-8)&(u+v<=1+1e-8)
 z=tris[:,0,2]+u*(tris[:,1,2]-tris[:,0,2])+v*(tris[:,2,2]-tris[:,0,2]);return max(z[ok],default=-1)
rng=random.Random(2026110903);placed=[]
for name in ['gold_cavity']*3+['sleeve']*3:
 radius=.00135 if name=='gold_cavity' else .0045
 for attempt in range(10000):
  x=rng.uniform(-.039,.039);y=-.03+rng.uniform(-.026,.026)
  if any(math.hypot(x-r['x'],y-r['y'])<radius+r['radius']+.003 for r in placed):continue
  samples=[(x,y)]+[(x+radius*math.cos(k*math.pi/12),y+radius*math.sin(k*math.pi/12)) for k in range(24)]
  if all(abs(height(xx,yy)-.1984)<1e-7 for xx,yy in samples):break
 else:raise RuntimeError('No flat non-overlapping placement')
 idx=sum(r['asset']==name for r in placed)+1;xf=UsdGeom.Xform.Define(s,f'/World/TableParts/{name}_{idx}');xf.GetPrim().GetReferences().AddReference('./'+name+'.usd')
 # Gold's notched end is -X: +90 degrees about Y points that end upwards.
 rot=Gf.Rotation(Gf.Vec3d(0,1,0),90) if name=='gold_cavity' else Gf.Rotation(Gf.Vec3d(0,0,1),rng.uniform(0,360))
 points=[]
 for p in Usd.PrimRange(xf.GetPrim()):
  if p.IsA(UsdGeom.Mesh):points.extend(rot.TransformDir(Gf.Vec3d(*v)) for v in UsdGeom.Mesh(p).GetPointsAttr().Get())
 z=.1984-min(v[2] for v in points);xf.AddTranslateOp().Set(Gf.Vec3d(x,y,z));xf.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
 xf.GetPrim().SetCustomDataByKey('placement','Resting visual layout; not grasped')
 placed.append(dict(asset=name,index=idx,x=x,y=y,z=z,radius=radius,up='notched end' if name=='gold_cavity' else 'short side'))
s.GetRootLayer().Save();(out/'placement.json').write_text(json.dumps({'seed':2026110903,'table_offset_y_m':-.03,'parts':placed},indent=2));print(json.dumps(placed,indent=2))
