"""Conservative CAD AABB validation for the default home configuration (metres)."""
import sys,xml.etree.ElementTree as E, numpy as np
from pathlib import Path
root=Path(__file__).resolve().parents[1]
from micromani_lerobot_schema import DEFAULT_HOME_ACTION,control_action_to_raw_targets
xml=E.parse(root/'urdf/微纳机器人URDF.urdf').getroot()
def rot(axis,a):
 axis=np.array(axis,dtype=float);axis/=np.linalg.norm(axis);x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);return np.eye(3)+np.sin(a)*K+(1-np.cos(a))*(K@K)
def transform(xyz,rpy):
 t=np.eye(4);t[:3,3]=xyz;r,p,y=rpy;t[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r);return t
def vals(e,k,default='0 0 0'):return list(map(float,e.get(k,default).split()))
meshes={}
for l in xml.findall('link'):
 v=l.find('visual');f=root/'meshes'/Path(v.find('geometry/mesh').get('filename')).name
 a=np.fromfile(f,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),offset=84)
 points=a['v'].reshape(-1,3).astype(float);o=v.find('origin');T=transform(vals(o,'xyz'),vals(o,'rpy'));meshes[l.get('name')]=points@T[:3,:3].T+T[:3,3]
def bounds(home):
 q=control_action_to_raw_targets(home);frames={'base_link':np.eye(4)}
 for j in xml.findall('joint'):
  o=j.find('origin');t=transform(vals(o,'xyz'),vals(o,'rpy'));m=np.eye(4);a=vals(j.find('axis'),'xyz');value=q.get(j.get('name'),0)
  if j.get('type')=='prismatic':m[:3,3]=np.array(a)*value
  elif j.get('type')=='revolute':m[:3,:3]=rot(a,value)
  frames[j.find('child').get('link')]=frames[j.find('parent').get('link')]@t@m
 return {n:np.stack(((v@frames[n][:3,:3].T+frames[n][:3,3]).min(0),(v@frames[n][:3,:3].T+frames[n][:3,3]).max(0))) for n,v in meshes.items()}
