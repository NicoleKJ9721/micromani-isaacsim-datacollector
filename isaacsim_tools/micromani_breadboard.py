"""Parametric breadboard visual mesh, SI units; blind holes, no simulated screw threads."""
import math,struct
from pathlib import Path
import numpy as np

def build_mesh(c):
    length,width,thick=(c[k] for k in ('length_m','width_m','thickness_m'))
    pitch,diam,depth,margin=(c[k] for k in ('pitch_m','hole_diameter_m','hole_depth_m','edge_margin_m'))
    if not (0<diam<pitch and 0<depth<thick and min(length,width)>2*margin and margin>diam/2):raise ValueError('invalid breadboard dimensions')
    nx=int((length/2-margin)/pitch+1e-9);ny=int((width/2-margin)/pitch+1e-9)
    xs=np.arange(-nx,nx+1)*pitch;ys=np.arange(-ny,ny+1)*pitch
    xb=[-length/2,*((xs[:-1]+xs[1:])/2),length/2];yb=[-width/2,*((ys[:-1]+ys[1:])/2),width/2]
    vertices=[];faces=[]
    def polygon(coords):
        k=len(vertices);vertices.extend(coords)
        for i in range(1,len(coords)-1):faces.append((k,k+i,k+i+1))
    for ix,x in enumerate(xs):
      for iy,y in enumerate(ys):
        outer=[];inner=[];bottom=[]
        angles=[2*math.pi*i/32 for i in range(32)]
        angles.extend(math.atan2(v-y,u-x)%(2*math.pi) for u in (xb[ix],xb[ix+1]) for v in (yb[iy],yb[iy+1]))
        angles=sorted(set(round(a,12) for a in angles))
        for a in angles:
            u,v=math.cos(a),math.sin(a)
            tx=(xb[ix+1]-x)/u if u>1e-9 else (xb[ix]-x)/u if u<-1e-9 else math.inf
            ty=(yb[iy+1]-y)/v if v>1e-9 else (yb[iy]-y)/v if v<-1e-9 else math.inf
            r=min(tx,ty);outer.append((x+r*u,y+r*v,thick));inner.append((x+diam/2*u,y+diam/2*v,thick));bottom.append((x+diam/2*u,y+diam/2*v,thick-depth))
        for i in range(len(angles)):
            j=(i+1)%len(angles);polygon([outer[i],outer[j],inner[j],inner[i]]);polygon([inner[i],inner[j],bottom[j],bottom[i]])
            polygon([(x,y,thick-depth),bottom[i],bottom[j]])
    corners=[(-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)]
    for i in range(4):
        x,y=corners[i];u,v=corners[(i+1)%4];polygon([(x,y,0),(u,v,0),(u,v,thick),(x,y,thick)])
    polygon([(x,y,0) for x,y in reversed(corners)])
    return np.asarray(vertices),np.asarray(faces),len(xs)*len(ys)

def write_stl(path,vertices,faces):
    with Path(path).open('wb') as f:
        f.write(b'Parametric breadboard, original base_link envelope'.ljust(80,b' '));f.write(struct.pack('<I',len(faces)))
        for ids in faces:
            t=vertices[ids];n=np.cross(t[1]-t[0],t[2]-t[0]);norm=np.linalg.norm(n);n=n/norm if norm else n
            f.write(struct.pack('<12fH',*n,*t.flatten(),0))
