"""EPG photo-referenced material zoning on existing CAD faces only."""
import numpy as np

def apply_epg_finish(stage):
    from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf
    from micromani_appearance import connected_face_groups
    def material(name,color,metallic,roughness):
        m=UsdShade.Material.Define(stage,'/World/Looks/EPG/'+name)
        s=UsdShade.Shader.Define(stage,str(m.GetPath())+'/Surface');s.CreateIdAttr('UsdPreviewSurface')
        for key,val,typ in [('diffuseColor',Gf.Vec3f(*color),Sdf.ValueTypeNames.Color3f),('metallic',metallic,Sdf.ValueTypeNames.Float),('roughness',roughness,Sdf.ValueTypeNames.Float)]:s.CreateInput(key,typ).Set(val)
        s.CreateOutput('surface',Sdf.ValueTypeNames.Token);m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface')
        return m
    mats={'housing':material('CharcoalSatin',(.095,.10,.105),.15,.52),
          'endcap':material('DarkEndcaps',(.022,.024,.026),.12,.48),
          'jaw':material('JawFinish',(.045,.048,.051),.35,.40),
          'slider':material('SilverSlides',(.55,.57,.59),.88,.25),
          'cable':material('CableRubber',(.009,.010,.011),0.,.72),
          'indicator':material('BlueIndicator',(.005,.12,.38),.12,.23)}
    report=[]
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath('/micromanipulator'))):
        if not prim:continue
        if not prim.IsA(UsdGeom.Mesh) or '/visuals/' not in str(prim.GetPath()):continue
        path=str(prim.GetPath());link=path.split('/')[2];key=link.removesuffix('_left').removesuffix('_right')
        if key not in ('link6','link_finger1','link_finger2'):continue
        mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get());faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
        groups={k:[] for k in mats}
        for ids in connected_face_groups(v,faces):
            pts=v[faces[ids]];ext=np.ptp(pts.reshape(-1,3),axis=0);centers=pts.mean(1)
            if key.startswith('link_finger'):
                kind='slider' if np.allclose(np.sort(ext),[.010,.0105,.021],atol=.0015) else 'jaw'
                groups[kind].extend(ids.tolist())
            elif np.allclose(ext,[.070,.035,.035],atol=.001):
                # Split the existing housing face regions; +X points toward jaws on model left.
                towards=centers[:,0]*(1 if link.endswith('_left') else -1)
                groups['endcap'].extend(ids[towards>.025].tolist());groups['housing'].extend(ids[towards<=.025].tolist())
            elif np.allclose(ext,[.005,.035,.035],atol=.001):groups['endcap'].extend(ids.tolist())
            elif np.allclose(ext,[.039,.0241,.014],atol=.002):
                metal=(np.abs(centers[:,0])>.0395)&(np.abs(centers[:,0])<.048)
                groups['slider'].extend(ids[metal].tolist());groups['cable'].extend(ids[~metal].tolist())
            elif len(ids)==746 and max(ext)<.004:groups['indicator'].extend(ids.tolist())
        claimed=set(i for ids in groups.values() for i in ids)
        for child in list(prim.GetChildren()):
            if child.IsA(UsdGeom.Subset):
                subset=UsdGeom.Subset(child)
                if child.GetName().startswith('EPG_'):stage.RemovePrim(child.GetPath());continue
                indices=subset.GetIndicesAttr().Get()
                remaining=[i for i in indices if i not in claimed]
                if remaining:subset.GetIndicesAttr().Set(remaining)
                else:stage.RemovePrim(child.GetPath())
        for kind,ids in groups.items():
            if not ids:continue
            sub=UsdGeom.Subset.Define(stage,path+'/EPG_'+kind);sub.CreateElementTypeAttr('face');sub.CreateFamilyNameAttr('materialBind');sub.CreateIndicesAttr(ids)
            UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mats[kind])
            report.append({'link':link,'finish':kind,'faces':len(ids)})
    if sum(r['finish']=='housing' for r in report)!=2:raise RuntimeError('Expected two EPG housing components')
    return report
