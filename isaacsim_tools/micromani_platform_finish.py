"""Photo-referenced material zoning for the non-EPG platform components."""
import numpy as np

def apply_platform_finish(stage):
    from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf
    from micromani_appearance import connected_face_groups
    def mat(name,color,metallic,roughness):
        m=UsdShade.Material.Define(stage,'/World/Looks/PlatformPhoto/'+name)
        s=UsdShade.Shader.Define(stage,str(m.GetPath())+'/Surface');s.CreateIdAttr('UsdPreviewSurface')
        for k,v,t in [('diffuseColor',Gf.Vec3f(*color),Sdf.ValueTypeNames.Color3f),('metallic',metallic,Sdf.ValueTypeNames.Float),('roughness',roughness,Sdf.ValueTypeNames.Float)]:s.CreateInput(k,t).Set(v)
        s.CreateOutput('surface',Sdf.ValueTypeNames.Token);m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m
    mats={'silver':mat('SatinMachinedAluminium',(.58,.60,.62),.72,.38),
          'black':mat('BlackAnodizedHousing',(.022,.024,.027),.28,.43),
          'rubber':mat('CableInsulation',(.009,.010,.011),0.,.72)}
    report=[]
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath('/micromanipulator'))):
        if not prim:continue
        path=str(prim.GetPath())
        if not prim.IsA(UsdGeom.Mesh) or '/visuals/' not in path:continue
        key=path.split('/')[2].removesuffix('_left').removesuffix('_right')
        if key not in ('link0','link1','link2','link3','link4','link5','link6'):continue
        mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get());faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
        regions={k:[] for k in mats}
        for ids in connected_face_groups(v,faces):
            p=v[faces[ids]];lo=p.reshape(-1,3).min(0);hi=p.reshape(-1,3).max(0);extent=hi-lo;c=p.mean(1)
            if key=='link0' and hi[2]<=.116 and lo[2]>=.009 and extent[1]>.10:
                regions['black'].extend(ids.tolist()) # black lift framework seen in platform photograph
            elif key in ('link0','link1','link2') and np.allclose(np.sort(extent),[.028,.033,.0475],atol=.0025):
                axis=int(np.argmax(extent));caps=(c[:,axis]<lo[axis]+.004)|(c[:,axis]>hi[axis]-.004)
                regions['silver'].extend(ids[caps].tolist());regions['black'].extend(ids[~caps].tolist())
            elif key=='link3' and extent[1]>.15:
                u=np.abs(c[:,1])
                flange=(u<.047)|((u>.072)&(u<.079))
                regions['silver'].extend(ids[flange].tolist());regions['rubber'].extend(ids[u>.085].tolist())
            elif key=='link4' and extent[1]>.20:
                u=np.abs(c[:,1])
                flange=(u>.030)&(u<.039)
                connector=(u>.072)&(u<.080)
                regions['silver'].extend(ids[flange|connector].tolist());regions['rubber'].extend(ids[u>.085].tolist())
            elif key=='link5' and extent[2]>.15:
                u=-np.abs(c[:,2])
                flange=(u>-.032)|((u<-.065)&(u>-.075))
                regions['silver'].extend(ids[flange].tolist());regions['rubber'].extend(ids[u<-.080].tolist())
            elif key=='link6':
                if np.allclose(extent,[.039,.0758,.039],atol=.001):
                    caps=(c[:,1]>-.007)|(c[:,1]<-.069)
                    regions['silver'].extend(ids[~caps].tolist());regions['black'].extend(ids[caps].tolist())
                elif len(ids) in (638,92,1362,1332):
                    regions['silver'].extend(ids.tolist()) # machined adapter/camera brackets, not EPG housing
        claimed={i for indices in regions.values() for i in indices}
        epg=set()
        for child in list(prim.GetChildren()):
            if not child.IsA(UsdGeom.Subset):continue
            if child.GetName().startswith('EPG_'):epg.update(UsdGeom.Subset(child).GetIndicesAttr().Get())
        if claimed&epg:raise RuntimeError('Platform finish would overwrite EPG faces')
        for child in list(prim.GetChildren()):
            if not child.IsA(UsdGeom.Subset):continue
            if child.GetName().startswith('Photo_'):stage.RemovePrim(child.GetPath());continue
            sub=UsdGeom.Subset(child);remaining=[i for i in sub.GetIndicesAttr().Get() if i not in claimed]
            if remaining:sub.GetIndicesAttr().Set(remaining)
            else:stage.RemovePrim(child.GetPath())
        for kind,ids in regions.items():
            if not ids:continue
            sub=UsdGeom.Subset.Define(stage,path+'/Photo_'+kind);sub.CreateElementTypeAttr('face');sub.CreateFamilyNameAttr('materialBind');sub.CreateIndicesAttr(ids)
            UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mats[kind]);report.append({'mesh':path,'finish':kind,'faces':len(ids)})
    return report
