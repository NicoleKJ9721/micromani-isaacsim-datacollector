"""Reference-based surface materials; keeps CAD vertices, joints and collision geometry."""
from pathlib import Path
import json
import numpy as np

def connected_face_groups(vertices,faces):
 _,inverse=np.unique(np.round(vertices,6),axis=0,return_inverse=True)
 faces=inverse[faces];parent=list(range(int(inverse.max())+1))
 def find(a):
  while parent[a]!=a:parent[a]=parent[parent[a]];a=parent[a]
  return a
 for face in faces:
  a=find(int(face[0]))
  for b in face[1:]:
   b=find(int(b))
   if a!=b:parent[b]=a
 labels=np.array([find(int(f[0])) for f in faces])
 return [np.where(labels==x)[0] for x in np.unique(labels)]

def apply_appearance(stage, output_dir):
    import numpy as np
    from PIL import Image
    from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
    out=Path(output_dir)
    rng=np.random.default_rng(36)
    # Roughness-only microstructure: surface finish, not copied catalog watermarks.
    grain=np.clip(.34+rng.normal(0,.022,(512,1))+rng.normal(0,.006,(512,512)),.2,.5)
    Image.fromarray((grain*255).astype('uint8')).save(out/'satin_metal_roughness.png')
    grain=np.clip(.47+rng.normal(0,.012,(512,512)),.4,.55)
    Image.fromarray((grain*255).astype('uint8')).save(out/'anodized_roughness.png')
    def make(name,color,metallic,roughness,texture=None):
        mat=UsdShade.Material.Define(stage,f'/World/Looks/Reference/{name}')
        surf=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');surf.CreateIdAttr('UsdPreviewSurface')
        surf.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        surf.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic)
        surf.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness)
        if texture:
            uv=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2')
            uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');uv.CreateOutput('result',Sdf.ValueTypeNames.Float2)
            tex=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Roughness');tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
            tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('raw')
            tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
            tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat')
            tex.CreateOutput('r',Sdf.ValueTypeNames.Float)
            surf.GetInput('roughness').ConnectToSource(tex.ConnectableAPI(),'r')
        surf.CreateOutput('surface',Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(),'surface')
        return mat
    silver=make('SatinSilver',(.68,.70,.72),.72,.34,'satin_metal_roughness.png')
    black=make('BlackAnodized',(.022,.025,.028),.35,.47,'anodized_roughness.png')
    steel=make('MachinedSteel',(.43,.46,.49),.95,.25)
    gripper=make('EPGDarkHousing',(.027,.030,.033),.35,.43,'anodized_roughness.png')
    # The imported white parent bindings used strongerThanDescendants and masked earlier edits.
    root=stage.GetPrimAtPath('/micromanipulator')
    for prim in Usd.PrimRange(root):
        UsdShade.MaterialBindingAPI.Apply(prim).UnbindAllBindings()
    roles={ 'base_link':('mounting_plate',silver),
           'link0':('KXL06075_X_and_support',silver),
           'link1':('KXL06050_Y',silver),
           'link2':('KXL06075_Z',silver),
           'link3':('KRW06360_RX',black),
           'link4':('KGW04040_RZ',black),
           'link5':('KRW04360_RY',black),
           'link6':('EPG006_housing_and_mount',gripper),
           'link_finger1':('EPG006_finger',black),
           'link_finger2':('EPG006_finger',black)}
    assigned=[]
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):continue
        path=str(prim.GetPath());parts=path.split('/')
        if any('collision' in p.lower() for p in parts):continue
        link=parts[2];key=link.removesuffix('_left').removesuffix('_right')
        if key not in roles:continue
        role,mat=roles[key]
        mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get())
        if len(points):
            axes=np.argsort(np.ptp(points,axis=0))[-2:]
            # Tile fine finish over each CAD component, no physical geometry displacement.
            uv=(points[:,axes]-points[:,axes].min(0))/.02
            UsdGeom.PrimvarsAPI(prim).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set([Gf.Vec2f(*map(float,v)) for v in uv])
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.weakerThanDescendants)
        prim.SetCustomDataByKey('appearance_reference',role)
        effective=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not effective or effective.GetPath()!=mat.GetPath():raise RuntimeError(f'material overridden: {path}')
        assigned.append({'mesh':path,'role':role,'material':str(mat.GetPath())})
        counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        if len(counts) and np.all(counts==3):
            faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
            subsets={'motor':[],'fastener':[]}
            for indices in connected_face_groups(points,faces):
                extent=np.ptp(points[faces[indices]].reshape(-1,3),axis=0)
                if key in ('link0','link1','link2') and np.allclose(np.sort(extent),[.028,.033,.0475],atol=.0025):
                    subsets['motor'].extend(indices.tolist())
                elif max(extent)<.012 and len(indices)>=48:
                    subsets['fastener'].extend(indices.tolist())
            for kind,indices in subsets.items():
                if not indices:continue
                subset=UsdGeom.Subset.Define(stage,path+'/Reference_'+kind)
                subset.CreateElementTypeAttr('face');subset.CreateFamilyNameAttr('materialBind');subset.CreateIndicesAttr(indices)
                submat=black if kind=='motor' else steel
                UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(submat)
                assigned.append({'mesh':str(subset.GetPath()),'role':kind,'material':str(submat.GetPath()),'faces':len(indices)})

    report={'mesh_count':sum('faces' not in x for x in assigned),'subset_count':sum('faces' in x for x in assigned),'approach':'photo-referenced component finishes; CAD geometry retained',
            'references':'PLATFORM_HARDWARE.md and docs/platform_hardware/catalog',
            'limitations':['catalog variants are appearance references, not confirmed full order codes',
                          'mixed-part STL links use dominant component finish; no invented vendor labels',
                          'surface roughness is a visual approximation, not measured BRDF'],
            'bindings':assigned}
    from micromani_epg_finish import apply_epg_finish
    report['epg_finish']=apply_epg_finish(stage)
    from micromani_platform_finish import apply_platform_finish
    report['platform_finish']=apply_platform_finish(stage)
    (out/'appearance_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return silver,black,steel,report
