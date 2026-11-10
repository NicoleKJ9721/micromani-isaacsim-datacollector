"""Build a separate dynamic assembly stage; never mutate supplied CAD or visual scene.
Run with USD, numpy and trimesh available. Physical coefficients are estimates.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/assembly_physics'

def attr(p,name,typ,value):
    p.CreateAttribute(name,typ,custom=False).Set(value)
def schema(p,name): p.AddAppliedSchema(name)
def contact(p,offset=2e-6):
    schema(p,'PhysxCollisionAPI')
    attr(p,'physxCollision:contactOffset',Sdf.ValueTypeNames.Float,offset)
    attr(p,'physxCollision:restOffset',Sdf.ValueTypeNames.Float,0.)
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    s=Usd.Stage.CreateNew(str(OUT/'micromani_assembly.usda'))
    s.GetRootLayer().subLayerPaths=['../table_parts_20261109/micromani_equipped.usda']
    UsdGeom.SetStageMetersPerUnit(s,1.); UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z)
    cfg=json.loads((ROOT/'config/assembly_contact.json').read_text())
    report={'contact_config':cfg,'assumptions':'Density, friction and drive limits are simulation estimates, not hardware calibration.',
            'physics_hz':960,'sensor_hz':30,'contact_offset_m':2e-6,'rest_offset_m':0.,'parts':[]}
    for asset,res in [('gold_cavity',1024),('sleeve',2048)]:
        density=cfg['materials'][asset]['density_kg_m3']
        mat=UsdShade.Material.Define(s,'/World/PhysicsMaterials/'+asset)
        pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(cfg['materials'][asset]['static_friction']);pm.CreateDynamicFrictionAttr(cfg['materials'][asset]['dynamic_friction']);pm.CreateRestitutionAttr(0.)
        schema(mat.GetPrim(),'PhysxMaterialAPI')
        attr(mat.GetPrim(),'physxMaterial:frictionCombineMode',Sdf.ValueTypeNames.Token,'multiply')
        for i in range(1,4):
            p=s.GetPrimAtPath(f'/World/TableParts/{asset}_{i}')
            UsdPhysics.RigidBodyAPI.Apply(p).CreateRigidBodyEnabledAttr(True)
            schema(p,'PhysxRigidBodyAPI')
            for name,value in [('solverPositionIterationCount',32),('solverVelocityIterationCount',8)]:
                attr(p,'physxRigidBody:'+name,Sdf.ValueTypeNames.Int,value)
            for name,value in [('sleepThreshold',0.),('stabilizationThreshold',0.),('maxDepenetrationVelocity',.02),('linearDamping',.02),('angularDamping',.02)]:
                attr(p,'physxRigidBody:'+name,Sdf.ValueTypeNames.Float,value)
            meshes=[]
            cache=UsdGeom.XformCache(); inv=cache.GetLocalToWorldTransform(p).GetInverse()
            for c in Usd.PrimRange(p):
                if not c.IsA(UsdGeom.Mesh):continue
                m=UsdGeom.Mesh(c); T=cache.GetLocalToWorldTransform(c)*inv
                v=np.array([T.Transform(Gf.Vec3d(*x)) for x in m.GetPointsAttr().Get()])
                counts=m.GetFaceVertexCountsAttr().Get(); assert all(n==3 for n in counts)
                faces=np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
                tm=trimesh.Trimesh(v,faces,process=True); assert tm.is_watertight and tm.volume>0
                meshes.append(tm)
                UsdPhysics.CollisionAPI.Apply(c)
                UsdPhysics.MeshCollisionAPI.Apply(c).CreateApproximationAttr('sdf')
                schema(c,'PhysxSDFMeshCollisionAPI')
                attr(c,'physxSDFMeshCollision:sdfResolution',Sdf.ValueTypeNames.Float,float(res))
                attr(c,'physxSDFMeshCollision:sdfSubgridResolution',Sdf.ValueTypeNames.Int,6)
                attr(c,'physxSDFMeshCollision:sdfNarrowBandThickness',Sdf.ValueTypeNames.Float,.01)
                contact(c)
                UsdShade.MaterialBindingAPI.Apply(c).Bind(mat,materialPurpose='physics')
            tm=trimesh.util.concatenate(meshes);tm.density=density
            eig,axes=np.linalg.eigh(tm.moment_inertia)
            if np.linalg.det(axes)<0:axes[:,0]*=-1
            inertia=UsdPhysics.MassAPI.Apply(p)
            inertia.CreateMassAttr(float(tm.mass));inertia.CreateCenterOfMassAttr(Gf.Vec3f(*tm.center_mass))
            inertia.CreateDiagonalInertiaAttr(Gf.Vec3f(*eig))
            inertia.CreatePrincipalAxesAttr(Gf.Quatf(Gf.Matrix3d(axes.T.tolist()).ExtractRotation().GetQuat()))
            p.SetCustomDataByKey('placement','Dynamic rigid body; no grasp attachment or pose reset')
            report['parts'].append(dict(path=str(p.GetPath()),mass_kg=tm.mass,volume_m3=tm.volume,density_kg_m3=density,sdf_resolution=res,estimated_grid_spacing_m=float(np.ptp(tm.vertices,axis=0).max()/res)))
    tpu=UsdShade.Material.Define(s,'/World/PhysicsMaterials/TPU_fingers')
    props=cfg['materials']['tpu_fingers'];pm=UsdPhysics.MaterialAPI.Apply(tpu.GetPrim())
    pm.CreateStaticFrictionAttr(props['static_friction']);pm.CreateDynamicFrictionAttr(props['dynamic_friction']);pm.CreateRestitutionAttr(0.)
    schema(tpu.GetPrim(),'PhysxMaterialAPI')
    attr(tpu.GetPrim(),'physxMaterial:frictionCombineMode',Sdf.ValueTypeNames.Token,'multiply')
    attr(tpu.GetPrim(),'physxMaterial:compliantContactStiffness',Sdf.ValueTypeNames.Float,props['contact_stiffness_n_m'])
    attr(tpu.GetPrim(),'physxMaterial:compliantContactDamping',Sdf.ValueTypeNames.Float,props['contact_damping_ns_m'])
    # URDF import put convex-hull collision on finger ancestor Xforms.
    # Remove inherited hulls, author only the dedicated collision meshes.
    for p in s.Traverse():
        path=str(p.GetPath())
        if not path.startswith('/micromanipulator/link_finger'):continue
        if p.HasAPI(UsdPhysics.CollisionAPI):p.RemoveAPI(UsdPhysics.CollisionAPI)
        if p.HasAPI(UsdPhysics.MeshCollisionAPI):p.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        if p.IsA(UsdGeom.Mesh) and '/collisions/' in path:
            mesh=UsdGeom.Mesh(p)
            vertices=np.asarray(mesh.GetPointsAttr().Get())
            faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
            # SDF contact generation samples triangles. Long CAD faces otherwise
            # supply too few contact points on a sub-millimetre gripping land.
            vertices,faces=trimesh.remesh.subdivide_to_size(vertices,faces,max_edge=.0005,max_iter=10)
            mesh.GetPointsAttr().Set(vertices.tolist());mesh.GetFaceVertexIndicesAttr().Set(faces.flatten().tolist())
            mesh.GetFaceVertexCountsAttr().Set([3]*len(faces));mesh.GetNormalsAttr().Block()
            mesh.CreateSubdivisionSchemeAttr('none')
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('sdf')
            schema(p,'PhysxSDFMeshCollisionAPI')
            attr(p,'physxSDFMeshCollision:sdfResolution',Sdf.ValueTypeNames.Float,1024.)
            attr(p,'physxSDFMeshCollision:sdfSubgridResolution',Sdf.ValueTypeNames.Int,6)
            attr(p,'physxSDFMeshCollision:sdfNarrowBandThickness',Sdf.ValueTypeNames.Float,.01)
    # Existing static table and finger geometry retain geometry; narrow contact skin.
    for p in s.Traverse():
        if p.HasAPI(UsdPhysics.CollisionAPI):
            if '/link_finger' in str(p.GetPath()):
                UsdShade.MaterialBindingAPI.Apply(p).Bind(tpu,materialPurpose='physics',bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            contact(p,2e-6 if ('finger' in str(p.GetPath()) or 'TableParts' in str(p.GetPath())) else 1e-5)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            schema(p,'PhysxArticulationAPI')
            attr(p,'physxArticulation:solverPositionIterationCount',Sdf.ValueTypeNames.Int,32)
            attr(p,'physxArticulation:solverVelocityIterationCount',Sdf.ValueTypeNames.Int,8)
        if p.IsA(UsdPhysics.PrismaticJoint) or p.IsA(UsdPhysics.RevoluteJoint):
            linear=p.IsA(UsdPhysics.PrismaticJoint); finger='finger' in p.GetName()
            d=UsdPhysics.DriveAPI.Get(p,'linear' if linear else 'angular')
            if d:
                d.CreateTypeAttr('force')
                d.CreateStiffnessAttr(2000. if finger else (2e6 if linear else 100.))
                d.CreateDampingAttr(4. if finger else (2800. if linear else 2.))
                d.CreateMaxForceAttr(cfg['gripper_drive_limits_n']['left' if p.GetName().endswith('left') else 'right'] if finger else (100. if linear else 2.))
    scenes=[p for p in s.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes: scenes=[UsdPhysics.Scene.Define(s,'/physicsScene').GetPrim()]
    for p in scenes:
        schema(p,'PhysxSceneAPI')
        attr(p,'physxScene:timeStepsPerSecond',Sdf.ValueTypeNames.UInt,960)
        attr(p,'physxScene:solverType',Sdf.ValueTypeNames.Token,'TGS')
        attr(p,'physxScene:enableGPUDynamics',Sdf.ValueTypeNames.Bool,True)
        attr(p,'physxScene:broadphaseType',Sdf.ValueTypeNames.Token,'GPU')
    s.GetRootLayer().customLayerData={'assemblyPhysics':{'left_gripper_drive_limit_n':cfg['gripper_drive_limits_n']['left'],'right_gripper_drive_limit_n':cfg['gripper_drive_limits_n']['right'],'physics_hz':960}}
    s.GetRootLayer().Save()
    (OUT/'physics_config.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
