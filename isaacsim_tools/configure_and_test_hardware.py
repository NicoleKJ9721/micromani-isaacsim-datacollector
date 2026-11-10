"""配置独立仿真场景并验证本体、传感器和渲染；不修改URDF几何。"""
import argparse
import json
import math
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--stage',required=True)
    p.add_argument('--out',required=True)
    a=p.parse_args()
    out=Path(a.out).resolve(); out.mkdir(parents=True,exist_ok=True)
    from isaacsim import SimulationApp
    kit=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':640,'height':480})
    import numpy as np
    import omni.usd
    import omni.kit.commands
    from PIL import Image
    from pxr import Usd, UsdGeom, UsdShade, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.sensors.camera import Camera
    from isaacsim.sensors.physics import ContactSensor
    from micromani_configure_cameras import CAMERA_POSES,set_world_camera_pose,focal_length_for_view,camera_matrix,wrist_work_target
    from micromani_lerobot_schema import CAMERA_SPECS,DEFAULT_HOME_ACTION,RAW_DOF_NAMES,CONTROL_ORDER,hkvl36_model_spec,FORCE_TORQUE_SENSORS
    from micromani_collect_isaac import get_dof_names,apply_control_action as set_positions,read_force_torque,as_uint8_rgb
    from micromani_lerobot_schema import control_action_to_raw_targets, GRIPPER_MAP, JOINT_LIMITS
    def apply_control_action(robot,index,action):
        targets=control_action_to_raw_targets(action)
        ids=np.array([index[n] for n in targets],dtype=np.int32)
        values=np.array([list(targets.values())],dtype=np.float32)
        robot.set_joint_position_targets(values,joint_indices=ids)
        set_positions(robot,index,action)
        robot.set_joint_velocities(np.zeros_like(values),joint_indices=ids)
    from micromani_grip_sensing import read_grip_sensors
    def set_translation(prim, position):
        xf=UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*position))

    report={'source_stage':str(Path(a.stage).resolve()),'checks':{},'source':'Isaac Sim runtime',
            'spec_source':'https://www.hkvtech.cn/archives/125/'}
    try:
        if not omni.usd.get_context().open_stage(str(Path(a.stage).resolve())):
            raise RuntimeError('cannot open source stage')
        for _ in range(40): kit.update()
        stage=omni.usd.get_context().get_stage()
        # 输出flatten场景，原导入层不做修改。
        stage.Flatten().Export(str(out/'micromani_equipped.usda'))
        omni.usd.get_context().open_stage(str(out/'micromani_equipped.usda'))
        for _ in range(20): kit.update()
        stage=omni.usd.get_context().get_stage()
        root=stage.GetPrimAtPath('/micromanipulator')
        if not root: raise RuntimeError('canonical /micromanipulator root required')
        for _ in range(4):
            for prim in list(stage.Traverse()):
                if prim.IsInstance(): prim.SetInstanceable(False)
        def material(name,color,metallic,roughness,textured=False):
            m=UsdShade.Material.Define(stage,f'/World/Looks/{name}')
            s=UsdShade.Shader.Define(stage,f'{m.GetPath()}/Surface'); s.CreateIdAttr('UsdPreviewSurface')
            s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            s.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic)
            s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness)
            if textured:
                uv=UsdShade.Shader.Define(stage,f'{m.GetPath()}/UV');uv.CreateIdAttr('UsdPrimvarReader_float2')
                uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');uv.CreateOutput('result',Sdf.ValueTypeNames.Float2)
                tex=UsdShade.Shader.Define(stage,f'{m.GetPath()}/Texture');tex.CreateIdAttr('UsdUVTexture')
                tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('brushed_aluminium.png'))
                tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
                tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat')
                tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
                s.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
            s.CreateOutput('surface',Sdf.ValueTypeNames.Token)
            m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface')
            return m
        from micromani_appearance import apply_appearance
        silver,dark,steel,report['materials']=apply_appearance(stage,out)
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
        # 本体质量沿用CAD；仅增加传感器可视包络，不重复叠加未知装配质量。
        spec=hkvl36_model_spec('HKVL-36A')
        report['wrist_sensor_spec']=spec
        for name,info in FORCE_TORQUE_SENSORS.items():
            parent=UsdGeom.Xform.Define(stage,info['sensor_prim_path']).GetPrim()
            parent.SetCustomDataByKey('hardware_spec',json.dumps(spec,ensure_ascii=False))
            parent.SetCustomDataByKey('source','simulated_joint_reaction')
            body=UsdGeom.Cylinder.Define(stage,info['sensor_prim_path']+'/Envelope')
            body.CreateRadiusAttr(.018);body.CreateHeightAttr(.021);body.CreateAxisAttr('Y')
            UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(silver)
            face=UsdGeom.Cylinder.Define(stage,info['sensor_prim_path']+'/ReferenceDarkFace')
            face.CreateRadiusAttr(.0178);face.CreateHeightAttr(.0002);face.CreateAxisAttr('Y')
            set_translation(face,(0,-.0106,0))
            UsdShade.MaterialBindingAPI.Apply(face.GetPrim()).Bind(dark)
            face.GetPrim().SetCustomDataByKey('appearance_reference','HKVL-36 series silver housing and dark end face; visual only')
            parent.SetCustomDataByKey('geometry_status','dimensioned envelope, not manufacturer CAD; visual only')
        dome=UsdLux.DomeLight.Define(stage,'/World/Fill');dome.CreateIntensityAttr(650)
        for name,pos,power in [('Key',(.2,-.2,.4),600),('Rim',(-.2,.12,.35),450)]:
            light=UsdLux.RectLight.Define(stage,f'/World/{name}');light.CreateIntensityAttr(power)
            light.CreateWidthAttr(.3);light.CreateHeightAttr(.3)
            set_translation(light, pos)
        distant=UsdLux.DistantLight.Get(stage,'/World/DistantLight')
        if distant: distant.GetIntensityAttr().Set(650)
        report['cameras']={}
        for path,s in CAMERA_POSES.items():
            set_world_camera_pose(stage,path,s['eye'],s['target'])
            cam=UsdGeom.Camera.Get(stage,path)
            distance=math.dist(s['eye'],s['target'])
            cam.CreateHorizontalApertureAttr(5.184);cam.CreateVerticalApertureAttr(3.888)
            cam.CreateFocalLengthAttr(focal_length_for_view(s['view_width_m'],distance,5.184))
            cam.CreateFocusDistanceAttr(distance);cam.CreateFStopAttr(0.)
            cam.CreateClippingRangeAttr(Gf.Vec2f(.0001,5.))
            cam.GetPrim().SetCustomDataByKey('resolution','640x480')
            cam.GetPrim().SetCustomDataByKey('fps',30)
            report['cameras'][path]={'resolution':[640,480],'fps':30,'focal_length_mm':cam.GetFocalLengthAttr().Get(),
                                    'view_width_m':s['view_width_m'],'calibration':'synthetic pinhole; not real-camera calibration',
                                    'aperture_mm':[5.184,3.888],
                                    'sensor_reference':'Sony IMX335 recording area 2592x1944 at 2 um; E2E interface labels all three channels IMX335; physical camera models unconfirmed',
                                    'optics_source':'https://www.sony-semicon.com/files/62/flyer_security/IMX335LQN_Flyer.pdf'}
        # URDF导入默认驱动无阻尼，重力下Z轴明显下沉；使用显式仿真伺服增益。
        report['servo']={'source':'simulation tuning, not measured hardware parameters',
                         'linear_stiffness':100000.,'linear_damping':632.,
                         'angular_stiffness':10000.,'angular_damping':200.}
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.PrismaticJoint) or prim.IsA(UsdPhysics.RevoluteJoint):
                linear=prim.IsA(UsdPhysics.PrismaticJoint)
                drive=UsdPhysics.DriveAPI.Apply(prim,'linear' if linear else 'angular')
                drive.CreateTypeAttr('acceleration')
                drive.CreateStiffnessAttr(100000. if linear else 10000.)
                drive.CreateDampingAttr(632. if linear else 200.)
        # 每指一个接触传感器，避免用腕部合力冒充夹持力。
        sensors={}
        for side in ('left','right'):
            for finger in (1,2):
                body=f'/micromanipulator/link_finger{finger}_{side}'
                UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(body))
                UsdPhysics.MeshCollisionAPI.Apply(stage.GetPrimAtPath(body)).CreateApproximationAttr('convexHull')
                PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(body)).CreateThresholdAttr(0.)
                sensor=ContactSensor(prim_path=body+'/GripContact',name=f'grip_{side}_{finger}',dt=1/240,
                                     min_threshold=0.,max_threshold=1000.,radius=-1.)
                sensors[(side,finger)]=sensor
        floor=stage.GetPrimAtPath('/groundPlane')
        if floor:
            UsdGeom.Xformable(floor).ClearXformOpOrder()
            UsdGeom.Xformable(floor).AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0,0,-.18))
        stage.GetRootLayer().Save()
        world=World(stage_units_in_meters=1.,physics_dt=1/240,rendering_dt=1/30)
        world.reset();robot=Articulation('/micromanipulator/root_joint');robot.initialize()
        names=get_dof_names(robot);indices={n:i for i,n in enumerate(names)}
        report['checks']['16_raw_dof']=set(RAW_DOF_NAMES)==set(names)
        report['checks']['14_logical_axes']=len(CONTROL_ORDER)==14
        actual_limits = np.asarray(robot.get_dof_limits())[0]
        limits = {}
        for name in CONTROL_ORDER:
            dofs = GRIPPER_MAP.get(name, (name,))
            limits[name] = {dof: actual_limits[indices[dof]].tolist() for dof in dofs}
        report['runtime_joint_limits_si'] = limits
        report['checks']['all_runtime_limits_match_control'] = all(
            np.allclose(pair, JOINT_LIMITS[name], rtol=0., atol=1e-6)
            for name, dofs in limits.items() for pair in dofs.values()
        )
        for sensor in sensors.values():sensor.initialize()
        cameras={}
        for name,s in CAMERA_SPECS.items():
            c=Camera(prim_path=s['prim_path'],resolution=(640,480),frequency=30);c.initialize();c.add_rgb_to_frame();cameras[name]=c
        for _ in range(120):
            apply_control_action(robot,indices,list(DEFAULT_HOME_ACTION));world.step(render=True)
        # 以home姿态下的实际包围盒配置视野，避免旧场景的固定世界坐标偏离夹爪。
        runtime_bodies={}
        for side in ('left','right'):
            for link in ('link6','link_finger1','link_finger2'):
                path=f'/micromanipulator/{link}_{side}'
                runtime_bodies[path]=robot.get_body_index(f'{link}_{side}')
        def actual_matrix(path):
            pose=np.asarray(robot._physics_view.get_link_transforms())[0,runtime_bodies[path]]
            position,q=pose[:3],pose[3:]
            transform=Gf.Transform();transform.SetTranslation(Gf.Vec3d(*map(float,position)))
            transform.SetRotation(Gf.Rotation(Gf.Quatd(float(q[3]),Gf.Vec3d(*map(float,q[:3])))))
            return transform.GetMatrix()
        def bbox(path):
            cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render])
            box=cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
            if path not in runtime_bodies:return box
            old=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            correction=old.GetInverse()*actual_matrix(path)
            result=Gf.Range3d()
            for i in range(8):result.UnionWith(correction.Transform(box.GetCorner(i)))
            return result
        def finger_mesh_points(path):
            body = stage.GetPrimAtPath(path)
            cache = UsdGeom.XformCache()
            correction = cache.GetLocalToWorldTransform(body).GetInverse() * actual_matrix(path)
            points = []
            for prim in Usd.PrimRange(body):
                if prim.IsA(UsdGeom.Mesh):
                    matrix = cache.GetLocalToWorldTransform(prim) * correction
                    points.extend(matrix.Transform(Gf.Vec3d(*map(float, p)))
                                  for p in UsdGeom.Mesh(prim).GetPointsAttr().Get())
            return np.asarray(points)

        bounds=bbox('/micromanipulator'); center=bounds.GetMidpoint(); size=bounds.GetSize()
        report['robot_bounds']={'min':list(bounds.GetMin()),'max':list(bounds.GetMax())}
        width=max(size[0],size[1],size[2]) * 1.3
        center=tuple(center)
        global_eye=(center[0]+width*.65,center[1]-width*.95,center[2]+width*.65)
        set_world_camera_pose(stage,'/World/global_camera',global_eye,center)
        cam=UsdGeom.Camera.Get(stage,'/World/global_camera')
        cam.GetFocalLengthAttr().Set(focal_length_for_view(width,math.dist(global_eye,center),5.184))
        wrist_targets={}
        for side in ('left','right'):
            paths = [f'/micromanipulator/link_finger{i}_{side}' for i in (1, 2)]
            b1, b2 = map(bbox, paths)
            report.setdefault('finger_bounds',{})[side]={'min1':list(b1.GetMin()),'max1':list(b1.GetMax()),'min2':list(b2.GetMin()),'max2':list(b2.GetMax())}
            inward_x = 1 if side == 'left' else -1
            target = wrist_work_target(*map(finger_mesh_points, paths), inward_x)
            wrist_targets[side]=target
            marker=UsdGeom.Cube.Define(stage,f'/World/ViewTarget_{side}');marker.CreateSizeAttr(.002)
            set_translation(marker, target)
            target_mat=material(f'ViewTarget_{side}',(.01,.35,.5),0.,.4)
            UsdShade.MaterialBindingAPI.Apply(marker.GetPrim()).Bind(target_mat)
            eye=(target[0]+(-.04 if side=='left' else .04),target[1]-.04,target[2]+.025)
            path=f'/micromanipulator/link6_{side}/{side}_wrist_camera'
            local=camera_matrix(eye,target)*actual_matrix(f'/micromanipulator/link6_{side}').GetInverse()
            xf=UsdGeom.Xformable(stage.GetPrimAtPath(path));xf.ClearXformOpOrder();xf.AddTransformOp().Set(local)
            UsdGeom.Camera.Get(stage,path).GetFocalLengthAttr().Set(focal_length_for_view(.04,math.dist(eye,target),5.184))
        for path in CAMERA_POSES:
            cam=UsdGeom.Camera.Get(stage,path)
            report['cameras'][path]['focal_length_mm']=cam.GetFocalLengthAttr().Get()
        report['cameras']['/World/global_camera']['view_width_m']=width
        for _ in range(12):world.step(render=True)
        def grip_readings():
            return read_grip_sensors(sensors)
        # 初始化后的Camera使用USD轴约定显式设置局部位姿，并筛选无遮挡视角。
        for side in ('left','right'):
            target=wrist_targets[side];camera=cameras[f'{side}_wrist']
            best=None;best_count=-1
            offsets = ((0, -.035, .075), (0, -.09, .03), (0, .035, .075),
                       (0, -.09, -.03), (0, .09, -.03), (-.08, 0, -.03), (.08, 0, -.03))
            for offset in offsets:
                eye=tuple(target[i]+offset[i] for i in range(3))
                local=camera_matrix(eye,target)*actual_matrix(f'/micromanipulator/link6_{side}').GetInverse()
                q=local.ExtractRotationQuat();t=local.ExtractTranslation()
                camera.set_local_pose(translation=np.array(t),orientation=np.array([q.GetReal(),*q.GetImaginary()]),camera_axes='usd')
                path=f'/micromanipulator/link6_{side}/{side}_wrist_camera'
                UsdGeom.Camera.Get(stage,path).GetFocalLengthAttr().Set(focal_length_for_view(.04,math.dist(eye,target),5.184))
                for _ in range(4):world.step(render=True)
                rgb=as_uint8_rgb(camera.get_rgba())
                count=int(((rgb[:,:,2].astype(float)>rgb[:,:,0].astype(float)+15)&(rgb[:,:,1].astype(float)>rgb[:,:,0].astype(float)+10)).sum())
                if count>best_count:best=(local,eye);best_count=count
                if count>20:break
            local,eye=best;q=local.ExtractRotationQuat()
            camera.set_local_pose(translation=np.array(local.ExtractTranslation()),orientation=np.array([q.GetReal(),*q.GetImaginary()]),camera_axes='usd')
            path=f'/micromanipulator/link6_{side}/{side}_wrist_camera'
            UsdGeom.Camera.Get(stage,path).GetFocalLengthAttr().Set(focal_length_for_view(.04,math.dist(eye,target),5.184))
            report['cameras'][path].update(eye=list(eye),target=list(target),target_color_pixels=best_count,
                target_definition='between inward CAD fingertips, 1 mm ahead at default home',
                focal_length_mm=UsdGeom.Camera.Get(stage,path).GetFocalLengthAttr().Get())
        for _ in range(4):world.step(render=True)
        report['checks']['physics_handle']=bool(robot.is_physics_handle_valid())
        # Conservative world-space bounds on runtime bodies, not the authored rest pose.
        finger_bounds={f'{finger}_{side}':bbox(f'/micromanipulator/{finger}_{side}')
                       for side in ('left','right') for finger in ('link_finger1','link_finger2')}
        clearances=[]
        for left in ('link_finger1_left','link_finger2_left'):
            for right in ('link_finger1_right','link_finger2_right'):
                a,b=finger_bounds[left],finger_bounds[right]
                separation=np.maximum(np.asarray(a.GetMin())-np.asarray(b.GetMax()),np.asarray(b.GetMin())-np.asarray(a.GetMax()))
                clearances.append(float(np.linalg.norm(np.maximum(separation,0.))))
        report['home_action_model_si']=list(DEFAULT_HOME_ACTION)
        report['cross_arm_finger_aabb_clearance_m']=min(clearances)
        report['checks']['home_fingers_separated_10mm']=min(clearances)>=.010
        report['grip_unloaded']=grip_readings()
        report['checks']['grip_unloaded'] = all(
            value <= 1e-5 for reading in report['grip_unloaded'].values()
            for value in reading['finger_contact_n']
        )
        readings,vector=read_force_torque(robot,indices,'HKVL-36A')
        report['wrist_readings']=readings
        report['checks']['wrist_12_finite']=len(vector)==12 and bool(np.isfinite(vector).all())
        report['checks']['four_contact_sensors']=len(sensors)==4
        for name,c in cameras.items():
            rgb=as_uint8_rgb(c.get_rgba());Image.fromarray(rgb).save(out/f'{name}.png')
            report['cameras'][name]={'shape':list(rgb.shape),'mean':float(rgb.mean()),'std':float(rgb.std())}
            report['checks'][f'{name}_image']=rgb.shape==(480,640,3) and float(rgb.std())>2.
            if name != 'global':
                blue=(rgb[:,:,2].astype(float)>rgb[:,:,0].astype(float)+15)&(rgb[:,:,1].astype(float)>rgb[:,:,0].astype(float)+10)
                report['cameras'][name]['target_color_pixels']=int(blue.sum())
                report['checks'][f'{name}_target_visible']=int(blue.sum())>20
        # 逐逻辑轴施加小幅目标，读取真实仿真状态；恢复home后继续。
        import numpy as np
        from micromani_collect_isaac import get_control_state
        home = list(DEFAULT_HOME_ACTION)
        motion = []
        for i, name in enumerate(CONTROL_ORDER):
            step = .001 if i % 7 in (0, 1, 2, 6) else .02
            for direction in (-1, 1):
                apply_control_action(robot, indices, home)
                world.step(render=False)
                baseline = get_control_state(robot, names)[i]
                target = home.copy()
                target[i] += direction * step
                apply_control_action(robot, indices, target)
                world.step(render=False)
                actual = get_control_state(robot, names)[i]
                error = abs(actual - target[i])
                motion.append({'axis': name, 'direction': direction, 'baseline': baseline,
                               'target': target[i], 'actual': actual, 'error': error,
                               'passed': error < step / 4 and direction * (actual - baseline) > step / 2})
        apply_control_action(robot, indices, home)
        world.step(render=False)
        report['motion'] = motion
        report['checks']['14_axis_commands_both_directions'] = all(sample['passed'] for sample in motion)
        # 对各夹指表面施加小型固定探针，验证接触响应；非抓取任务结果。
        probes=[]
        for side, finger in sensors:
            points = finger_mesh_points(f'/micromanipulator/link_finger{finger}_{side}')
            top = points[np.abs(points[:, 2] - points[:, 2].max()) < 1e-5]
            point = top.mean(axis=0) + np.array([0., 0., .0008])
            report.setdefault('probe_positions',[]).append(list(point))
            probe=DynamicCuboid(prim_path=f'/World/TestProbe_{side}_{finger}',position=np.array(point),scale=np.array([.001,.001,.001]),mass=.001,size=1.,color=np.array([1.,.3,.1]))
            probes.append(probe)
        # 新增碰撞体后reset以确保进入物理场景。
        world.reset();robot.initialize()
        for sensor in sensors.values():sensor.initialize()
        apply_control_action(robot,indices,home)
        peaks={side:0. for side in ('left','right')}
        finger_peaks = {side: [0., 0.] for side in peaks}
        for _ in range(120):
            apply_control_action(robot,indices,home);world.step(render=False)
            values=grip_readings()
            for side in peaks:
                peaks[side] = max(peaks[side], values[side]['grip_force_n'])
                finger_peaks[side] = [max(old, current) for old, current in
                                      zip(finger_peaks[side], values[side]['finger_contact_n'])]
        report['contact_probe_peak_n']=peaks
        report['contact_probe_description']='four 1 g falling cubes; contact detection check, not force calibration'
        report['per_finger_contact_probe_peak_n'] = finger_peaks
        for side, finger_values in finger_peaks.items():
            report['checks'][f'{side}_both_fingers_responded'] = all(v > 1e-5 for v in finger_values)
        # Save the safe home pose as the USD rest view, so opening the file is safe too.
        for _ in range(8):
            apply_control_action(robot,indices,home);world.step(render=False)
        poses=np.asarray(robot._physics_view.get_link_transforms())[0].copy()
        body_names=list(robot.body_names)
        for probe in probes:
            stage.RemovePrim(probe.prim_path)
        for side in ('left', 'right'):
            stage.RemovePrim(f'/World/ViewTarget_{side}')
        world.stop()
        cache=UsdGeom.XformCache()
        for name,pose in zip(body_names,poses):
            prim=stage.GetPrimAtPath(f'/micromanipulator/{name}')
            if not prim:continue
            pos,q=pose[:3],pose[3:]
            t=Gf.Transform();t.SetTranslation(Gf.Vec3d(*map(float,pos)))
            t.SetRotation(Gf.Rotation(Gf.Quatd(float(q[3]),Gf.Vec3d(*map(float,q[:3])))))
            local=t.GetMatrix()*cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse()
            xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder()
            for attr in list(prim.GetAttributes()):
                if attr.GetName().startswith('xformOp:'):prim.RemoveProperty(attr.GetName())
            xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(local.ExtractTranslation())
            xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(local.ExtractRotationQuat()))
            xf.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1,1,1))
        for name,value in control_action_to_raw_targets(home).items():
            for prim in stage.Traverse():
                if prim.GetName()!=name:continue
                linear=prim.IsA(UsdPhysics.PrismaticJoint)
                UsdPhysics.DriveAPI.Get(prim,'linear' if linear else 'angular').GetTargetPositionAttr().Set(value if linear else math.degrees(value))
        stage.GetRootLayer().Save()
        report['checks']['materials_bound']=report['materials']['mesh_count']>0
        report['passed']=all(report['checks'].values())
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    except BaseException as exc:
        import traceback
        report['error']=str(exc);traceback.print_exc()
    finally:
        (out/'hardware_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        if not report.get('passed',False): kit._app.post_quit(1)
        kit.close()


if __name__=='__main__':main()
