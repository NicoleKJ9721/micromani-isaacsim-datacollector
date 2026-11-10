#!/usr/bin/env python3
"""本地 Isaac Sim 遥操作录制。终端控制 episode，UDP 接入 PICO/外部目标。"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
import queue
import socket
import sys
import threading
import time
import traceback
from pathlib import Path
from micromani_sim_contract import SimContract, validate_action
from micromani_episode import EpisodeSession


def parse_args():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',required=True)
    p.add_argument('--calibration',required=True)
    p.add_argument('--session',required=True)
    p.add_argument('--task',required=True)
    p.add_argument('--fps',type=int,default=30)
    p.add_argument('--initial-state-json',help='Public robot encoder snapshot for a new episode initialization only')
    p.add_argument('--profile-advance',action='store_true',help='Report measured controller, physics and rendering wall times')
    p.add_argument('--physics-hz',type=int,default=960)
    p.add_argument('--free-motion-physics-hz',type=int,default=0,help='Optional lower rate only when fingers are over 15 mm above table and contact-free')
    p.add_argument('--host',default='127.0.0.1')
    p.add_argument('--port',type=int,default=49010)
    p.add_argument('--resume',action='store_true')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--view-camera',default='/World/global_camera',help='GUI preview camera; does not change recorded cameras')
    p.add_argument('--api-port',type=int,default=0,help='loopback sensor/control HTTP API; 0 disables')
    p.add_argument('--camera-preview-port',type=int,default=0,help='loopback live camera preview; 0 disables')
    p.add_argument('--keep-open',action='store_true',help='烟测保存后保留可交互仿真窗口')
    p.add_argument('--smoke-frames',type=int,default=0,help='固定 home 目标的仿真烟测，非真人示教')
    return p.parse_args()


def camera_sample(camera):
    from micromani_collect_isaac import as_uint8_rgb
    import math
    frame=camera.get_current_frame()
    stamp=float(frame['rendering_time'])
    if not math.isfinite(stamp): raise ValueError('invalid camera timestamp')
    rgb=as_uint8_rgb(frame['rgb'])
    if rgb.shape!=(480,640,3): raise ValueError('bad camera image shape')
    return rgb,stamp


def synchronize_cameras(world, cameras, max_renders=12):
    """Wait for this physics step's images without advancing physics or retiming frames."""
    target=float(world.current_time)
    for attempt in range(max_renders+1):
        stamps={name:camera_sample(camera)[1] for name,camera in cameras.items()}
        if all(abs(stamp-target)<1e-6 for stamp in stamps.values()):
            return
        if attempt<max_renders:
            world.render()
    raise ValueError(f'camera synchronization timeout: physics={target}, cameras={stamps}')


def main():
    args=parse_args()
    calibration = json.loads(Path(args.calibration).read_text())
    if args.resume:
        saved = json.loads((Path(args.session) / 'session.json').read_text())
        contract = SimContract(calibration, version=saved['dataContract']['version'])
    else:
        contract = SimContract(calibration)
    if args.fps<=0 or args.physics_hz<args.fps or args.physics_hz%args.fps or args.smoke_frames<0: raise ValueError('invalid frame rate/count')
    if args.free_motion_physics_hz and (args.free_motion_physics_hz<args.fps or args.free_motion_physics_hz%args.fps or args.free_motion_physics_hz>args.physics_hz): raise ValueError('invalid free-motion physics rate')
    stage=Path(args.stage).resolve()
    stage_identity={'path':str(stage),'sha256':hashlib.sha256(stage.read_bytes()).hexdigest()}
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.headless,'renderer':'RaytracedLighting','fast_shutdown':True})
    session=None; sock=None; preview=None; api=None
    try:
        import numpy as np
        import omni.usd
        import carb
        for setting in ("/exts/isaacsim.core.throttling/enable_async", "/app/asyncRendering", "/app/asyncRenderingLowLatency", "/omni/replicator/asyncRendering"):
            carb.settings.get_settings().set(setting, False)
        from PIL import Image
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.sensors.camera import Camera
        from micromani_lerobot_schema import ARTICULATION_ROOT,CAMERA_SPECS,DEFAULT_HOME_ACTION
        from micromani_collect_isaac import get_dof_names,get_control_state,read_force_torque,apply_control_action,as_uint8_rgb
        if omni.usd.get_context().open_stage(str(stage)) is False: raise RuntimeError('cannot open stage')
        for _ in range(60): app.update()
        if not args.headless:
            from omni.kit.viewport.utility import get_active_viewport
            viewport=get_active_viewport()
            if viewport: viewport.set_active_camera(args.view_camera)
        # 每个数据帧一个明确物理步和渲染步，磁盘写入只影响墙钟速度。
        world=World(stage_units_in_meters=1.,physics_dt=1/args.physics_hz,rendering_dt=1/args.fps)
        from micromani_grip_sensing import create_finger_sensors,read_grip_sensors
        grip_sensors=create_finger_sensors(dt=1/args.fps)
        world.reset()
        for sensor in grip_sensors.values(): sensor.initialize()
        robot=Articulation(ARTICULATION_ROOT); robot.initialize()
        names=get_dof_names(robot); indices={n:i for i,n in enumerate(names)}
        cameras={}
        for name,spec in CAMERA_SPECS.items():
            camera=Camera(prim_path=spec['prim_path'],resolution=tuple(spec['resolution']),frequency=args.fps)
            camera.initialize(); camera.add_rgb_to_frame(); cameras[name]=camera
        action=validate_action(DEFAULT_HOME_ACTION)
        if args.initial_state_json:
            from micromani_lerobot_schema import CONTROL_ORDER
            initial=json.loads(Path(args.initial_state_json).read_text())['state']
            action=validate_action([initial[k] for k in CONTROL_ORDER])
        from micromani_motion import JointMotion,drive_targets
        # One initialization reset; every subsequent movement uses physical drives.
        apply_control_action(robot,indices,action)
        motion=JointMotion(action)
        current_physics_hz=args.physics_hz
        if args.free_motion_physics_hz:
            from micromani_robot_calibration import frames as robot_frames, meshes as robot_meshes
            from micromani_step_rate import select_physics_hz
        advance_cost={'controller_s':0.,'physics_s':0.,'render_s':0.,'frames':0}
        def advance():
            nonlocal current_physics_hz
            if args.free_motion_physics_hz and advance_cost['frames']>=30:
                measured=get_control_state(robot,names);rf=robot_frames(measured)
                low=min(float((pts@rf[name][:3,:3].T+rf[name][:3,3])[:,2].min()) for name,pts in robot_meshes.items() if name.startswith('link_finger'))
                contacts=[v for g in read_grip_sensors(grip_sensors).values() for v in g['finger_contact_n']]
                closing=any(abs(motion.target[i]-measured[i])>1e-5 for i in (6,13))
                rate=select_physics_hz(low,contacts,closing,args.physics_hz,args.free_motion_physics_hz)
                if rate!=current_physics_hz:
                    world.set_simulation_dt(physics_dt=1/rate,rendering_dt=1/args.fps)
                    current_physics_hz=rate
                    print('PHYSICS_RATE '+str(rate),flush=True)
                    if api:api.schema['current_physics_hz']=rate
            for _ in range(current_physics_hz//args.fps):
                tick=time.perf_counter()
                reference=motion.step(1/current_physics_hz,get_control_state(robot,names))
                drive_targets(robot,indices,reference)
                advance_cost['controller_s']+=time.perf_counter()-tick
                tick=time.perf_counter();world.step(render=False)
                advance_cost['physics_s']+=time.perf_counter()-tick
            tick=time.perf_counter();world.render()
            advance_cost['render_s']+=time.perf_counter()-tick
            advance_cost['frames']+=1
            if args.profile_advance and advance_cost['frames']%30==0:
                print('ADVANCE_PROFILE '+json.dumps(advance_cost),flush=True)
        for _ in range(60): advance()
        if not robot.is_physics_handle_valid(): raise RuntimeError('invalid articulation physics handle')
        # 先预检相机，不通过补帧或重试步进来伪造同帧同步。
        def capture():
            synchronize_cameras(world,cameras)
            images={}; timestamps={}; frame_ids={}
            for name,camera in cameras.items():
                rgb,stamp=camera_sample(camera)
                frame_ids[name]=stamp
                buf=io.BytesIO(); Image.fromarray(rgb).save(buf,format='PNG')
                images[name]=buf.getvalue(); timestamps[name]=stamp
            return images,timestamps,frame_ids
        _,camera_times,last_ids=capture()
        if args.camera_preview_port or args.api_port:
            from micromani_camera_preview import CameraPreview
            preview=CameraPreview(args.camera_preview_port)
            preview.publish(cameras,world.current_time)
            print(f'Live camera preview: http://127.0.0.1:{preview.server.server_port}/frame',flush=True)
        # 使用原始仿真时间；不通过偏移修饰过时的相机帧。
        session=EpisodeSession(args.session,contract,fps=args.fps,task=args.task,
                               stage=stage_identity,resume=args.resume)
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind((args.host,args.port)); sock.setblocking(False)
        if args.api_port:
            from micromani_api import SimulationAPI
            api=SimulationAPI(args.api_port, fps=args.fps, preview=preview)
            api.schema['actuator_configuration']=omni.usd.get_context().get_stage().GetRootLayer().customLayerData.get('assemblyPhysics',{})
            api.schema.update(control_mode="bounded_quintic_joint_trajectory_physical_drive",physics_hz=args.physics_hz,command_semantics="applied means trajectory accepted; consult state.motion for physical completion",command_order="one HTTP command per sensor frame; busy motions reject new commands")
            api.schema['stop_semantics'] = 'hold_measured_arm_positions_preserve_applied_gripper_references_v1'
            print(f'Sensor/control API: http://127.0.0.1:{args.api_port}/v1/schema',flush=True)
        active_command_id=None
        def publish_sensors():
            if api:
                ft,_=read_force_torque(robot,indices,'HKVL-36A')
                api.publish(sim_time=world.current_time,state=get_control_state(robot,names),
                            action=action,force_torque=ft,grip=read_grip_sensors(grip_sensors),motion={**motion.snapshot(get_control_state(robot,names)),"command_id":active_command_id})
        def poll_control(action,action_time,source):
            nonlocal active_command_id
            sim_time=float(world.current_time)
            if api and api.consume_stop():
                motion.stop(get_control_state(robot,names),preserve_grippers=True)
                # Drop stale datagrams too; stop takes precedence over both transports.
                for _ in range(256):
                    try: sock.recvfrom(65535)
                    except BlockingIOError: break
                return list(motion.target),sim_time,'stop_hold'
            for _ in range(256):
                try: raw,addr=sock.recvfrom(65535)
                except BlockingIOError: break
                try:
                    packet=json.loads(raw)
                    candidate=validate_action(packet['axes'])
                    motion.submit(candidate)
                    active_command_id=None
                    action=candidate; action_time=sim_time
                    source=f"udp:{addr[0]}:{str(packet.get('source','external_control'))[:80]}"
                except (ValueError,KeyError,TypeError,RuntimeError) as exc: print('Packet rejected:',exc,flush=True)
            if api:
                action,command_id=api.apply_next(action,motion.submit,sim_time)
                if command_id: active_command_id=command_id; action_time=sim_time; source='http:'+command_id
            return action,action_time,source
        publish_sensors()
        commands=queue.Queue()
        def stdin_commands():
            for line in sys.stdin: commands.put(line.strip())
            commands.put('eof')
        if not args.smoke_frames:
            threading.Thread(target=stdin_commands,daemon=True).start()
        else: session.start()
        action_time=float(world.current_time)
        source='synthetic_smoke' if args.smoke_frames else 'initial_home_hold'
        print('Commands: start | save success/failure/unlabeled | discard | reset | finish',flush=True)
        running=True
        while app.is_running() and running:
            tick=time.monotonic()
            while not commands.empty():
                command=commands.get_nowait().split()
                if not command: continue
                try:
                    if command[0]=='start': session.start()
                    elif command[0]=='save': session.save(command[1] if len(command)>1 else 'unlabeled')
                    elif command[0]=='discard': session.discard()
                    elif command[0]=='reset':
                        if session.active: raise RuntimeError('save/discard before reset')
                        motion.submit(list(DEFAULT_HOME_ACTION))
                        action=list(DEFAULT_HOME_ACTION); action_time=float(world.current_time); source='manual_reset'
                    elif command[0]=='finish': session.finish(); running=False
                    elif command[0]=='eof':
                        session.interrupt('terminal closed'); running=False
                    else: raise ValueError('unknown command')
                    print('episode:',session.active,'closed:',session.closed,flush=True)
                except (ValueError,RuntimeError) as exc: print('Command rejected:',exc,flush=True)
            if not running: break
            sim_time=float(world.current_time)
            if not world.is_playing():
                app.update(); time.sleep(.01); continue
            if session.active and session.active['status']=='recording':
                images,timestamps,ids=capture()
                if any(ids[n]<=last_ids[n] for n in ids):
                    # 首帧允许使用预检的当前图像，之后必须是新渲染帧。
                    if session.frame_count: raise ValueError('camera frame did not advance')
                last_ids=ids
                _,force=read_force_torque(robot,indices,'HKVL-36A')
                grip = read_grip_sensors(grip_sensors)
                state_before_action=get_control_state(robot,names)
                action,action_time,source=poll_control(action,action_time,source)
                session.append(sim_time=sim_time,state=state_before_action,action=action,force=force,
                    images=images,source_times={'state':sim_time,'force':sim_time,
                        **timestamps},
                    action_time=action_time,action_source=source,
                    finger_contact_n=[value for side in ('left', 'right') for value in grip[side]['finger_contact_n']])
            else:
                action,action_time,source=poll_control(action,action_time,source)
            advance()
            if preview: preview.publish(cameras,world.current_time)
            publish_sensors()
            if args.smoke_frames and session.frame_count>=args.smoke_frames:
                session.save('unlabeled'); session.finish()
                print(f'Saved {session.frame_count} synchronized frames to {args.session}',flush=True)
                if args.keep_open and not args.headless:
                    print('Verification complete. Close the Isaac Sim window to exit.',flush=True)
                    while app.is_running():
                        preview_tick=time.monotonic()
                        if not world.is_playing():
                            app.update(); time.sleep(.01); continue
                        action,action_time,source=poll_control(action,action_time,source)
                        advance()
                        publish_sensors()
                        if preview: preview.publish(cameras,world.current_time)
                        time.sleep(max(0,1/args.fps-(time.monotonic()-preview_tick)))
                break
            if not args.smoke_frames: time.sleep(max(0,1/args.fps-(time.monotonic()-tick)))
        if args.smoke_frames and not session.closed:
            raise RuntimeError(f'smoke recording incomplete: {session.frame_count}/{args.smoke_frames} frames')
    except BaseException as exc:
        traceback.print_exc()
        if session: session.interrupt(exc)
        app._app.post_quit(1)
        raise
    finally:
        if session and session.active and session.active['status']=='recording': session.interrupt('application closed')
        if api: api.close()
        if preview: preview.close()
        if sock: sock.close()
        app.close()


if __name__=='__main__': main()
