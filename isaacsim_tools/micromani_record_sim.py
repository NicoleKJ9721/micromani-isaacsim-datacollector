"""Isaac Sim 独立录制入口；UDP JSON 控制与 episode 操作共享一个队列。"""
from __future__ import annotations
import argparse
import json
import socket
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--stage', required=True)
    p.add_argument('--session', required=True)
    p.add_argument('--operator-left-model-side', choices=['left','right'], required=True)
    p.add_argument('--zero-gap-mm', type=float, nargs=2, required=True, metavar=('LEFT','RIGHT'))
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=49010)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--headless', action='store_true')
    a = p.parse_args()
    if a.fps <= 0:
        p.error('fps must be positive')
    from isaacsim import SimulationApp
    kit = SimulationApp({'headless': a.headless, 'renderer': 'RaytracedLighting'})
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.sensors.camera import Camera
    from micromani_collect_isaac import (get_dof_names, get_control_state, read_force_torque,
                                         apply_control_action, as_uint8_rgb, write_png)
    from micromani_lerobot_schema import ARTICULATION_ROOT, CAMERA_SPECS, DEFAULT_HOME_ACTION
    from micromani_grip_sensing import create_finger_sensors, read_grip_sensors
    from micromani_sim_recording import SimSession, profile_metadata, validate_action
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((a.host, a.port))
        sock.setblocking(False)
        if omni.usd.get_context().open_stage(str(Path(a.stage).resolve())) is False:
            raise RuntimeError('cannot open scene')
        for _ in range(90):
            kit.update()
        world = World(stage_units_in_meters=1., physics_dt=1/a.fps, rendering_dt=1/a.fps)
        grip_sensors=create_finger_sensors(dt=1/a.fps)
        world.reset()
        for sensor in grip_sensors.values(): sensor.initialize()
        robot = Articulation(ARTICULATION_ROOT)
        robot.initialize()
        index = {name:i for i,name in enumerate(get_dof_names(robot))}
        cameras = {}
        for name,spec in CAMERA_SPECS.items():
            cameras[name] = Camera(prim_path=spec['prim_path'], resolution=tuple(spec['resolution']))
            cameras[name].initialize()
        home = list(DEFAULT_HOME_ACTION)
        for _ in range(60):
            apply_control_action(robot,index,home)
            world.step(render=True)
        if not robot.is_physics_handle_valid():
            raise RuntimeError('invalid articulation physics handle')
        session = SimSession(a.session, profile_metadata(a.operator_left_model_side,a.zero_gap_mm),a.fps)
        action, action_time = None, None
        sim_time = 0.
        take = 0
        print(f'READY UDP {a.host}:{a.port}', flush=True)
        while kit.is_running() and session.phase != 'finished':
            started = time.monotonic()
            for _ in range(64):
                try:
                    data,peer = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                try:
                    packet = json.loads(data)
                    op = packet.get('op')
                    if op == 'start':
                        session.start(packet['task'])
                    elif op == 'save':
                        session.save(packet.get('outcome','unreviewed'))
                    elif op == 'discard':
                        session.discard()
                    elif op == 'reset':
                        if session.phase != 'reset_pending':
                            raise ValueError('save/discard before reset')
                        world.reset()
                        robot.initialize()
                        for _ in range(60):
                            apply_control_action(robot,index,home)
                            world.step(render=True)
                        for sensor in grip_sensors.values(): sensor.initialize()
                        session.reset()
                        take += 1
                        action,action_time = None,None
                    elif op == 'finish':
                        session.finish()
                    elif 'axes' in packet:
                        action = validate_action(packet['axes'])
                        action_time = sim_time
                    else:
                        raise ValueError('unknown op; expected op or axes')
                    reply = {'ok':True,'phase':session.phase}
                except Exception as exc:
                    reply = {'ok':False,'error':str(exc),'phase':session.phase}
                sock.sendto(json.dumps(reply).encode(),peer)
            if session.phase == 'recording':
                try:
                    if action is None:
                        raise ValueError('send a valid axes packet before start')
                    images = {}
                    for name,camera in cameras.items():
                        rgb = as_uint8_rgb(camera.get_rgba())
                        if rgb.shape != (480,640,3):
                            raise ValueError(f'invalid camera {name}: {rgb.shape}')
                        relative = f'images/take_{take:06d}/{name}/{len(session.frames):06d}.png'
                        write_png(session.root/relative,rgb)
                        images[name] = relative
                    _,force = read_force_torque(robot,index,'HKVL-36A')
                    grip=read_grip_sensors(grip_sensors)
                    session.add(get_control_state(robot,get_dof_names(robot)),action,force,images,sim_time,action_time,
                                grip_force=[grip[side]['grip_force_n'] for side in ('left','right')])
                except Exception as exc:
                    session.phase = 'interrupted'
                    session._write_manifest()
                    print(f'RECORDING_INTERRUPTED: {exc}',flush=True)
            if action is not None and session.phase == 'recording':
                apply_control_action(robot,index,action)
            world.step(render=True)
            sim_time += 1/a.fps
            time.sleep(max(0.,1/a.fps-(time.monotonic()-started)))
    finally:
        sock.close()
        kit.close()


if __name__ == '__main__':
    main()
