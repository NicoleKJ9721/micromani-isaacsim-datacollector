"""Loopback simulation API. HTTP threads never access Isaac objects."""
import copy
import json
import math
import threading
import time
import uuid
from collections import OrderedDict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from micromani_lerobot_schema import CONTROL_ORDER, JOINT_LIMITS
from micromani_sim_contract import validate_action


def validate_command(body):
    if not isinstance(body, dict) or set(body) - {'axes', 'joints', 'ttl_s'}:
        raise ValueError('expected axes or joints, optional ttl_s; unknown fields rejected')
    if ('axes' in body) == ('joints' in body):
        raise ValueError('provide exactly one of axes or joints')
    ttl = float(body.get('ttl_s', 2.))
    if not math.isfinite(ttl) or not .05 <= ttl <= 10:
        raise ValueError('ttl_s must be within [0.05, 10] wall-clock seconds')
    if 'axes' in body:
        targets = dict(zip(CONTROL_ORDER, validate_action(body['axes'])))
    else:
        targets = body['joints']
        if not isinstance(targets, dict) or not targets or set(targets) - set(CONTROL_ORDER):
            raise ValueError('joints must contain known logical control names')
        targets = {k: float(v) for k, v in targets.items()}
        for k, v in targets.items():
            lo, hi = JOINT_LIMITS[k]
            if not math.isfinite(v) or not lo <= v <= hi:
                raise ValueError(f'{k} outside [{lo}, {hi}]')
    return targets, ttl


class SimulationAPI:
    def __init__(self, port=49012, fps=30, preview=None):
        self.lock = threading.Lock()
        self.pending = deque()
        self.commands = OrderedDict()
        self.snapshot = None
        self.preview = preview
        self.sequence = 0
        self.stop_requested = False
        self.schema = {
            'version': 'micromani.sim.api.v1', 'source': 'isaac_sim',
            'side_convention': 'model_left_then_model_right; operator sides are reversed',
            'control_order': list(CONTROL_ORDER), 'joint_limits': JOINT_LIMITS,
            'units': {'joint1_2_3': 'm', 'joint4_5_6': 'rad', 'gripper': 'm per finger (total gap approximately 2q)',
                      'force': 'N', 'torque': 'N*m', 'timestamps': 'simulation seconds'},
            'gripper_control': {'names': ['left_gripper','right_gripper'], 'coupling': 'two fingers share one target per arm; individual finger commands rejected', 'client_opening_unit': 'nominal total gap mm'},
            'control_mode': 'absolute_joint_position_set_and_drive_target; no velocity/force/trajectory control',
            'command_semantics': 'queued until simulation thread applies; applied means setter succeeded, not physical target convergence',
            'command_order': 'one HTTP command per physics step; HTTP wins over UDP on that step',
            'stop_behavior': 'hold measured arm positions and retain current gripper drive references; cancel queued commands; no continued closing toward future targets',
            'sample_rate_hz': fps, 'wall_rate_guaranteed': False,
            'force_source': 'untared simulated joint6 child-frame reaction; raw and HKVL-36A quantized values',
            'grip_source': 'four simulated finger contact magnitudes; pair mean is not calibrated actuator force',
            'camera_names': ['left_wrist', 'right_wrist', 'global'], 'camera_encoding': 'base64 JPEG RGB',
            'camera_timestamps': 'individual rendering_time; compare with state simulation_time',
            'endpoints': ['/v1/schema', '/v1/state', '/v1/cameras', '/v1/commands', '/v1/commands/{id}', '/v1/stop'],
            'limits': {'pending_commands': 32, 'retained_results': 256, 'body_bytes': 16384},
        }
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup(); self.connection.settimeout(3)
            def reply(self, status, body):
                data = json.dumps(body, allow_nan=False, separators=(',', ':')).encode()
                self.send_response(status); self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                try: self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError): pass
            def do_GET(self):
                if self.path == '/v1/schema': self.reply(200, owner.schema)
                elif self.path == '/v1/state':
                    with owner.lock: snapshot = copy.deepcopy(owner.snapshot)
                    if snapshot is None: self.reply(503, {'error': 'waiting for sensor sample'}); return
                    snapshot['age_wall_s'] = max(0., time.time() - snapshot['published_at'])
                    snapshot['stale'] = snapshot['age_wall_s'] > 1.
                    self.reply(200, snapshot)
                elif self.path == '/v1/cameras':
                    payload = owner.preview.payload if owner.preview else None
                    if not payload: self.reply(503, {'error': 'waiting for cameras'}); return
                    frame = json.loads(payload);frame['age_wall_s'] = max(0., time.time()-frame['published_at'])
                    frame['stale'] = frame['age_wall_s'] > 1.;self.reply(200, frame)
                elif self.path.startswith('/v1/commands/'):
                    with owner.lock: result = copy.deepcopy(owner.commands.get(self.path.rsplit('/', 1)[-1]))
                    self.reply(200 if result else 404, result or {'error': 'unknown or evicted command'})
                else: self.reply(404, {'error': 'unknown endpoint'})
            def do_POST(self):
                if self.path == '/v1/stop':
                    with owner.lock:
                        owner.stop_requested = True
                        while owner.pending:
                            cid, _, _ = owner.pending.popleft()
                            owner.commands[cid].update(status='cancelled',error='stop requested')
                    self.reply(202, {'status':'stop_requested'}); return
                if self.path != '/v1/commands': self.reply(404, {'error': 'unknown endpoint'}); return
                try:
                    n = int(self.headers.get('Content-Length', '0'))
                    if not 0 < n <= 16384: raise ValueError('invalid body size')
                    body = json.loads(self.rfile.read(n))
                    result = owner.submit(body)
                    self.reply(202, result)
                except OverflowError as exc: self.reply(429, {'error': str(exc)})
                except (ValueError, TypeError, KeyError, TimeoutError) as exc: self.reply(400, {'error': str(exc)})
            def log_message(self, *args): pass
        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def submit(self, body):
        targets, ttl = validate_command(body)
        with self.lock:
            if self.stop_requested: raise OverflowError('stop awaiting simulation acknowledgement')
            if len(self.pending) >= 32: raise OverflowError('command queue full')
            command_id = uuid.uuid4().hex
            result = {'id': command_id, 'status': 'accepted', 'targets': targets, 'accepted_at': time.time()}
            self.commands[command_id] = result
            self.pending.append((command_id, targets, time.monotonic()+ttl))
            while len(self.commands) > 256: self.commands.popitem(last=False)
            return copy.deepcopy(result)

    def consume_stop(self):
        with self.lock:
            requested=self.stop_requested; self.stop_requested=False
            return requested

    def apply_next(self, current, apply, sim_time):
        """Only call on simulation thread, immediately before the next world step."""
        with self.lock:
            if not self.pending: return current, None
            command_id, targets, deadline = self.pending.popleft()
        result = {}
        action = list(current)
        if time.monotonic() > deadline:
            result = {'status': 'expired', 'error': 'command TTL elapsed before apply'}
        else:
            try:
                candidate = validate_action([targets.get(k, v) for k, v in zip(CONTROL_ORDER, current)])
                apply(candidate)
                action = candidate
                result = {'status': 'applied', 'applied_sim_time': float(sim_time), 'action': action}
            except Exception as exc:
                result = {'status': 'failed', 'error': str(exc)}
        with self.lock: self.commands[command_id].update(result)
        return action, command_id if result['status'] == 'applied' else None

    def publish(self, *, sim_time, state, action, force_torque, grip, motion=None):
        snapshot = {'simulation_time': float(sim_time), 'published_at': time.time(),
                    'state': dict(zip(CONTROL_ORDER, map(float, state))),
                    'target': dict(zip(CONTROL_ORDER, map(float, action))),
                    'force_torque': force_torque, 'grip': grip,
                    'gripper_opening_mm': {side:float(state[i])*2000 for side,i in [('left',6),('right',13)]}}
        if motion is not None: snapshot['motion'] = motion
        # Fail explicitly on invalid sensor values instead of publishing non-standard JSON.
        json.dumps(snapshot, allow_nan=False)
        with self.lock:
            self.sequence += 1; snapshot['sequence'] = self.sequence; self.snapshot = snapshot

    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=2)
