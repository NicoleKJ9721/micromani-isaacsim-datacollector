"""Standard-library-only client; importing does not connect or move anything."""
import json
import time
from urllib.request import Request, urlopen

class MicroManiClient:
    def __init__(self, base_url='http://127.0.0.1:49012', timeout=3.):
        self.base_url = base_url.rstrip('/'); self.timeout = timeout
    def _request(self, path, body=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = Request(self.base_url+path, data=data, headers={'Content-Type': 'application/json'})
        with urlopen(request, timeout=self.timeout) as response: result=json.load(response)
        if path=='/v1/commands' and body is not None:self._last_command_id=result['id']
        if path=='/v1/stop':self._last_command_id=None
        return result
    def stop(self): return self._request('/v1/stop', {})
    def wait_motion(self, timeout=60., command_id=None):
        """Wait on observed physical motion, not HTTP acceptance. One controller at a time."""
        command_id=command_id or getattr(self,'_last_command_id',None)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if command_id:
                status=self.command(command_id)['status']
                if status not in ('accepted','applied'):raise RuntimeError('command '+status)
            state=self.state()
            if state['stale']: raise RuntimeError('sensor stream stale')
            if command_id and state['motion'].get('command_id')!=command_id:
                time.sleep(.03);continue
            if state['motion']['status'] not in ('moving','settling'): return state
            time.sleep(.03)
        raise TimeoutError('motion unfinished; inspect state or stop')
    def schema(self): return self._request('/v1/schema')
    def state(self): return self._request('/v1/state')
    def cameras(self): return self._request('/v1/cameras')
    def force_torque(self):
        s=self.state(); return {k:s[k] for k in ('simulation_time','age_wall_s','stale','force_torque')}
    def grip(self):
        s=self.state(); return {k:s[k] for k in ('simulation_time','age_wall_s','stale','grip')}
    def set_joints(self, targets, ttl_s=2.):
        return self._request('/v1/commands', {'joints': targets, 'ttl_s': ttl_s})
    def set_action(self, axes, ttl_s=2.):
        return self._request('/v1/commands', {'axes': list(axes), 'ttl_s': ttl_s})
    def set_arm(self, side, joints_si, ttl_s=2.):
        """Six joint targets: X/Y/Z metres, RX/RZ/RY radians; not Cartesian pose."""
        if side not in ('left','right') or len(joints_si)!=6:
            raise ValueError('model side left/right and six joint values required')
        return self.set_joints({f'joint{i+1}_{side}':v for i,v in enumerate(joints_si)},ttl_s)
    def camera(self, name):
        if name not in ('left_wrist','right_wrist','global'): raise ValueError('unknown camera')
        frame=self.cameras()
        return {**frame['cameras'][name], 'age_wall_s':frame['age_wall_s'], 'stale':frame['stale']}
    def set_gripper(self, side, opening_mm, ttl_s=2.):
        """One symmetric gripper command; nominal total opening in mm (0..20)."""
        if side not in ('left','right'): raise ValueError('side must be model left or right')
        return self.set_joints({side+'_gripper':float(opening_mm)/2000.},ttl_s)
    def command(self, command_id): return self._request('/v1/commands/'+command_id)
    def wait(self, command_id, timeout=5.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            result=self.command(command_id)
            if result['status']!='accepted': return result
            time.sleep(.02)
        raise TimeoutError('command still pending; inspect status before retrying')
