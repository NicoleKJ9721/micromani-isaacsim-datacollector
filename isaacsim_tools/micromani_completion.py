"""Interpret public finite-motion completion without hiding raw server status.

Contact-limited completion is opt-in for named, force-validated grippers only.
It is not grasp/assembly success and never sends commands or changes drive refs.
"""
import math
from micromani_lerobot_schema import CONTROL_ORDER

class ContactCompletionGate:
    def __init__(self, command_id, target, contact_holds=None, stable_samples=5):
        if not command_id or stable_samples < 5:
            raise ValueError('command and at least five fresh samples required')
        self.command_id=command_id
        self.target={k:float(target[k]) for k in CONTROL_ORDER}
        if not all(math.isfinite(v) for v in self.target.values()):
            raise ValueError('nonfinite target')
        self.holds=dict(contact_holds or {})
        for side,cfg in self.holds.items():
            if side not in ('left','right') or not (0<cfg['minimum_n']<cfg['maximum_n']<=.05) or not (5e-6<=cfg['max_compression_m']<=50e-6):
                raise ValueError('invalid exploratory contact bounds')
        self.required=stable_samples;self.sequence=None;self.stamp=None
        self.samples=[];self.fault=None

    def observe(self,s):
        def fail(reason):
            self.fault=reason
            return {'complete':False,'fault':reason,'raw_status':s.get('motion',{}).get('status')}
        if self.fault:return fail(self.fault)
        try:
            if s['stale']:return fail('stale_sensor_state')
            seq=s['sequence'];stamp=float(s['simulation_time']);m=s['motion'];status=m['status']
            if not isinstance(seq,int) or isinstance(seq,bool) or not math.isfinite(stamp):return fail('invalid_clock')
            if self.sequence is not None and (seq<self.sequence or stamp<self.stamp):return fail('clock_regressed')
            if seq==self.sequence or stamp==self.stamp:return {'complete':False,'pending':'duplicate_sample','raw_status':status}
            self.sequence=seq;self.stamp=stamp
            if m.get('command_id')!=self.command_id:return fail('command_changed')
            if any(not math.isfinite(float(s['target'][k])) or abs(s['target'][k]-self.target[k])>1e-12 for k in CONTROL_ORDER):return fail('target_changed')
            for w in s['force_torque'].values():
                if w['saturated'] or len(w['wrench'])!=6 or not all(math.isfinite(v) for v in w['wrench']):return fail('invalid_wrist')
            if status in ('moving','settling'):
                self.samples=[]
                return {'complete':False,'pending':status,'raw_status':status}
            if status not in ('reached','contact_or_tracking_error'):return fail('unexpected_terminal_status')
            if not all(math.isfinite(float(m[k])) for k in ('elapsed_s','duration_s')) or m['duration_s']<0 or m['elapsed_s']+1e-9<m['duration_s']:return fail('trajectory_not_finished')
            if len(m['reference'])!=len(CONTROL_ORDER) or any(not math.isfinite(v) or abs(v-self.target[k])>1e-12 for k,v in zip(CONTROL_ORDER,m['reference'])):return fail('reference_not_finished')
            intentional_contact=False;measured={}
            for k in CONTROL_ORDER:
                value=float(s['state'][k]);measured[k]=value
                if not math.isfinite(value):return fail('nonfinite_encoder')
                error=value-self.target[k]
                if k.endswith('_gripper'):
                    side=k.split('_')[0];cfg=self.holds.get(side)
                    if cfg:
                        force=s['grip'][side]['finger_contact_n']
                        if len(force)!=2 or not all(math.isfinite(v) and cfg['minimum_n']<=v<=cfg['maximum_n'] for v in force):return fail('contact_force_out_of_bounds_'+side)
                        if not (-5e-6<=error<=cfg['max_compression_m']):return fail('contact_position_out_of_bounds_'+side)
                        intentional_contact |= error>5e-6
                    elif abs(error)>5e-6:return fail('unapproved_gripper_tracking_error')
                else:
                    tolerance=5e-6 if int(k[5])<=3 else 5e-4
                    if abs(error)>tolerance:return fail('arm_tracking_error_'+k)
            if status=='contact_or_tracking_error':
                if not intentional_contact or m['elapsed_s']+1e-9<m['duration_s']+2:return fail('unexplained_contact_or_tracking_error')
            self.samples.append((stamp,measured));self.samples=self.samples[-self.required:]
            stable=len(self.samples)>=self.required and self.samples[-1][0]-self.samples[0][0]>=(self.required-1)/30-1e-6
            for side in self.holds:
                vals=[row[1][side+'_gripper'] for row in self.samples]
                stable &= max(vals)-min(vals)<=1e-6
            return {'complete':bool(stable),'classification':'contact_limited_complete' if intentional_contact else 'position_complete','raw_status':status,'fresh_terminal_samples':len(self.samples),'not_grasp_success':True}
        except (KeyError,TypeError,ValueError,OverflowError):return fail('malformed_public_state')
