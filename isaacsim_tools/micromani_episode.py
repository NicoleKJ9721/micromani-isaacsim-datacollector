"""串行仿真 episode 写入：保存/丢弃/续录与不可变来源元数据。"""
from __future__ import annotations
import json
import math
from pathlib import Path
from micromani_sim_contract import finite_vector, validate_action, CAMERA_KEYS
from micromani_grip_sensing import gripping_force


class EpisodeSession:
    def __init__(self, root, contract, *, fps, task, stage, resume=False):
        self.root=Path(root); self.contract=contract
        if not isinstance(fps,int) or fps<=0 or not task.strip(): raise ValueError('fps/task invalid')
        self.fps=fps; self.active=None; self.frame_count=0; self.last_time=None
        self.closed=False
        metadata={'schema_version':'micromani_raw_v2','fps':fps,'task':task,'stage':stage,
                  'dataContract':contract.metadata()}
        path=self.root/'session.json'
        if resume:
            self.data=json.loads(path.read_text())
            if any(self.data.get(k)!=v for k,v in metadata.items()): raise ValueError('incompatible session; create a new dataset')
            drafts=[e for e in self.data['episodes'] if e['status'] in ('recording','interrupted')]
            if drafts:
                self.active=drafts[-1]; self.active['status']='interrupted'
        else:
            self.root.mkdir(parents=True,exist_ok=False)
            self.data={**metadata,'episodes':[]}
            self._persist()

    def _persist(self):
        tmp=self.root/'session.json.tmp'
        tmp.write_text(json.dumps(self.data,ensure_ascii=False,indent=2,allow_nan=False))
        tmp.replace(self.root/'session.json')

    def start(self):
        if self.closed or self.active is not None: raise RuntimeError('finish/discard current episode before starting')
        index=len(self.data['episodes'])
        self.active={'index':index,'directory':f'episode_{index:06d}','status':'recording','frames':0,
                     'outcome':'unlabeled','dataContract':self.contract.metadata()}
        (self.root/self.active['directory']).mkdir()
        self.data['episodes'].append(self.active)
        self.frame_count=0; self.last_time=None
        self._persist()

    def append(self, *, sim_time, state, action, force, images, source_times, action_time, action_source, grip_force=None, finger_contact_n=None):
        if self.active is None or self.active['status']!='recording': raise RuntimeError('episode is not recording')
        state=finite_vector(state,14); action=validate_action(action); force=finite_vector(force,12)
        if not math.isfinite(sim_time) or not math.isfinite(action_time) or action_time>sim_time+1e-8:
            raise ValueError('invalid or future action timestamp')
        if not action_source: raise ValueError('action source required')
        if self.last_time is not None and abs(sim_time-self.last_time-1/self.fps)>1e-6:
            raise ValueError('simulation time gap or rewind; discard interrupted episode')
        required={'state','force',*CAMERA_KEYS}
        if set(source_times)!=required or set(images)!=set(CAMERA_KEYS): raise ValueError('missing source')
        if any(not math.isfinite(t) or abs(t-sim_time)>1e-6 for t in source_times.values()):
            raise ValueError('sources must belong to the same simulation step')
        self.contract.to_dataset(state); self.contract.to_dataset(action)
        contact_fields = {}
        if finger_contact_n is not None:
            fingers = finite_vector(finger_contact_n, 4)
            readings = [gripping_force(*pair) for pair in (fingers[:2], fingers[2:])]
            grip_force = [reading['grip_force_n'] for reading in readings]
            contact_fields = {'finger_contact_n': fingers,
                              'grip_bilateral_contact': [reading['bilateral_contact'] for reading in readings]}
        if grip_force is not None:
            grip = finite_vector(grip_force, 2)
            if min(grip) < 0:
                raise ValueError('negative gripping force')
            contact_fields['observation_grip_force'] = grip
        directory=self.root/self.active['directory']; paths={}
        for name,content in images.items():
            if not isinstance(content,bytes) or not content: raise ValueError('empty image')
        for name,content in images.items():
            rel=Path('images')/name/f'{self.frame_count:06d}.png'
            path=directory/rel; path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(content)
            paths[name]=str(rel)
        row={'frame_index':self.frame_count,'timestamp':self.frame_count/self.fps,
             'simulation_time':sim_time,'source_times':source_times,'action_time':action_time,
             'action_source':action_source,'action_held':action_time<sim_time-1e-8,
             'observation_state':state,'action':action,'observation_force_torque':force,'images':paths,
             **contact_fields}
        with (directory/'frames.jsonl').open('a') as fp:
            fp.write(json.dumps(row,allow_nan=False)+'\n')
        self.frame_count+=1; self.last_time=sim_time
        self.active['frames']=self.frame_count

    def save(self,outcome='unlabeled'):
        if self.active is None or self.active['status']!='recording': raise RuntimeError('no recordable episode')
        if not self.frame_count: raise ValueError('cannot save empty episode')
        if outcome not in ('success','failure','unlabeled'): raise ValueError('invalid outcome')
        self.active.update(status='saved',outcome=outcome)
        self._persist(); self.active=None

    def interrupt(self,reason):
        if self.active is not None:
            self.active.update(status='interrupted',error=str(reason)); self._persist()

    def discard(self):
        if self.active is None: raise RuntimeError('no episode to discard')
        self.active['status']='discarded'
        self._persist(); self.active=None

    def finish(self):
        if self.active is not None: raise RuntimeError('save or discard before finishing')
        self._persist(); self.closed=True
