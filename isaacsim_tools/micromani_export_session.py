"""将已保存的仿真 episodes 事务式导出为 LeRobot；保留原始数据。"""
from __future__ import annotations
import json
import math
import shutil
import uuid
from pathlib import Path
import numpy as np
from PIL import Image
from micromani_sim_contract import SimContract, CAMERA_KEYS, finite_vector, validate_action


def read_episode(root, episode, metadata, contract):
    contract.validate_metadata(episode['dataContract'])
    folder=root/episode['directory']
    if not folder.resolve().is_relative_to(root.resolve()): raise ValueError('episode path escapes session')
    rows=[json.loads(line) for line in (folder/'frames.jsonl').read_text().splitlines() if line.strip()]
    if not rows or len(rows)!=episode['frames']: raise ValueError('episode frame count mismatch')
    last=None
    for i,row in enumerate(rows):
        if row['frame_index']!=i or not math.isfinite(row['timestamp']) or abs(row['timestamp']-i/metadata['fps'])>1e-6:
            raise ValueError('invalid frame index/timestamp')
        t=row['simulation_time']
        if not math.isfinite(t) or (last is not None and abs(t-last-1/metadata['fps'])>1e-6):
            raise ValueError('simulation clock gap')
        last=t
        times=row['source_times']
        if set(times)!={'state','force',*CAMERA_KEYS} or any(not math.isfinite(v) or abs(v-t)>1e-6 for v in times.values()):
            raise ValueError('unaligned sources')
        if not math.isfinite(row['action_time']) or row['action_time']>t+1e-8 or not row['action_source']:
            raise ValueError('invalid action provenance')
        finite_vector(row['observation_state'],14); validate_action(row['action'])
        finite_vector(row['observation_force_torque'],12)
        contract.to_dataset(row['observation_state']); contract.to_dataset(row['action'])
        for camera in CAMERA_KEYS:
            path=folder/row['images'][camera]
            if not path.resolve().is_relative_to(folder.resolve()): raise ValueError('image path escapes episode')
            with Image.open(path) as img:
                img.load()
                if img.size!=(640,480) or img.mode!='RGB': raise ValueError('expected 640x480 RGB image')
    return folder,rows


def export_session(root, output, repo_id, *, image_dtype='image', overwrite=False):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root=Path(root).resolve(); output=Path(output).resolve()
    if output==root or output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError('raw session and output must be separate')
    if output.exists() and not overwrite: raise FileExistsError(output)
    metadata=json.loads((root/'session.json').read_text())
    if metadata.get('schema_version')!='micromani_raw_v2': raise ValueError('unsupported session')
    contract=SimContract.from_metadata(metadata['dataContract'])
    episodes=[e for e in metadata['episodes'] if e['status']=='saved']
    if not episodes: raise ValueError('no saved episodes')
    if any(e['status'] in ('recording','interrupted') for e in metadata['episodes']):
        raise ValueError('save or discard unresolved episodes first')
    prepared=[read_episode(root,e,metadata,contract) for e in episodes]
    availability=['observation_grip_force' in r for _,rows in prepared for r in rows]
    if any(availability) and not all(availability): raise ValueError('mixed gripping force availability')
    include_grip=all(availability)
    if include_grip:
        for _,rows in prepared:
            for r in rows:
                if min(finite_vector(r['observation_grip_force'],2))<0: raise ValueError('invalid gripping force')
    output.parent.mkdir(parents=True,exist_ok=True)
    staging=output.with_name(output.name+'.staging-'+uuid.uuid4().hex)
    dataset=None
    try:
        dataset=LeRobotDataset.create(repo_id=repo_id,fps=metadata['fps'],root=staging,
            robot_type='micromani_isaac',features=contract.features(image_dtype,include_grip=include_grip),use_videos=image_dtype=='video',
            image_writer_processes=0,image_writer_threads=3,metadata_buffer_size=1)
        for folder,rows in prepared:
            for row in rows:
                left,right=contract.force_to_dataset(row['observation_force_torque'])
                frame={'observation.state':np.asarray(contract.to_dataset(row['observation_state']),dtype=np.float32),
                       'action':np.asarray(contract.to_dataset(row['action']),dtype=np.float32),
                       'observation.force_left':np.asarray(left,dtype=np.float32),
                       'observation.force_right':np.asarray(right,dtype=np.float32),'task':metadata['task']}
                for camera,key in CAMERA_KEYS.items():
                    with Image.open(folder/row['images'][camera]) as img: frame[key]=np.array(img)
                if include_grip:
                    g=dict(zip(('left','right'),row['observation_grip_force']))
                    frame['observation.grip_force']=np.asarray([g[side] for side in contract.sides],dtype=np.float32)
                dataset.add_frame(frame)
            dataset.save_episode(parallel_encoding=False)
        dataset.finalize(); dataset=None
        provenance={'dataContract':contract.metadata(),'rawSession':str(root),'episodes':[
            {**e,'lerobotEpisodeIndex':i} for i,e in enumerate(episodes)]}
        (staging/'meta/micromani_info.json').write_text(json.dumps(provenance,indent=2,ensure_ascii=False))
        info_path=staging/'meta/info.json'; info=json.loads(info_path.read_text())
        info['dataContract']=contract.metadata(); info_path.write_text(json.dumps(info,indent=2))
        # 逐帧来源与时间偏差独立存储，保留模型 SI 原始力，不伪造硬件校准。
        sidecar=staging/'provenance'; sidecar.mkdir()
        for i,(_,rows) in enumerate(prepared):
            with (sidecar/f'episode_{i:06d}.jsonl').open('w') as fp:
                for row in rows:
                    fp.write(json.dumps({k:v for k,v in row.items() if k!='images'})+'\n')
        loaded=LeRobotDataset(repo_id,root=staging,return_uint8=True)
        expected=sum(len(rows) for _,rows in prepared)
        if len(loaded)!=expected: raise RuntimeError('LeRobot reload frame count mismatch')
        offset=0
        for _,rows in prepared:
            for row in rows:
                sample=loaded[offset]; offset+=1
                np.testing.assert_allclose(np.asarray(sample['action']),contract.to_dataset(row['action']),rtol=1e-6,atol=1e-5)
                for key in CAMERA_KEYS.values():
                    if tuple(sample[key].shape)!=(3,480,640): raise RuntimeError('reloaded image shape mismatch')
        report={'frames':expected,'episodes':len(episodes),'reload_verified':True,'source':'isaac_sim'}
        (staging/'micromani_validation.json').write_text(json.dumps(report,indent=2))
        if output.exists():
            backup=output.with_name(output.name+'.backup-'+uuid.uuid4().hex)
            output.rename(backup)
            try: staging.rename(output)
            except Exception:
                backup.rename(output); raise
            report['previous_output_backup']=str(backup)
        else: staging.rename(output)
        return report
    except Exception:
        if dataset is not None:
            try: dataset.finalize()
            except Exception: pass
        if staging.exists(): shutil.rmtree(staging)
        raise
