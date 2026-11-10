"""将完成的仿真会话导出为 LeRobot；导出前完整检查，不覆盖已有数据。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from micromani_sim_recording import CAMERAS, VERSION, from_dataset, validate_contract, vector


def load_session(root):
    root = Path(root).resolve()
    manifest = json.loads((root/'manifest.json').read_text())
    validate_contract(manifest.get('dataContract'))
    if manifest.get('schema_version') != VERSION or manifest.get('phase') != 'finished':
        raise ValueError('only finished simulation sessions can be exported')
    fps = manifest['fps']
    if not isinstance(fps, int) or fps <= 0 or not manifest['episodes']:
        raise ValueError('invalid fps or empty session')
    episodes = []
    def local_file(relative):
        p = (root/relative).resolve()
        if not p.is_relative_to(root) or not p.is_file():
            raise ValueError('missing file or path outside session')
        return p
    from PIL import Image
    for i, episode in enumerate(manifest['episodes']):
        rows = [json.loads(line) for line in local_file(episode['path']).read_text().splitlines()]
        if episode['index'] != i or len(rows) != episode['frames'] or not rows:
            raise ValueError('episode count/index mismatch')
        for j, row in enumerate(rows):
            if row['frame_index'] != j or abs(row['timestamp']-j/fps) > 1e-6:
                raise ValueError('non-contiguous frame index/time')
            vector(row['observation.state'], 14)
            from_dataset(row['action'], manifest['dataContract'])
            for side in ('left', 'right'):
                vector(row[f'observation.force_{side}'], 6)
            if not 0 <= row['sim_time']-row['action_time'] <= .25:
                raise ValueError('invalid action time')
            if j and abs(row['sim_time']-rows[j-1]['sim_time']-1/fps) > 1e-6:
                raise ValueError('non-contiguous simulation time')
            if set(row['images']) != set(CAMERAS):
                raise ValueError('camera set mismatch')
            for relative in row['images'].values():
                with Image.open(local_file(relative)) as image:
                    image.load()
                    if image.size != (640, 480) or image.mode != 'RGB':
                        raise ValueError('camera requires 640x480 RGB')
        episodes.append(rows)
    grip_presence = ['observation.grip_force' in row for rows in episodes for row in rows]
    if any(grip_presence) and not all(grip_presence):
        raise ValueError('mixed gripping-force availability')
    for rows in episodes:
        for row in rows:
            if 'observation.grip_force' in row and min(vector(row['observation.grip_force'],2)) < 0:
                raise ValueError('invalid gripping force')
    return manifest, episodes


def export_session(root, output, repo_id, image_dtype='image'):
    manifest, episodes = load_session(root)
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    import numpy as np
    from PIL import Image
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    contract = manifest['dataContract']
    features = {key: {'dtype': 'float32', 'shape': (14,), 'names': contract['stateOrder']}
                for key in ('observation.state', 'action')}
    if 'observation.grip_force' in episodes[0][0]:
        features['observation.grip_force'] = {'dtype':'float32','shape':(2,),
                                              'names':['operator_left_N','operator_right_N']}
    for side in ('left', 'right'):
        features[f'observation.force_{side}'] = {'dtype': 'float32', 'shape': (6,),
                                                'names': ['fx', 'fy', 'fz', 'mx', 'my', 'mz']}
    for name in CAMERAS.values():
        features[f'observation.images.{name}'] = {'dtype': image_dtype, 'shape': (480,640,3),
                                                 'names': ['height','width','channels']}
    ds = LeRobotDataset.create(repo_id=repo_id, fps=manifest['fps'], root=output,
                               robot_type='micromani_isaac', features=features,
                               use_videos=image_dtype == 'video', image_writer_processes=0,
                               image_writer_threads=6, metadata_buffer_size=1)
    for rows in episodes:
        for row in rows:
            frame = {key: np.asarray(row[key], dtype=np.float32) for key in features if 'images' not in key}
            frame['task'] = row['task']
            for raw, name in CAMERAS.items():
                with Image.open(Path(root)/row['images'][raw]) as image:
                    frame[f'observation.images.{name}'] = np.asarray(image).copy()
            ds.add_frame(frame)
        ds.save_episode(parallel_encoding=False)
    ds.finalize()
    meta = output/'meta'
    for i, rows in enumerate(episodes):
        timing = [{'frame_index': r['frame_index'], 'sim_time': r['sim_time'],
                   'action_time': r['action_time']} for r in rows]
        (meta/f'sim_timing_{i:06d}.jsonl').write_text(
            ''.join(json.dumps(r)+'\n' for r in timing))
    (meta/'micromani_sim_info.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    info_path = meta/'info.json'
    info = json.loads(info_path.read_text())
    info['dataContract'] = contract
    info_path.write_text(json.dumps(info, ensure_ascii=False, indent=2))
    loaded = LeRobotDataset(repo_id, root=output, return_uint8=True)
    if len(loaded) != sum(len(rows) for rows in episodes):
        raise RuntimeError('LeRobot reload frame count mismatch')
    return {'frames': len(loaded), 'episodes': len(episodes), 'contract': VERSION}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--session', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--repo-id', default='micromani/isaac_operator_sim')
    p.add_argument('--image-dtype', choices=['image','video'], default='image')
    a = p.parse_args()
    print(json.dumps(export_session(a.session, a.output, a.repo_id, a.image_dtype)))
