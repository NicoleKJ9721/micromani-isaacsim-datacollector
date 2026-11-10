"""Nominal robot-only FK. No task objects or simulator state are read.

Column vectors, metres/radians, base_link frame. This is design calibration,
not measured hardware calibration. Encoders must come from the public API.
"""
import numpy as np
from micromani_home_geometry import xml, meshes, transform, vals, rot
from micromani_lerobot_schema import DEFAULT_HOME_ACTION, control_action_to_raw_targets


def frames(action):
    q = control_action_to_raw_targets(action)
    result = {'base_link': np.eye(4)}
    pending = list(xml.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in result:
                continue
            origin = joint.find('origin')
            matrix = transform(vals(origin, 'xyz'), vals(origin, 'rpy'))
            motion = np.eye(4)
            value = q.get(joint.get('name'), 0.)
            if joint.get('type') in ('revolute', 'continuous', 'prismatic'):
                axis = vals(joint.find('axis'), 'xyz')
                if joint.get('type') == 'prismatic':
                    motion[:3, 3] = np.asarray(axis) * value
                else:
                    motion[:3, :3] = rot(axis, value)
            result[joint.find('child').get('link')] = result[parent] @ matrix @ motion
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('Disconnected robot kinematic tree')
    return result


def tcp_offsets():
    """Tip front-plane midpoint between fingers, expressed in link6 frame."""
    home = frames(DEFAULT_HOME_ACTION)
    result = {}
    for side, inward in (('left', 1), ('right', -1)):
        centers = []
        for index in (1, 2):
            name = f'link_finger{index}_{side}'
            points = meshes[name] @ home[name][:3, :3].T + home[name][:3, 3]
            tip = points[inward * points[:, 0] > (inward * points[:, 0]).max() - 1e-6]
            centers.append((tip.min(0) + tip.max(0)) / 2)
        result[side] = (np.linalg.inv(home['link6_' + side]) @ np.r_[np.mean(centers, axis=0), 1])[:3]
    return result


TCP_OFFSETS = tcp_offsets()


def tcp(action, side):
    matrix = frames(action)['link6_' + side].copy()
    matrix[:3, 3] += matrix[:3, :3] @ TCP_OFFSETS[side]
    return matrix
