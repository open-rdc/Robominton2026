"""Validated robot design configuration shared by launch, URDF and simulation."""
from copy import deepcopy
import math
from pathlib import Path
import yaml

FIELDS = {
    'chassis': 'size center_z mass',
    'wheels': 'wheelbase track radius width mass drive_effort drive_velocity steer_size steer_mass steer_effort steer_velocity damping friction',
    'mast': 'size xyz mass',
    'camera': 'size xyz rpy mass width height fx fy cx cy rate',
    'lidar': 'size xyz rpy mass lower upper initial effort velocity damping friction angle_min angle_max samples range_min range_max rate',
    'racket': 'pivot rpy handle_length handle_width handle_mass grip_length grip_width reach radius thickness head_mass lower upper effort velocity damping friction ready backswing follow_through manual_velocity preparation_time stroke_time hold_time return_time',
}
POSITIVE = set('center_z mass wheelbase track radius width height drive_effort drive_velocity steer_mass steer_effort steer_velocity fx fy rate samples range_min range_max effort velocity handle_length handle_width handle_mass grip_length grip_width reach thickness head_mass manual_velocity preparation_time stroke_time return_time'.split())
NONNEGATIVE = {'damping', 'friction', 'hold_time'}
VECTORS = {'size', 'steer_size', 'xyz', 'rpy', 'pivot'}


def validate_robot_config(data):
    data = deepcopy(data)
    if not isinstance(data, dict) or set(data) != set(FIELDS):
        raise ValueError('robot_config groups must be: ' + ', '.join(FIELDS))
    for group, fields in FIELDS.items():
        values = data[group]
        if not isinstance(values, dict) or set(values) != set(fields.split()):
            raise ValueError(group + ': missing or unknown fields; expected ' + fields)
        for name, value in values.items():
            label = group + '.' + name
            if name in VECTORS:
                if not isinstance(value, list) or len(value) != 3:
                    raise ValueError(label + ' must contain exactly 3 numbers')
                if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
                    raise ValueError(label + ' must contain finite numbers')
                values[name] = list(map(float, value))
                if name in {'size', 'steer_size'} and min(value) <= 0:
                    raise ValueError(label + ' dimensions must be positive')
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(label + ' must be a finite number')
            integer = group == 'camera' and name in {'width', 'height'} or name == 'samples'
            if integer and (value != int(value) or value < 1):
                raise ValueError(label + ' must be a positive integer')
            values[name] = int(value) if integer else float(value)
            if name in POSITIVE and value <= 0:
                raise ValueError(label + ' must be positive')
            if name in NONNEGATIVE and value < 0:
                raise ValueError(label + ' must be nonnegative')
    body, camera, lidar, racket = [data[key] for key in ['chassis', 'camera', 'lidar', 'racket']]
    if body['center_z'] < body['size'][2] / 2:
        raise ValueError('chassis bottom must be on or above the ground')
    for group in ('camera', 'lidar', 'mast'):
        if data[group]['xyz'][2] <= 0:
            raise ValueError(group + '.xyz z must be above ground')
    if not (0 <= camera['cx'] < camera['width'] and 0 <= camera['cy'] < camera['height']):
        raise ValueError('camera principal point must be inside the image')
    if camera['rate'] > 50 or lidar['rate'] > 50:
        raise ValueError('sensor rate must be <= 50 Hz (scene update rate)')
    if not lidar['lower'] < lidar['upper'] or not lidar['lower'] <= lidar['initial'] <= lidar['upper']:
        raise ValueError('lidar initial angle must lie within ordered joint limits')
    if not (lidar['samples'] >= 2 and 0 < lidar['angle_max']-lidar['angle_min'] <= 2*math.pi+1e-9):
        raise ValueError('lidar scan requires >= 2 samples and angular span in (0, 2*pi]')
    if lidar['range_min'] >= lidar['range_max']:
        raise ValueError('lidar range_min must be smaller than range_max')
    if not racket['lower'] <= racket['backswing'] < racket['ready'] < racket['follow_through'] <= racket['upper']:
        raise ValueError('racket requires lower <= backswing < ready < follow_through <= upper')
    if racket['manual_velocity'] > racket['velocity']:
        raise ValueError('racket manual_velocity exceeds joint velocity')
    if not racket['grip_length'] <= racket['handle_length'] <= racket['reach']:
        raise ValueError('racket requires grip_length <= handle_length <= reach')
    if racket['pivot'][2] <= 0:
        raise ValueError('racket pivot must be above ground')
    return data


def load_robot_config(path):
    with Path(path).open() as stream:
        return validate_robot_config(yaml.safe_load(stream))


def node_parameters(config):
    """Translate design fields to node params; design overrides sim.yaml geometry."""
    wh, cam, lid, rac = [config[key] for key in ['wheels', 'camera', 'lidar', 'racket']]
    base = {'wheelbase': wh['wheelbase'], 'track': wh['track'], 'wheel_radius': wh['radius'],
            'max_wheel_speed': wh['drive_velocity']}
    scene = {}
    for prefix, sensor in [('camera', cam), ('lidar', lid)]:
        for key, value in sensor.items():
            if key not in {'size', 'mass', 'effort', 'damping', 'friction'}:
                scene[prefix + '_' + key] = value
    for index, axis in enumerate('xyz'):
        scene['racket_pivot_' + axis] = rac['pivot'][index]
    for index, axis in enumerate(('roll', 'pitch', 'yaw')):
        scene['racket_mount_' + axis] = rac['rpy'][index]
    for name in ('reach', 'radius', 'thickness'):
        scene['racket_' + name] = rac[name]
    for name in ('lower', 'upper', 'ready', 'backswing', 'follow_through', 'velocity',
                 'manual_velocity', 'preparation_time', 'stroke_time', 'hold_time', 'return_time'):
        scene['swing_' + name] = rac[name]
    return base, scene
