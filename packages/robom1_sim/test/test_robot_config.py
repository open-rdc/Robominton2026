from pathlib import Path
import numpy as np
import pytest
import xacro
from xml.etree import ElementTree as ET
from robom1_sim.physics import RacketGeometry, racket_pose, collide_racket
from robom1_sim.robot_config import load_robot_config, node_parameters, validate_robot_config
from robom1_sim.swing import PitchSwing

DESCRIPTION = Path(__file__).resolve().parents[2] / 'robom1_description'


@pytest.mark.parametrize('name', ['robot.yaml', 'robot_large.example.yaml'])
def test_design_expands_to_connected_urdf_and_physical_inertias(name):
    path = DESCRIPTION / 'config' / name
    cfg = load_robot_config(path)
    robot = ET.fromstring(xacro.process_file(str(DESCRIPTION/'urdf'/'robom1.urdf.xacro'),
                                            mappings={'robot_config': str(path)}).toxml())
    links = {link.attrib['name']: link for link in robot.findall('link')}
    children = set()
    for joint in robot.findall('joint'):
        assert joint.find('parent').attrib['link'] in links
        child = joint.find('child').attrib['link']
        assert child in links and child not in children
        children.add(child)
    assert set(links) - children == {'base_footprint'}
    assert len(links) == 16 and len(children) == 15
    for link in links.values():
        inertia = link.find('inertial')
        if inertia is None:
            continue
        assert float(inertia.find('mass').attrib['value']) > 0
        values = inertia.find('inertia').attrib
        diagonal = np.array([float(values[key]) for key in ['ixx','iyy','izz']])
        assert np.all(diagonal > 0) and np.all(2*diagonal <= sum(diagonal)+1e-10)
    wh = cfg['wheels']
    wheel = links['front_left_wheel_link']
    dimensions = wheel.find('collision/geometry/cylinder').attrib
    assert float(dimensions['radius']) == wh['radius']
    assert float(dimensions['length']) == wh['width']
    # Solid cylindrical head inertia about its local x normal, not a box inertia.
    head = links['racket_head_link'].find('inertial/inertia').attrib
    assert float(head['ixx']) == pytest.approx(cfg['racket']['head_mass']*cfg['racket']['radius']**2/2)
    assert float(links['base_link'].find('inertial/mass').attrib['value']) == cfg['chassis']['mass']
    base_params, scene_params = node_parameters(cfg)
    assert base_params['wheel_radius'] == wh['radius']
    assert scene_params['camera_xyz'] == cfg['camera']['xyz']
    assert scene_params['lidar_velocity'] == cfg['lidar']['velocity']
    for name, group, key in [('camera_joint','camera','xyz'), ('lidar_pitch_joint','lidar','xyz'),
                              ('racket_swing_joint','racket','pivot')]:
        joint = robot.find("joint[@name='{}']".format(name))
        position = np.fromstring(joint.find('origin').attrib['xyz'],sep=' ')
        position[2] += cfg['chassis']['center_z']
        np.testing.assert_allclose(position,cfg[group][key])
        np.testing.assert_allclose(np.fromstring(joint.find('origin').attrib['rpy'],sep=' '),cfg[group]['rpy'])
        if group != 'camera':
            assert float(joint.find('limit').attrib['velocity']) == cfg[group]['velocity']


@pytest.mark.parametrize('group,key,value', [
    ('wheels','radius',0), ('wheels','mass',-1), ('chassis','size',[1,2]),
    ('camera','rpy',[0,float('nan'),0]), ('camera','fx',float('inf')),
    ('camera','width',320.5), ('camera','cx',9999), ('camera','rate',60),
    ('lidar','samples',1), ('lidar','range_min',100), ('lidar','upper',-1),
    ('lidar','initial',5), ('racket','ready',4), ('racket','velocity',0),
    ('racket','manual_velocity',100), ('racket','handle_length',2),
    ('racket','hold_time',-1), ('wheels','mass',True),
])
def test_rejects_invalid_design_before_nodes_start(group,key,value):
    cfg=load_robot_config(DESCRIPTION/'config'/'robot.yaml')
    cfg[group][key]=value
    with pytest.raises(ValueError):
        validate_robot_config(cfg)


def test_unknown_field_is_rejected_instead_of_silently_ignored():
    cfg=load_robot_config(DESCRIPTION/'config'/'robot.yaml')
    cfg['camera']['pitch_angle_typo']=.5
    with pytest.raises(ValueError):
        validate_robot_config(cfg)


def test_slow_swing_extends_times_instead_of_exceeding_joint_velocity():
    cfg=load_robot_config(DESCRIPTION/'config'/'robot_large.example.yaml')
    _, params=node_parameters(cfg)
    swing=PitchSwing(**{key[6:]:value for key,value in params.items() if key.startswith('swing_')})
    swing.trigger()
    previous=swing.position
    extrema=[previous]
    while swing.active:
        current=swing.step(.001)
        assert abs(current-previous)/.001 <= cfg['racket']['velocity']+1e-7
        extrema.append(current)
        previous=current
    assert min(extrema) == pytest.approx(cfg['racket']['backswing'],abs=1e-4)
    assert max(extrema) == pytest.approx(cfg['racket']['follow_through'],abs=1e-4)
    assert current == cfg['racket']['ready']


def test_mounted_racket_collision_uses_urdf_fixed_rotation_and_local_axis():
    cfg=load_robot_config(DESCRIPTION/'config'/'robot_large.example.yaml')
    _, params=node_parameters(cfg)
    defaults=vars(RacketGeometry())
    geometry=RacketGeometry(**{key:params['racket_'+key] for key in defaults})
    base=np.array([2.,1.,0.])
    angle=.2
    pivot,center,orientation=racket_pose(base,.4,angle,geometry)
    normal=orientation[:,0]
    _,outgoing,hit=collide_racket(center+.02*normal,-2*normal,.035,base,.4,angle,
                                np.zeros(3),0,0,geometry,.8)
    assert hit is not None
    np.testing.assert_allclose(outgoing,1.6*normal,atol=1e-10)
