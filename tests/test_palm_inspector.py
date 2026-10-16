from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from tools import palm_inspector as inspector
from tools import urdf_observation_viewer as viewer

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('filename,kind,count,palm', [
    ('xarm7_wuji_90.urdf', 'wuji', 27, 'right_palm_link'),
    ('xarm7_wuji_90_camera.urdf', 'wuji', 27, 'right_palm_link'),
    ('xarm7_wuji_direct.urdf', 'wuji', 27, 'right_palm_link'),
    ('xarm7_wuji_direct_camera.urdf', 'wuji', 27, 'right_palm_link'),
    ('xarm7_xhand_direct.urdf', 'xhand', 19, 'right_hand_link'),
    ('xarm7_xhand_direct_camera.urdf', 'xhand', 19, 'right_hand_link'),
    ('assets/urdf/kuka_sharpa_description/iiwa14_left_sharpa_adjusted_restricted.urdf', 'sharpa', 29, 'left_hand_C_MC'),
])
def test_repository_hand_mapping(filename, kind, count, palm):
    model = viewer._load_urdf(ROOT / filename, load_meshes=False)
    detected, names, parent, tips = inspector.identify_hand(model)
    assert (detected, len(names), parent) == (kind, count, palm)
    assert names == inspector.KNOWN_HANDS[kind][0]
    assert len(tips) == 5
    model.update_cfg({name: 0.1 for name in names})
    local = inspector.initial_palm_transform(model, kind, parent)
    if kind == 'wuji':
        np.testing.assert_allclose(local[:3, 3], [0.004, -0.0023529, 0.0434161])
    elif kind == 'xhand':
        np.testing.assert_allclose(local, np.eye(4))
    else:
        wrist = model.get_transform('iiwa14_link_7')
        world = model.get_transform(parent) @ local
        np.testing.assert_allclose(world[:3, 3], viewer.transform_point(wrist, np.array([0, -0.02, 0.16])))


def test_unknown_joints_and_mimic(tmp_path):
    path = tmp_path / 'wuji_filename_does_not_identify_hand.urdf'
    path.write_text('''<robot name="new_hand">
    <link name="base"/><link name="slide"/><link name="turn"/><link name="follower"/>
    <joint name="slide_joint" type="prismatic"><parent link="base"/><child link="slide"/>
      <axis xyz="1 0 0"/><limit lower="0" upper="0.2" effort="1" velocity="1"/></joint>
    <joint name="spin_joint" type="continuous"><parent link="slide"/><child link="turn"/>
      <axis xyz="0 0 1"/></joint>
    <joint name="follow_joint" type="revolute"><parent link="turn"/><child link="follower"/>
      <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
      <mimic joint="spin_joint" multiplier="0.5" offset="0"/></joint>
    </robot>''')
    model = viewer._load_urdf(path, load_meshes=False)
    kind, names, parent, tips = inspector.identify_hand(model)
    assert (kind, names, parent, tips) == ('generic', ('slide_joint', 'spin_joint'), 'base', ())
    model.update_cfg({'slide_joint': 0.1, 'spin_joint': 0.2})
    np.testing.assert_allclose(model.get_transform('slide')[:3, 3], [0.1, 0, 0])
    assert np.isfinite(model.get_transform('follower')).all()


def test_generic_gripper():
    model = viewer._load_urdf(ROOT / 'xarm7_gripper.urdf', load_meshes=False)
    kind, names, parent, tips = inspector.identify_hand(model)
    assert kind == 'generic'
    assert names == tuple(model.actuated_joint_names)
    assert parent in model.link_map


def test_export_is_fixed_joint_with_selected_parent():
    xml = inspector.palm_xml('right_hand_link', [0.01, 0.02, 0.03], [0.1, 0.2, 0.3])
    robot = ET.fromstring('<robot>' + xml + '</robot>')
    joint = robot.find('joint')
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'right_hand_link'
    np.testing.assert_allclose(np.fromstring(joint.find('origin').get('rpy'), sep=' '), [0.1, 0.2, 0.3])
    with pytest.raises(ValueError):
        inspector.palm_xml('palm_center', [0, 0, 0], [0, 0, 0])


def _snapshot(model, kind, parent, tips, **changes):
    params = dict(
        model=model, kind=kind, parent=parent, local_palm=np.eye(4),
        object_position=np.array([0.45, 0.02, 0.56]), object_wxyz=[1, 0, 0, 0],
        object_size=[0.04, 0.06, 0.08], keypoint_scale=1.5,
        anchor='virtual', tip_links=tips,
    )
    params.update(changes)
    return inspector.compute_single_observation(**params)


def test_xhand_object_and_palm_updates():
    model = viewer._load_urdf(ROOT / 'xarm7_xhand_direct.urdf', load_meshes=False)
    kind, names, parent, tips = inspector.identify_hand(model)
    model.update_cfg({n: 0.1 for n in names})
    first = _snapshot(model, kind, parent, tips)
    assert len(first['geometry_vector']) == 34
    assert first['tip_links'] == list(tips)
    # Moving the object shifts all keypoint vectors by the same world delta.
    delta = np.array([0.03, -0.02, 0.01])
    moved = _snapshot(model, kind, parent, tips, object_position=np.array(first['object_position']) + delta)
    np.testing.assert_allclose(np.array(moved['geometry_fields']['keypoints_rel_palm']) - first['geometry_fields']['keypoints_rel_palm'], np.tile(delta, (4, 1)))
    np.testing.assert_allclose(moved['geometry_fields']['fingertip_pos_rel_palm'], first['geometry_fields']['fingertip_pos_rel_palm'])
    # Local palm translation shifts every relative vector in base axes.
    local = np.eye(4)
    local[:3, 3] = [0.01, 0.02, 0.03]
    shifted = _snapshot(model, kind, parent, tips, local_palm=local)
    world_delta = model.get_transform(parent)[:3, :3] @ local[:3, 3]
    np.testing.assert_allclose(np.array(shifted['geometry_fields']['fingertip_pos_rel_palm']) - first['geometry_fields']['fingertip_pos_rel_palm'], np.tile(-world_delta, (5, 1)))
    # Object rotation and dimensions are applied before world translation.
    rotated = _snapshot(model, kind, parent, tips, object_wxyz=[np.sqrt(.5), 0, 0, np.sqrt(.5)], object_size=[.1, .2, .3])
    rotation = viewer.wxyz_to_matrix([np.sqrt(.5), 0, 0, np.sqrt(.5)])
    expected = (rotation @ (viewer.KEYPOINT_CORNERS * np.array([.1, .2, .3]) * .75).T).T + first['object_position']
    np.testing.assert_allclose(rotated['object_keypoints'], expected)


@pytest.mark.parametrize('kind,path', [('wuji', viewer.OLD_URDF), ('sharpa', viewer.SIMTOOLREAL_URDF)])
def test_single_legacy_matches_original_geometry(kind, path):
    model = viewer._load_urdf(path, load_meshes=False)
    _, names, parent, tips = inspector.identify_hand(model)
    joints = {name: .1 for name in names}
    model.update_cfg(joints)
    actual = _snapshot(model, kind, parent, tips, anchor='legacy')
    if kind == 'sharpa':
        wrist, palm, fingertip_positions = viewer.compute_sharpa_geometry(model, joints)
    else:
        old = viewer.compute_observation_geometry(
            model, joint_values=joints, object_position=np.array(actual['object_position']),
            object_wxyz=np.array([1, 0, 0, 0]), object_size=np.array(actual['object_size']),
            keypoint_scale=1.5, link7_linear_velocity=np.zeros(3), link7_angular_velocity=np.zeros(3),
        )
        wrist, palm, fingertip_positions = old.link7_transform, old.code_palm_position, old.fingertip_positions
        np.testing.assert_allclose(actual['geometry_vector'], old.critical_actor_block(true_palm=False))
    np.testing.assert_allclose(actual['geometry_fields']['palm_pos'], palm)
    np.testing.assert_allclose(actual['geometry_fields']['palm_rot_xyzw'], viewer.rotation_matrix_to_xyzw(wrist[:3, :3]))
    np.testing.assert_allclose(actual['fingertip_positions'], fingertip_positions)


def test_generic_optional_tips_and_invalid_inputs():
    model = viewer._load_urdf(ROOT / 'xarm7_gripper.urdf', load_meshes=False)
    snapshot = _snapshot(model, 'generic', 'link_tcp', ())
    assert len(snapshot['geometry_vector']) == 19
    assert snapshot['fingertip_positions'] == []
    snapshot = _snapshot(model, 'generic', 'link_tcp', ('link_tcp',))
    np.testing.assert_allclose(snapshot['geometry_fields']['fingertip_pos_rel_palm'], [[0, 0, 0]])
    assert len(snapshot['geometry_vector']) == 22
    with pytest.raises(ValueError, match='Unknown fingertip'):
        _snapshot(model, 'generic', 'link_tcp', ('missing',))
    with pytest.raises(ValueError):
        _snapshot(model, 'generic', 'link_tcp', (), object_size=[0, .1, .1])
    with pytest.raises(ValueError, match='Unsupported observation anchor'):
        _snapshot(model, 'generic', 'link_tcp', (), anchor='legacy')
