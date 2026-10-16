import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from tools.urdf_palm import save_palm_frame
from tools.urdf_observation_viewer import _load_urdf

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('filename,parent', [('xarm7_xhand_direct.urdf', 'right_hand_link'), ('xarm7_wuji_90.urdf', 'right_palm_link')])
def test_save_add_update_and_preserve_robot(tmp_path, filename, parent):
    path = tmp_path / filename
    shutil.copyfile(ROOT / filename, path)
    def other_elements():
        return [ET.tostring(e) for e in ET.parse(path).getroot()
                if e.get('name') not in ('palm_center', 'palm_center_joint')]
    before = other_elements()
    model = _load_urdf(path, load_meshes=False)
    joints = model.actuated_joint_names
    save_palm_frame(path, parent, [.01001, .02002, .03003], [.0001, .0002, .0003])
    save_palm_frame(path, parent, [.04001, .02002, .03003], [.0004, .0002, .0003])
    root = ET.parse(path).getroot()
    assert len(root.findall("link[@name='palm_center']")) == 1
    assert len(root.findall("joint[@name='palm_center_joint']")) == 1
    assert before == other_elements()
    restored = _load_urdf(path, load_meshes=False)
    assert restored.actuated_joint_names == joints
    local = restored.get_transform('palm_center', parent)
    np.testing.assert_allclose(local[:3, 3], [.04001, .02002, .03003])
    from scipy.spatial.transform import Rotation
    np.testing.assert_allclose(local[:3, :3], Rotation.from_euler('xyz', [.0004, .0002, .0003]).as_matrix())


def test_save_rejects_cycles_and_invalid_values_without_write(tmp_path):
    path = tmp_path / 'robot.urdf'
    path.write_text('''<robot name="test"><!--keep this--><link name="base"/>
<link name="palm_center"/><link name="child"/>
<joint name="custom_palm" type="fixed"><parent link="base"/><child link="palm_center"/></joint>
<joint name="descendant" type="fixed"><parent link="palm_center"/><child link="child"/></joint></robot>''')
    before = path.read_bytes()
    for parent, xyz in [('missing', [0, 0, 0]), ('palm_center', [0, 0, 0]), ('child', [0, 0, 0]), ('base', [float('nan'), 0, 0])]:
        with pytest.raises(ValueError):
            save_palm_frame(path, parent, xyz, [0, 0, 0])
        assert path.read_bytes() == before
    save_palm_frame(path, 'base', [1, 2, 3], [0, 0, 0])
    root = ET.parse(path).getroot()
    assert root.find("joint[@name='custom_palm']/origin") is not None
    assert root.find("joint[@name='palm_center_joint']") is None
    assert b'<!--keep this-->' in path.read_bytes()
