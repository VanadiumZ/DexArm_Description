"""Write an explicitly confirmed virtual palm frame to a URDF."""
from pathlib import Path
import os
import tempfile

import numpy as np
from lxml import etree


def save_palm_frame(path, parent, xyz, rpy):
    path = Path(path).resolve(strict=True)
    values = np.asarray([xyz, rpy], dtype=float)
    if values.shape != (2, 3) or not np.isfinite(values).all():
        raise ValueError("掌心 xyz/rpy 必须各包含三个有限数值")
    parser = etree.XMLParser(remove_blank_text=False, resolve_entities=False)
    tree = etree.parse(str(path), parser)
    root = tree.getroot()
    if root.tag != 'robot':
        raise ValueError("文件不是 URDF robot")
    links = root.findall('link')
    joints = root.findall('joint')
    if parent not in {link.get('name') for link in links}:
        raise ValueError(f"URDF 中不存在参考 link：{parent}")
    descendants = {'palm_center'}
    for _ in joints:
        for joint in joints:
            if joint.find('parent').get('link') in descendants:
                descendants.add(joint.find('child').get('link'))
    if parent in descendants:
        raise ValueError("掌心参考 link 不能是 palm_center 或其子节点，否则会形成环")
    palms = [link for link in links if link.get('name') == 'palm_center']
    attachments = [joint for joint in joints if joint.find('child').get('link') == 'palm_center']
    if len(palms) > 1 or len(attachments) > 1:
        raise ValueError("URDF 中存在重复的 palm_center 定义")
    if attachments and attachments[0].get('type') != 'fixed':
        raise ValueError("已有 palm_center 不是固定关节，不能直接覆写")
    if not attachments and any(joint.get('name') == 'palm_center_joint' for joint in joints):
        raise ValueError("palm_center_joint 名称已被其他关节使用")
    if not palms:
        link = etree.SubElement(root, 'link', name='palm_center')
        link.tail = '\n'
    if attachments:
        joint = attachments[0]
    else:
        joint = etree.SubElement(root, 'joint', name='palm_center_joint', type='fixed')
        joint.text = '\n    '
        etree.SubElement(joint, 'parent', link=parent).tail = '\n    '
        etree.SubElement(joint, 'child', link='palm_center').tail = '\n    '
        joint.tail = '\n'
    joint.find('parent').set('link', parent)
    origin = joint.find('origin')
    if origin is None:
        origin = etree.SubElement(joint, 'origin')
        origin.tail = '\n  '
    origin.set('xyz', ' '.join(format(v, '.17g') for v in values[0]))
    origin.set('rpy', ' '.join(format(v, '.17g') for v in values[1]))
    # Replace only after a complete write; keep existing comments and mesh paths.
    data = etree.tostring(tree, encoding='utf-8', xml_declaration=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.' + path.name, suffix='.tmp', delete=False) as output:
            temporary = Path(output.name)
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path
