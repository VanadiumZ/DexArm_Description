"""Single-URDF palm tuning with explicit known-hand mappings."""
from __future__ import annotations

import json
import threading
import time
from xml.sax.saxutils import quoteattr

import numpy as np
from scipy.spatial.transform import Rotation
from lxml import etree

from tools.urdf_palm import save_palm_frame

from tools.urdf_observation_viewer import (
    UrdfView, _load_urdf, OLD_START_ARM_POSE, NEW_START_ARM_POSE,
    SHARPA_START_ARM, xyzw_to_wxyz, KEYPOINT_CORNERS,
    FINGERTIP_LINK_NAMES, FINGERTIP_OFFSETS_LINK4, SHARPA_TIP_LINKS,
    transform_point, rotation_matrix_to_xyzw, wxyz_to_matrix,
)

# Fixed control order: arm, then hand. Tips use thumb/index/middle/ring/pinky.
# These are authored tip frames, not the historical observation pad offsets.
KNOWN_HANDS = {'wuji': (('joint1',
           'joint2',
           'joint3',
           'joint4',
           'joint5',
           'joint6',
           'joint7',
           'right_finger1_joint1',
           'right_finger1_joint2',
           'right_finger1_joint3',
           'right_finger1_joint4',
           'right_finger2_joint1',
           'right_finger2_joint2',
           'right_finger2_joint3',
           'right_finger2_joint4',
           'right_finger3_joint1',
           'right_finger3_joint2',
           'right_finger3_joint3',
           'right_finger3_joint4',
           'right_finger4_joint1',
           'right_finger4_joint2',
           'right_finger4_joint3',
           'right_finger4_joint4',
           'right_finger5_joint1',
           'right_finger5_joint2',
           'right_finger5_joint3',
           'right_finger5_joint4'),
          'right_palm_link',
          ('right_finger1_tip_link',
           'right_finger2_tip_link',
           'right_finger3_tip_link',
           'right_finger4_tip_link',
           'right_finger5_tip_link')),
 'sharpa': (('iiwa14_joint_1',
             'iiwa14_joint_2',
             'iiwa14_joint_3',
             'iiwa14_joint_4',
             'iiwa14_joint_5',
             'iiwa14_joint_6',
             'iiwa14_joint_7',
             'left_1_thumb_CMC_FE',
             'left_thumb_CMC_AA',
             'left_thumb_MCP_FE',
             'left_thumb_MCP_AA',
             'left_thumb_IP',
             'left_2_index_MCP_FE',
             'left_index_MCP_AA',
             'left_index_PIP',
             'left_index_DIP',
             'left_3_middle_MCP_FE',
             'left_middle_MCP_AA',
             'left_middle_PIP',
             'left_middle_DIP',
             'left_4_ring_MCP_FE',
             'left_ring_MCP_AA',
             'left_ring_PIP',
             'left_ring_DIP',
             'left_5_pinky_CMC',
             'left_pinky_MCP_FE',
             'left_pinky_MCP_AA',
             'left_pinky_PIP',
             'left_pinky_DIP'),
            'left_hand_C_MC',
            ('left_thumb_fingertip',
             'left_index_fingertip',
             'left_middle_fingertip',
             'left_ring_fingertip',
             'left_pinky_fingertip')),
 'xhand': (('joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'joint7',
            'right_hand_thumb_bend_joint',
            'right_hand_thumb_rota_joint1',
            'right_hand_thumb_rota_joint2',
            'right_hand_index_bend_joint',
            'right_hand_index_joint1',
            'right_hand_index_joint2',
            'right_hand_mid_joint1',
            'right_hand_mid_joint2',
            'right_hand_ring_joint1',
            'right_hand_ring_joint2',
            'right_hand_pinky_joint1',
            'right_hand_pinky_joint2'),
           'right_hand_link',
           ('right_hand_thumb_rota_tip',
            'right_hand_index_rota_tip',
            'right_hand_mid_tip',
            'right_hand_ring_tip',
            'right_hand_pinky_tip'))}


def identify_hand(model):
    """Recognize by hand joint/link signature, independently of the filename.

    Known hand joints retain their fixed order; extra actuated joints follow
    in URDF order. An unfamiliar hand uses the full URDF order.
    """
    names = tuple(model.actuated_joint_names)
    for kind, (fixed_order, palm, tips) in KNOWN_HANDS.items():
        hand_joints = fixed_order[7:]
        if palm in model.link_map and set(hand_joints).issubset(names):
            ordered = tuple(n for n in fixed_order if n in names)
            ordered += tuple(n for n in names if n not in ordered)
            return kind, ordered, palm, tuple(t for t in tips if t in model.link_map)
    return "generic", names, model.base_link, ()


def initial_palm_transform(model, kind, parent):
    if "palm_center" in model.link_map:
        return np.array(model.get_transform("palm_center", parent), copy=True)
    transform = np.eye(4)
    if kind == "sharpa":
        wrist = model.get_transform("iiwa14_link_7", parent) if "iiwa14_link_7" in model.link_map else None
        if wrist is not None:
            transform = np.array(wrist, copy=True)
            transform[:3, 3] += wrist[:3, :3] @ np.array([0.0, -0.02, 0.16])
    return transform


def palm_xml(parent, xyz, rpy):
    if parent == "palm_center":
        raise ValueError("palm_center cannot be its own parent")
    position = " ".join(f"{v:.9g}" for v in xyz)
    rotation = " ".join(f"{v:.9g}" for v in rpy)
    return (f'<link name="palm_center"/>\n'
            f'<joint name="palm_center_joint" type="fixed">\n'
            f'  <parent link={quoteattr(parent)}/>\n'
            f'  <child link="palm_center"/>\n'
            f'  <origin xyz="{position}" rpy="{rotation}"/>\n</joint>')


def observation_tip_definition(kind, selected_links):
    if kind == "wuji":
        return FINGERTIP_LINK_NAMES, FINGERTIP_OFFSETS_LINK4
    if kind == "sharpa":
        return SHARPA_TIP_LINKS, np.tile([0.02, 0.002, 0.0], (5, 1))
    return tuple(selected_links), np.zeros((len(selected_links), 3))


def compute_single_observation(model, kind, parent, local_palm, object_position,
                               object_wxyz, object_size, keypoint_scale,
                               anchor="virtual", tip_links=()):
    """Static FK geometry in robot-base axes; not a full policy observation."""
    size = np.asarray(object_size, dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError("物体 xyz 尺寸必须为正的有限数值")
    if not np.isfinite(keypoint_scale) or keypoint_scale <= 0:
        raise ValueError("keypoint scale 必须为正的有限数值")
    virtual = model.get_transform(parent) @ local_palm
    if anchor == "virtual":
        palm = virtual
    elif anchor == "semantic":
        palm = model.get_transform(parent)
    elif anchor == "legacy" and kind == "wuji":
        palm = model.get_transform("link7")
    elif anchor == "legacy" and kind == "sharpa":
        palm = np.array(model.get_transform("iiwa14_link_7"), copy=True)
        palm[:3, 3] = transform_point(palm, np.array([0.0, -0.02, 0.16]))
    else:
        raise ValueError(f"Unsupported observation anchor: {kind}/{anchor}")
    links, offsets = observation_tip_definition(kind, tip_links)
    missing = [name for name in links if name not in model.link_map]
    if missing:
        raise ValueError(f"Unknown fingertip links: {missing}")
    tips = np.array([transform_point(model.get_transform(name), offset)
                     for name, offset in zip(links, offsets)], dtype=float).reshape(-1, 3)
    rotation = wxyz_to_matrix(object_wxyz)
    keypoints = (rotation @ (KEYPOINT_CORNERS * (0.5 * keypoint_scale * size)).T).T + object_position
    fields = {
        "palm_pos": palm[:3, 3],
        "palm_rot_xyzw": rotation_matrix_to_xyzw(palm[:3, :3]),
        "fingertip_pos_rel_palm": tips - palm[:3, 3],
        "keypoints_rel_palm": keypoints - palm[:3, 3],
    }
    slices = {}
    cursor = 0
    for name, value in fields.items():
        slices[name] = [cursor, cursor + value.size]
        cursor += value.size
    vector = np.concatenate([value.ravel() for value in fields.values()])
    assert np.isfinite(vector).all()
    return {
        "hand_type": kind, "anchor": anchor, "palm_parent": parent,
        "coordinate_frame": "robot base axes; relative vectors are world-axis differences",
        "definition": "static geometry block only; not a complete trained-policy input",
        "tip_links": list(links), "tip_offsets_local": np.asarray(offsets).tolist(),
        "geometry_fields": {name: value.tolist() for name, value in fields.items()},
        "geometry_slices": slices, "geometry_vector": vector.tolist(),
        "object_position": np.asarray(object_position).tolist(),
        "object_rot_xyzw": rotation_matrix_to_xyzw(rotation).tolist(),
        "object_size": size.tolist(), "keypoint_scale": float(keypoint_scale),
        "fingertip_positions": tips.tolist(), "object_keypoints": keypoints.tolist(),
        "virtual_palm_position": virtual[:3, 3].tolist(),
        "virtual_palm_rot_xyzw": rotation_matrix_to_xyzw(virtual[:3, :3]).tolist(),
        "palm_local_xyz": local_palm[:3, 3].tolist(),
        "palm_local_rpy": Rotation.from_matrix(local_palm[:3, :3]).as_euler("xyz").tolist(),
    }


def observation_markdown(snapshot):
    lines = ["### Observation 几何分量", f"参考：`{snapshot['anchor']}`；相对向量使用基坐标轴；quaternion 为 xyzw。"]
    for name, values in snapshot["geometry_fields"].items():
        lines.append(f"**{name}**\n```\n{np.array2string(np.asarray(values), precision=5)}\n```")
    lines.append(f"**object_rot_xyzw**: `{snapshot['object_rot_xyzw']}`")
    lines.append(f"**指尖顺序**: `{snapshot['tip_links']}`")
    lines.append(f"**几何向量 {len(snapshot['geometry_vector'])}D**（仅上述四项，非完整策略输入）\n```\n"
                 + np.array2string(np.asarray(snapshot["geometry_vector"]), precision=5) + "\n```")
    lines.append(f"字段区间（左闭右开）：`{snapshot['geometry_slices']}`")
    return "\n\n".join(lines)


def run_single_viewer(args):
    import viser

    model = _load_urdf(args.urdf, load_meshes=True)
    kind, joint_names, default_palm, tips = identify_hand(model)
    # Do not allow a cycle when replacing an existing palm_center joint.
    excluded = {"palm_center"}
    for _ in model.joint_map:
        excluded.update(j.child for j in model.joint_map.values() if j.parent in excluded)
    parents = tuple(n for n in model.link_map if n not in excluded)
    parent = args.palm_link or default_palm
    if parent not in parents:
        raise ValueError(f"Invalid palm parent {parent!r}; choose from {parents}")

    initial = {n: 0.0 for n in joint_names}
    if kind in ("wuji", "xhand") and all(f"joint{i}" in initial for i in range(1, 8)):
        # Select mount from authored geometry, not filename.
        pose = NEW_START_ARM_POSE
        if kind == "wuji" and abs(model.get_transform(default_palm, "link7")[0, 3]) > 0.05:
            pose = OLD_START_ARM_POSE
        initial.update(zip((f"joint{i}" for i in range(1, 8)), pose))
    if kind == "sharpa":
        initial.update((n, v) for n, v in zip(KNOWN_HANDS[kind][0][:7], SHARPA_START_ARM) if n in initial)

    server = viser.ViserServer(host=args.host, port=args.port)
    server.scene.set_up_direction("+z")
    server.gui.add_markdown(f"## {args.urdf.name}\n识别：**{kind}**；{len(joint_names)} 个独立关节。")
    server.gui.add_markdown("已知手型使用固定关节顺序与掌心 link；未知手型使用 URDF 顺序，请选择参考 link。XHand/未知手型没有掌心标定值时从参考 link 原点开始。Wuji/Sharpa 使用历史指尖偏移；XHand 使用 URDF tip frame 原点，尚未做接触点标定。")
    visual_root = server.scene.add_frame("/robot/visual", show_axes=False)
    collision_root = server.scene.add_frame("/robot/collision", show_axes=False, visible=False)
    visual = UrdfView(model, server, root_node_name="/robot/visual", color=(70, 130, 255), collision=False)
    collision = UrdfView(model, server, root_node_name="/robot/collision", color=(255, 70, 70), collision=True)
    show_visual = server.gui.add_checkbox("显示 visual", initial_value=True)
    show_collision = server.gui.add_checkbox("显示 collision", initial_value=False)
    handles = {}
    with server.gui.add_folder("关节控制"):
        for name in joint_names:
            joint = model.joint_map[name]
            if joint.type not in ("revolute", "continuous", "prismatic"):
                raise ValueError(f"Unsupported joint type: {name} ({joint.type})")
            value = initial[name]
            lower = joint.limit.lower if joint.limit is not None else None
            upper = joint.limit.upper if joint.limit is not None else None
            if lower is not None: value = max(value, lower)
            if upper is not None: value = min(value, upper)
            unit = "m" if joint.type == "prismatic" else "rad"
            handles[name] = server.gui.add_number(
                f"{name} ({unit})", initial_value=float(value),
                min=lower, max=upper, step=0.001 if unit == "m" else 0.01,
            )
    model.update_cfg({n: h.value for n, h in handles.items()})
    with server.gui.add_folder("Virtual palm tuning · 手掌局部坐标"):
        parent_control = server.gui.add_dropdown("掌心参考 link", parents, initial_value=parent)
        marker_size = server.gui.add_dropdown(
            "掌心球显示", ("大球 · 半径 25 mm", "小球 · 半径 4 mm", "隐藏球体"),
            initial_value="大球 · 半径 25 mm",
        )
        position_steps = {"1 mm": 0.001, "0.1 mm": 0.0001, "0.01 mm": 0.00001}
        rotation_steps = {"0.01 rad": 0.01, "0.001 rad": 0.001, "0.0001 rad": 0.0001}
        position_step = server.gui.add_dropdown("位置调节步长", tuple(position_steps), initial_value="0.1 mm")
        rotation_step = server.gui.add_dropdown("姿态调节步长", tuple(rotation_steps), initial_value="0.001 rad")
        transform = initial_palm_transform(model, kind, parent)
        xyz = server.gui.add_vector3("xyz (m)", initial_value=tuple(transform[:3, 3]), step=0.0001)
        rpy = server.gui.add_vector3("rpy (rad)", initial_value=tuple(Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")), step=0.001)
        xyz.precision = 7
        rpy.precision = 7
        reset = server.gui.add_button("恢复已加载／确认的掌心")
        confirm_palm = server.gui.add_button("确认掌心并写入 URDF")
        save_status = server.gui.add_markdown(f"保存目标：`{args.urdf.resolve()}`")
        server.gui.add_markdown("大球方便在网格内定位，球心才是掌心参考点，球大小不影响 observation。可切换精调步长，也可直接输入数值。调整时仅预览；点击“确认掌心并写入 URDF”保存当前参考 link、xyz/rpy。已有定义会更新，没有则添加。切换参考 link 会重置局部变换。")
        xml = server.gui.add_markdown("")
    palm_frame = server.scene.add_frame("/palm", axes_length=0.065, axes_radius=0.003)
    palm_markers = {
        label: server.scene.add_icosphere(f"/palm/marker_{index}", radius=radius, color=(230, 80, 255), visible=index == 0)
        for index, (label, radius) in enumerate((("大球 · 半径 25 mm", 0.025), ("小球 · 半径 4 mm", 0.004)))
    }
    with server.gui.add_folder("Object / observation · 物体与观测", expand_by_default=True):
        server.gui.add_markdown("拖动黄色物体控件改变位置和姿态；修改尺寸、关节或掌心后，观测与相对向量实时更新。仅计算静态几何，不填造速度、动作或完整训练输入。")
        object_size = server.gui.add_vector3("物体尺寸 xyz (m)", initial_value=(0.04, 0.04, 0.04), step=0.005)
        keypoint_scale = server.gui.add_number("Keypoint scale", initial_value=1.5, min=0.01, step=0.05)
        anchors = ["virtual", "semantic"]
        if (kind == "wuji" and "link7" in model.link_map) or (kind == "sharpa" and "iiwa14_link_7" in model.link_map):
            anchors.append("legacy")
        anchor_control = server.gui.add_dropdown("Observation 掌心参考", anchors, initial_value="virtual")
        server.gui.add_markdown("virtual：调节后的虚拟掌心；semantic：所选参考 link 原点；legacy：Wuji 的 link7 或 Sharpa 历史偏移点。紫色为虚拟掌心，绿色为当前 observation 参考点。")
        tip_control = None
        if kind == "generic":
            tip_control = server.gui.add_text("指尖 link（逗号分隔，可为空）", initial_value="")
            server.gui.add_markdown("未知型号需手动指定指尖 link，按输入顺序计算；留空只显示掌心与物体分量。")
        show_vectors = server.gui.add_checkbox("显示指尖 / keypoint 相对向量", initial_value=True)
        reset_object = server.gui.add_button("重置物体姿态与尺寸")
        print_snapshot = server.gui.add_button("Print snapshot JSON")
        summary = server.gui.add_markdown("")
    object_control = server.scene.add_transform_controls(
        "/object", position=(0.45, 0.0, 0.56), wxyz=(1.0, 0.0, 0.0, 0.0), scale=0.12,
    )
    object_box = server.scene.add_box("/object/box", dimensions=(0.04, 0.04, 0.04), color=(255, 210, 50))
    observation_frame = server.scene.add_frame("/observation_palm", axes_length=0.08, axes_radius=0.002)
    server.scene.add_icosphere("/observation_palm/marker", radius=0.005, color=(70, 255, 100))
    tip_cloud = server.scene.add_point_cloud("/observation_tips", points=np.zeros((0, 3), dtype=np.float32), colors=(40, 220, 255), point_size=0.012)
    keypoint_cloud = server.scene.add_point_cloud("/object_keypoints", points=np.zeros((4, 3), dtype=np.float32), colors=(255, 220, 40), point_size=0.012)
    vectors = server.scene.add_line_segments("/relative_vectors", points=np.zeros((0, 2, 3), dtype=np.float32), colors=(40, 220, 255), line_width=2.0)
    last_snapshot = {}
    saved_palm = None
    lock = threading.RLock()

    def update(_event=None):
        nonlocal last_snapshot
        with lock, server.atomic():
            for label, sphere in palm_markers.items():
                sphere.visible = marker_size.value == label
            xyz.step = position_steps[position_step.value]
            rpy.step = rotation_steps[rotation_step.value]
            cfg = {n: float(h.value) for n, h in handles.items()}
            visual.update_cfg(cfg)
            collision.update_cfg(cfg)
            visual_root.visible = show_visual.value
            collision_root.visible = show_collision.value
            local = np.eye(4)
            local[:3, :3] = Rotation.from_euler("xyz", rpy.value).as_matrix()
            local[:3, 3] = xyz.value
            world = model.get_transform(parent_control.value) @ local
            palm_frame.position = world[:3, 3]
            palm_frame.wxyz = xyzw_to_wxyz(Rotation.from_matrix(world[:3, :3]).as_quat())
            xml.content = "```xml\n" + palm_xml(parent_control.value, xyz.value, rpy.value) + "\n```"
            selected_tips = tips if tip_control is None else tuple(n.strip() for n in tip_control.value.split(",") if n.strip())
            try:
                snapshot = compute_single_observation(
                    model, kind, parent_control.value, local,
                    np.asarray(object_control.position), object_control.wxyz,
                    object_size.value, keypoint_scale.value, anchor_control.value, selected_tips,
                )
            except ValueError as exc:
                last_snapshot = {}
                summary.content = f"**输入错误：{exc}**\n修正后恢复观测显示。"
                vectors.visible = tip_cloud.visible = keypoint_cloud.visible = observation_frame.visible = False
                return
            snapshot["joint_names"] = list(joint_names)
            snapshot["joint_positions_raw"] = [cfg[n] for n in joint_names]
            last_snapshot = snapshot
            fields = snapshot["geometry_fields"]
            palm_position = np.asarray(fields["palm_pos"])
            observation_frame.position = palm_position
            observation_frame.wxyz = xyzw_to_wxyz(np.asarray(fields["palm_rot_xyzw"]))
            observation_frame.visible = True
            tip_points = np.asarray(snapshot["fingertip_positions"], dtype=np.float32).reshape(-1, 3)
            key_points = np.asarray(snapshot["object_keypoints"], dtype=np.float32)
            tip_cloud.points = tip_points
            keypoint_cloud.points = key_points
            tip_cloud.visible = keypoint_cloud.visible = True
            targets = np.concatenate([tip_points, key_points])
            vectors.points = np.stack([np.broadcast_to(palm_position, targets.shape), targets], axis=1).astype(np.float32)
            vectors.visible = show_vectors.value
            object_box.dimensions = tuple(snapshot["object_size"])
            summary.content = observation_markdown(snapshot)

    def reset_palm(_event):
        with lock:
            if saved_palm is None:
                local = initial_palm_transform(model, kind, parent_control.value)
            else:
                saved_parent, saved_local = saved_palm
                local = model.get_transform(saved_parent, parent_control.value) @ saved_local
            xyz.value = tuple(local[:3, 3])
            rpy.value = tuple(Rotation.from_matrix(local[:3, :3]).as_euler("xyz"))
            update()

    @confirm_palm.on_click
    def confirm_palm_to_file(_event):
        nonlocal saved_palm
        with lock:
            try:
                path = save_palm_frame(args.urdf, parent_control.value, xyz.value, rpy.value)
            except (OSError, ValueError, etree.XMLSyntaxError) as exc:
                save_status.content = f"**保存失败：{exc}**"
                return
            local = np.eye(4)
            local[:3, :3] = Rotation.from_euler("xyz", rpy.value).as_matrix()
            local[:3, 3] = xyz.value
            saved_palm = (parent_control.value, local)
            save_status.content = (f"**已保存到 `{path}`**\n\n"
                                   f"parent: `{parent_control.value}`；xyz: `{tuple(xyz.value)}`；rpy: `{tuple(rpy.value)}`")

    @print_snapshot.on_click
    def export_snapshot(_event):
        with lock:
            if last_snapshot:
                print(json.dumps(last_snapshot, indent=2, ensure_ascii=False), flush=True)
            else:
                print("Observation input invalid; fix inputs before exporting.", flush=True)

    @reset_object.on_click
    def restore_object(_event):
        with lock:
            object_control.position = (0.45, 0.0, 0.56)
            object_control.wxyz = (1.0, 0.0, 0.0, 0.0)
            object_size.value = (0.04, 0.04, 0.04)
            keypoint_scale.value = 1.5
            update()

    object_control.on_update(update)
    if tip_control is not None:
        tip_control.on_update(update)
    for handle in (*handles.values(), xyz, rpy, show_visual, show_collision,
                   object_size, keypoint_scale, anchor_control, show_vectors,
                   marker_size, position_step, rotation_step):
        handle.on_update(update)
    parent_control.on_update(reset_palm)
    reset.on_click(reset_palm)

    @server.on_client_connect
    def set_camera(client):
        client.camera.up_direction = (0.0, 0.0, 1.0)
        client.camera.position = (2.0, -2.0, 1.5)
        client.camera.look_at = (0.0, 0.0, 0.5)

    update()
    print(f"[palm-inspector] ready: {kind}, {len(joint_names)} joints", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
