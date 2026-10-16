"""Interactive Viser comparison of the two xArm7 + Wuji URDF observation frames.

This standalone tool preserves SimToolReal-style training geometry from
Interprior ``envs/simulation/utils/observation.py`` (no runtime dependency):

* the deployed ``palm_*`` source is ``link7`` after fixed-joint merging;
* fingertip pad centers are ``link4`` origins plus the calibrated offsets;
* ``*_rel_palm`` vectors are subtractions in the robot-base/world axes, not
  vectors rotated into palm-local axes.

The viewer also computes the same quantities against the actual URDF
``right_palm_link`` so the semantic error is visible without changing the
policy input contract.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


REPO_ROOT = Path(__file__).resolve().parents[1]
OLD_URDF = REPO_ROOT / "xarm7_wuji_90.urdf"
NEW_URDF = REPO_ROOT / "xarm7_wuji_direct_camera.urdf"
SIMTOOLREAL_URDF = REPO_ROOT / "assets/urdf/kuka_sharpa_description/iiwa14_left_sharpa_adjusted_restricted.urdf"
SHARPA_TIP_LINKS = tuple(f"left_{finger}_DP" for finger in ("index", "middle", "ring", "thumb", "pinky"))
SHARPA_START_ARM = np.array([-1.571, 1.571, 0.0, 1.376, 0.0, 1.485, 1.308])


def compute_sharpa_geometry(urdf: Any, joint_values: dict[str, float]):
    """Legacy env.py geometry, before observation noise/delay and clipping."""
    urdf.update_cfg(joint_values)
    wrist = np.asarray(urdf.get_transform("iiwa14_link_7"))
    center = transform_point(wrist, np.array([0.0, -0.02, 0.16]))
    tips = np.stack([
        transform_point(urdf.get_transform(name), np.array([0.02, 0.002, 0.0]))
        for name in SHARPA_TIP_LINKS
    ])
    return wrist, center, tips

JOINT_NAMES = tuple(
    [f"joint{i}" for i in range(1, 8)]
    + [
        f"right_finger{finger}_joint{joint}"
        for finger in range(1, 6)
        for joint in range(1, 5)
    ]
)
FINGERTIP_LINK_NAMES = tuple(f"right_finger{i}_link4" for i in range(1, 6))
FINGERTIP_OFFSETS_LINK4 = np.asarray(
    (
        (-0.0011048, 0.00023218, 0.0283),
        (-0.0010500, 0.00000000, 0.0274),
        (-0.0010491, 0.00000000, 0.0267),
        (-0.0010528, 0.00000000, 0.0267),
        (-0.0010446, 0.00001131, 0.0267),
    ),
    dtype=np.float64,
)
KEYPOINT_CORNERS = np.asarray(
    ((1, 1, 1), (1, 1, -1), (-1, -1, 1), (-1, -1, -1)),
    dtype=np.float64,
)
# Keep these paired with ROBOT_SETUP_PROFILES in
# envs/simulation/env_cfg/base_env_cfg.py. The two arm configurations require
# different joint poses to realize the same physical palm-down orientation.
OLD_START_ARM_POSE = np.asarray(
    (0.0, -0.548838, -0.000004, 0.096461, 0.0, 0.643490, 0.000005),
    dtype=np.float64,
)
NEW_START_ARM_POSE = np.asarray(
    (0.0, 0.0, 0.0, 0.0, np.pi, np.pi / 2.0, 0.0), dtype=np.float64
)


@dataclass(frozen=True)
class ObservationGeometry:
    """Critical observation blocks for one URDF at one configuration."""

    link7_transform: np.ndarray
    palm_transform: np.ndarray
    fingertip_positions: np.ndarray
    object_keypoints: np.ndarray
    current_palm_velocity: np.ndarray
    true_palm_velocity: np.ndarray

    @property
    def code_palm_position(self) -> np.ndarray:
        return self.link7_transform[:3, 3]

    @property
    def true_palm_position(self) -> np.ndarray:
        return self.palm_transform[:3, 3]

    @property
    def code_palm_quat_xyzw(self) -> np.ndarray:
        return rotation_matrix_to_xyzw(self.link7_transform[:3, :3])

    @property
    def true_palm_quat_xyzw(self) -> np.ndarray:
        return rotation_matrix_to_xyzw(self.palm_transform[:3, :3])

    def fingertip_rel(self, *, true_palm: bool) -> np.ndarray:
        anchor = self.true_palm_position if true_palm else self.code_palm_position
        return self.fingertip_positions - anchor

    def keypoints_rel(self, *, true_palm: bool) -> np.ndarray:
        anchor = self.true_palm_position if true_palm else self.code_palm_position
        return self.object_keypoints - anchor

    def critical_actor_block(self, *, true_palm: bool) -> np.ndarray:
        """Return palm_pos/rot + fingertip/keypoint blocks (34 dimensions)."""

        position = self.true_palm_position if true_palm else self.code_palm_position
        quaternion = self.true_palm_quat_xyzw if true_palm else self.code_palm_quat_xyzw
        return np.concatenate(
            (
                position,
                quaternion,
                self.fingertip_rel(true_palm=true_palm).reshape(-1),
                self.keypoints_rel(true_palm=true_palm).reshape(-1),
            )
        )


def rotation_matrix_to_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to a deterministic xyzw quaternion."""

    quaternion = Rotation.from_matrix(rotation).as_quat()
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def xyzw_to_wxyz(quaternion: np.ndarray) -> np.ndarray:
    return np.asarray(quaternion, dtype=np.float64)[[3, 0, 1, 2]]


def wxyz_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm <= 1.0e-12:
        raise ValueError("object quaternion must be non-zero")
    return Rotation.from_quat((quaternion / norm)[[1, 2, 3, 0]]).as_matrix()


def transform_point(transform: np.ndarray, point: np.ndarray) -> np.ndarray:
    return transform[:3, :3] @ point + transform[:3, 3]


def compute_observation_geometry(
    urdf: Any,
    *,
    joint_values: dict[str, float],
    object_position: np.ndarray,
    object_wxyz: np.ndarray,
    object_size: np.ndarray,
    keypoint_scale: float,
    link7_linear_velocity: np.ndarray,
    link7_angular_velocity: np.ndarray,
) -> ObservationGeometry:
    """Compute the observation geometry using the same axes as the policy."""

    urdf.update_cfg(joint_values)
    link7_transform = np.asarray(urdf.get_transform("link7"), dtype=np.float64)
    palm_transform = np.asarray(
        urdf.get_transform("right_palm_link"), dtype=np.float64
    )

    fingertip_positions = np.stack(
        [
            transform_point(
                np.asarray(urdf.get_transform(link_name), dtype=np.float64),
                offset,
            )
            for link_name, offset in zip(
                FINGERTIP_LINK_NAMES, FINGERTIP_OFFSETS_LINK4, strict=True
            )
        ]
    )

    object_rotation = wxyz_to_matrix(object_wxyz)
    local_keypoints = KEYPOINT_CORNERS * (0.5 * keypoint_scale * object_size)
    object_keypoints = (
        object_rotation @ local_keypoints.T
    ).T + np.asarray(object_position, dtype=np.float64)

    linear = np.asarray(link7_linear_velocity, dtype=np.float64)
    angular = np.asarray(link7_angular_velocity, dtype=np.float64)
    palm_offset_world = palm_transform[:3, 3] - link7_transform[:3, 3]
    current_palm_velocity = np.concatenate((linear, angular))
    true_palm_velocity = np.concatenate(
        (linear + np.cross(angular, palm_offset_world), angular)
    )
    return ObservationGeometry(
        link7_transform=link7_transform,
        palm_transform=palm_transform,
        fingertip_positions=fingertip_positions,
        object_keypoints=object_keypoints,
        current_palm_velocity=current_palm_velocity,
        true_palm_velocity=true_palm_velocity,
    )


def installation_transform(urdf: Any) -> np.ndarray:
    """Return link7_T_right_palm_link from the authored fixed-joint chain."""

    return np.asarray(
        urdf.get_transform("right_palm_link", "link7"), dtype=np.float64
    )


def _load_urdf(path: Path, *, load_meshes: bool) -> Any:
    import yourdfpy

    if not path.is_file():
        raise FileNotFoundError(path)
    return yourdfpy.URDF.load(
        str(path),
        build_scene_graph=True,
        build_collision_scene_graph=load_meshes,
        load_meshes=load_meshes,
        load_collision_meshes=load_meshes,
        filename_handler=partial(yourdfpy.filename_handler_magic, dir=path.parent),
    )


def _array_text(array: np.ndarray, precision: int = 4) -> str:
    return np.array2string(
        np.asarray(array),
        precision=precision,
        suppress_small=True,
        separator=", ",
        max_line_width=120,
    )


def _rotation_angle_degrees(first: np.ndarray, second: np.ndarray) -> float:
    relative = first[:3, :3].T @ second[:3, :3]
    return float(np.degrees(Rotation.from_matrix(relative).magnitude()))


def build_snapshot(
    old: ObservationGeometry,
    new: ObservationGeometry,
    *,
    true_palm: bool,
    old_installation: np.ndarray,
    new_installation: np.ndarray,
) -> dict[str, Any]:
    old_block = old.critical_actor_block(true_palm=true_palm)
    new_block = new.critical_actor_block(true_palm=true_palm)
    return {
        "anchor": "right_palm_link" if true_palm else "link7_current_code",
        "old_link7_T_palm": old_installation.tolist(),
        "new_link7_T_palm": new_installation.tolist(),
        "old": {
            "code_palm_pos": old.code_palm_position.tolist(),
            "code_palm_rot_xyzw": old.code_palm_quat_xyzw.tolist(),
            "true_palm_pos": old.true_palm_position.tolist(),
            "true_palm_rot_xyzw": old.true_palm_quat_xyzw.tolist(),
            "current_palm_vel": old.current_palm_velocity.tolist(),
            "true_palm_vel": old.true_palm_velocity.tolist(),
            "critical_34d": old_block.tolist(),
        },
        "new": {
            "code_palm_pos": new.code_palm_position.tolist(),
            "code_palm_rot_xyzw": new.code_palm_quat_xyzw.tolist(),
            "true_palm_pos": new.true_palm_position.tolist(),
            "true_palm_rot_xyzw": new.true_palm_quat_xyzw.tolist(),
            "current_palm_vel": new.current_palm_velocity.tolist(),
            "true_palm_vel": new.true_palm_velocity.tolist(),
            "critical_34d": new_block.tolist(),
        },
        "new_minus_old_critical_34d": (new_block - old_block).tolist(),
    }


def snapshot_markdown(snapshot: dict[str, Any]) -> str:
    old = snapshot["old"]
    new = snapshot["new"]
    delta = np.asarray(snapshot["new_minus_old_critical_34d"])
    return f"""
### Live observation comparison

Anchor for relative vectors: **`{snapshot['anchor']}`**

| value | old side mount | new direct mount |
|---|---|---|
| code `palm_pos` (link7) | `{_array_text(np.asarray(old['code_palm_pos']))}` | `{_array_text(np.asarray(new['code_palm_pos']))}` |
| true palm position | `{_array_text(np.asarray(old['true_palm_pos']))}` | `{_array_text(np.asarray(new['true_palm_pos']))}` |
| code `palm_rot` xyzw | `{_array_text(np.asarray(old['code_palm_rot_xyzw']))}` | `{_array_text(np.asarray(new['code_palm_rot_xyzw']))}` |
| true palm rotation xyzw | `{_array_text(np.asarray(old['true_palm_rot_xyzw']))}` | `{_array_text(np.asarray(new['true_palm_rot_xyzw']))}` |
| current `palm_vel` | `{_array_text(np.asarray(old['current_palm_vel']))}` | `{_array_text(np.asarray(new['current_palm_vel']))}` |
| true offset-point velocity | `{_array_text(np.asarray(old['true_palm_vel']))}` | `{_array_text(np.asarray(new['true_palm_vel']))}` |

Diagnostic concat (not one contiguous actor slice):
`critical_34d = palm_pos[3] + palm_rot_xyzw[4] + fingertip_rel[15] + keypoint_rel[12]`

```text
old:   {_array_text(np.asarray(old['critical_34d']))}
new:   {_array_text(np.asarray(new['critical_34d']))}
delta: {_array_text(delta)}
```

Delta norms: `L2={np.linalg.norm(delta):.5f}`, `max_abs={np.max(np.abs(delta)):.5f}`.
Quaternion signs are canonicalized to `qw >= 0`; `q` and `-q` represent the same rotation.
"""


def run_viewer(args: argparse.Namespace) -> None:
    import viser
    from viser import transforms as viser_tf

    class UrdfView:
        """Minimal visual/collision URDF renderer without viser.extras imports."""

        def __init__(
            self,
            urdf: Any,
            *,
            root_node_name: str,
            color: tuple[int, int, int],
            collision: bool,
        ) -> None:
            self.urdf = urdf
            self.collision = collision
            self.scene = urdf.collision_scene if collision else urdf.scene
            if self.scene is None:
                raise ValueError("requested URDF scene graph was not loaded")
            self.joint_frames = []
            for joint in urdf.joint_map.values():
                self.joint_frames.append(
                    server.scene.add_frame(
                        self._node_name(joint.child, root_node_name), show_axes=False
                    )
                )
            for link_name, mesh in self.scene.geometry.items():
                parent = self.scene.graph.transforms.parents[link_name]
                transform = self.scene.graph.get(
                    frame_to=link_name, frame_from=parent
                )[0]
                mesh = mesh.copy()
                mesh.apply_transform(transform)
                server.scene.add_mesh_simple(
                    self._node_name(link_name, root_node_name),
                    mesh.vertices,
                    mesh.faces,
                    color=color,
                )

        def _node_name(self, frame_name: str, root_node_name: str) -> str:
            frames = []
            while frame_name != self.scene.graph.base_frame:
                frames.append(frame_name)
                frame_name = self.scene.graph.transforms.parents[frame_name]
            frames.append(root_node_name)
            return "/".join(frames[::-1])

        def update_cfg(self, configuration: np.ndarray) -> None:
            self.urdf.update_cfg(configuration)
            for joint, frame in zip(
                self.urdf.joint_map.values(), self.joint_frames, strict=True
            ):
                transform = self.scene.graph.get(
                    frame_to=joint.child, frame_from=joint.parent
                )[0]
                frame.wxyz = viser_tf.SO3.from_matrix(transform[:3, :3]).wxyz
                frame.position = transform[:3, 3]

        def get_actuated_joint_limits(
            self,
        ) -> dict[str, tuple[float | None, float | None]]:
            limits = {}
            for name, joint in zip(
                self.urdf.actuated_joint_names,
                self.urdf.actuated_joints,
                strict=True,
            ):
                limits[name] = (
                    (-np.pi, np.pi)
                    if joint.limit is None
                    else (joint.limit.lower, joint.limit.upper)
                )
            return limits

    old_urdf = _load_urdf(args.old_urdf, load_meshes=True)
    new_urdf = _load_urdf(args.new_urdf, load_meshes=True)
    sharpa_urdf = _load_urdf(args.simtoolreal_urdf, load_meshes=True)
    assert len(sharpa_urdf.actuated_joint_names) == 29
    assert set(old_urdf.actuated_joint_names) == set(JOINT_NAMES)
    assert set(new_urdf.actuated_joint_names) == set(JOINT_NAMES)

    server = viser.ViserServer(host=args.host, port=args.port)
    server.scene.set_up_direction("+z")
    server.scene.add_frame("/simtoolreal", position=(0.0, -1.65, 0.0), axes_length=0.12)
    server.scene.add_label("/simtoolreal/label", "SimToolReal · Kuka + left Sharpa · 29 DOF", position=(0.0, 0.0, 1.25))
    sharpa_visual_root = server.scene.add_frame("/simtoolreal/visual_model", show_axes=False)
    sharpa_collision_root = server.scene.add_frame("/simtoolreal/collision_model", show_axes=False, visible=False)
    sharpa_robot = UrdfView(sharpa_urdf, root_node_name="/simtoolreal/visual_model", color=(155, 110, 220), collision=False)
    sharpa_collision = UrdfView(sharpa_urdf, root_node_name="/simtoolreal/collision_model", color=(255, 70, 70), collision=True)
    sharpa_wrist = server.scene.add_frame("/simtoolreal/wrist", axes_length=0.10)
    sharpa_center = server.scene.add_frame("/simtoolreal/observation_palm", axes_length=0.08)
    sharpa_tips = server.scene.add_point_cloud("/simtoolreal/tips", points=np.zeros((5, 3)), colors=(40, 220, 255), point_size=0.018)
    sharpa_vectors = [server.scene.add_spline_catmull_rom(f"/simtoolreal/tip_vector_{i}", positions=np.zeros((2, 3)), color=(40, 220, 255), line_width=3, segments=1) for i in range(5)]
    sharpa_keypoints = server.scene.add_point_cloud("/simtoolreal/keypoints", points=np.zeros((4, 3)), colors=(255, 220, 40), point_size=0.018)
    sharpa_keypoint_vectors = [server.scene.add_spline_catmull_rom(f"/simtoolreal/keypoint_vector_{i}", positions=np.zeros((2, 3)), color=(255, 220, 40), line_width=3, segments=1) for i in range(4)]
    kuka_object_control = server.scene.add_transform_controls(
        "/simtoolreal/object_control", scale=0.12,
        position=(0.45, 0.0, 0.56), wxyz=(1.0, 0.0, 0.0, 0.0),
        translation_limits=((0.1, 0.8), (-0.4, 0.4), (0.2, 1.0)),
        rotation_limits=((-np.pi, np.pi),) * 3,
    )
    sharpa_object = server.scene.add_box("/simtoolreal/object_control/object", dimensions=(0.04, 0.04, 0.04), color=(255, 210, 50))
    sharpa_handles = {}
    sharpa_limits = list(sharpa_robot.get_actuated_joint_limits().items())
    with server.gui.add_folder("Kuka arm control · 7 joints (rad)", expand_by_default=True):
        for index, (name, (lower, upper)) in enumerate(sharpa_limits[:7]):
            sharpa_handles[name] = server.gui.add_slider(name, min=float(lower), max=float(upper), step=0.01, initial_value=float(np.clip(SHARPA_START_ARM[index], lower, upper)))
        reset_kuka = server.gui.add_button("Reset Kuka + Sharpa start pose")
        zero_kuka = server.gui.add_button("Zero Kuka + Sharpa joints")
        server.gui.add_markdown("Kuka 旁的黄色物体控件可拖动和旋转，与两套 Wuji 共享物体姿态。")
    with server.gui.add_folder("Sharpa hand control · 22 joints (rad)", expand_by_default=False):
        for name, (lower, upper) in sharpa_limits[7:]:
            sharpa_handles[name] = server.gui.add_slider(name, min=float(lower), max=float(upper), step=0.01, initial_value=float(np.clip(0.0, lower, upper)))
    with server.gui.add_folder("SimToolReal 资产与 observation 对照", expand_by_default=False):
        server.gui.add_markdown((REPO_ROOT / "docs/asset_observation_comparison.md").read_text())
        sharpa_summary = server.gui.add_markdown("")
    server.scene.add_grid(
        "/ground",
        width=2.4,
        height=2.4,
        position=(0.0, 0.0, 0.0),
        cell_size=0.1,
        cell_thickness=0.5,
        section_size=0.5,
        section_thickness=1.0,
        plane="xy",
    )

    server.scene.add_frame(
        "/old", show_axes=True, axes_length=0.12, position=(0.0, -0.55, 0.0)
    )
    server.scene.add_frame(
        "/new", show_axes=True, axes_length=0.12, position=(0.0, 0.55, 0.0)
    )
    server.scene.add_label(
        "/old/label", "OLD · side mount", position=(0.0, 0.0, 0.95)
    )
    server.scene.add_label(
        "/new/label", "NEW · direct camera mount", position=(0.0, 0.0, 0.95)
    )

    old_visual_root = server.scene.add_frame("/old/visual_model", show_axes=False)
    new_visual_root = server.scene.add_frame("/new/visual_model", show_axes=False)
    old_collision_root = server.scene.add_frame(
        "/old/collision_model", show_axes=False, visible=False
    )
    new_collision_root = server.scene.add_frame(
        "/new/collision_model", show_axes=False, visible=False
    )
    old_robot = UrdfView(
        old_urdf,
        root_node_name="/old/visual_model",
        color=(70, 130, 255),
        collision=False,
    )
    new_robot = UrdfView(
        new_urdf,
        root_node_name="/new/visual_model",
        color=(255, 155, 60),
        collision=False,
    )
    old_collision_robot = UrdfView(
        old_urdf,
        root_node_name="/old/collision_model",
        color=(255, 70, 70),
        collision=True,
    )
    new_collision_robot = UrdfView(
        new_urdf,
        root_node_name="/new/collision_model",
        color=(255, 70, 70),
        collision=True,
    )

    object_control = server.scene.add_transform_controls(
        "/new/object_control",
        scale=0.12,
        position=(0.45, 0.0, 0.56),
        wxyz=(1.0, 0.0, 0.0, 0.0),
        translation_limits=((0.1, 0.8), (-0.4, 0.4), (0.2, 1.0)),
        rotation_limits=((-np.pi, np.pi),) * 3,
    )
    new_object_box = server.scene.add_box(
        "/new/object_control/object",
        color=(255, 210, 50),
        dimensions=(0.04, 0.04, 0.04),
    )
    old_object_box = server.scene.add_box(
        "/old/object",
        color=(255, 210, 50),
        dimensions=(0.04, 0.04, 0.04),
    )

    overlay_handles: dict[str, dict[str, Any]] = {}
    for name in ("old", "new"):
        overlay_handles[name] = {
            "link7": server.scene.add_frame(
                f"/{name}/obs/link7",
                axes_length=0.10,
                axes_radius=0.004,
            ),
            "link7_point": server.scene.add_point_cloud(
                f"/{name}/obs/link7_marker",
                points=np.zeros((1, 3)),
                colors=(255, 70, 70),
                point_size=0.025,
                point_shape="circle",
            ),
            "palm": server.scene.add_frame(
                f"/{name}/obs/right_palm_link",
                axes_length=0.10,
                axes_radius=0.004,
            ),
            "palm_point": server.scene.add_point_cloud(
                f"/{name}/obs/right_palm_link_marker",
                points=np.zeros((1, 3)),
                colors=(70, 255, 100),
                point_size=0.025,
                point_shape="circle",
            ),
            "tips": server.scene.add_point_cloud(
                f"/{name}/obs/fingertips",
                points=np.zeros((5, 3)),
                colors=(40, 220, 255),
                point_size=0.018,
                point_shape="circle",
            ),
            "keypoints": server.scene.add_point_cloud(
                f"/{name}/obs/object_keypoints",
                points=np.zeros((4, 3)),
                colors=(255, 220, 40),
                point_size=0.018,
                point_shape="circle",
            ),
            "tip_vectors": [
                server.scene.add_spline_catmull_rom(
                    f"/{name}/obs/fingertip_rel_palm_{index}",
                    positions=np.zeros((2, 3)),
                    color=(40, 220, 255),
                    line_width=3.0,
                    segments=1,
                )
                for index in range(5)
            ],
            "keypoint_vectors": [
                server.scene.add_spline_catmull_rom(
                    f"/{name}/obs/keypoint_rel_palm_{index}",
                    positions=np.zeros((2, 3)),
                    color=(255, 220, 40),
                    line_width=3.0,
                    segments=1,
                )
                for index in range(4)
            ],
        }

    old_install = installation_transform(old_urdf)
    new_install = installation_transform(new_urdf)
    install_summary = server.gui.add_markdown(
        f"""
## xArm7 + Wuji observation inspector

The robots start from their own paired `ROBOT_SETUP_PROFILES` arm pose, which
gives approximately the same physical palm orientation. Arm joints are edited
independently; the 20 internal hand joints and object pose are shared. The
robots are separated only for display.

The base actor observation is **134D**. With the RL-trajectory surface feature enabled it is **174D**, with these slices:
`joint_pos[0:27]`, `joint_vel[27:54]`, `prev_action_targets[54:81]`,
`palm_pos[81:84]`, `palm_rot[84:88]`, `object_rot[88:92]`,
`fingertip_pos_rel_palm[92:107]`, `keypoints_rel_palm[107:119]`,
`keypoints_rel_goal[119:131]`, `object_scales[131:134]`, and
`hand_keypoint_to_surface_vec[134:174]`. `palm_vel` is critic-only.

| authored fixed chain | translation (m) | rotation angle |
|---|---|---|
| old `link7_T_palm` | `{_array_text(old_install[:3, 3])}` | `{np.degrees(Rotation.from_matrix(old_install[:3, :3]).magnitude()):.3f}°` |
| new `link7_T_palm` | `{_array_text(new_install[:3, 3])}` | `{np.degrees(Rotation.from_matrix(new_install[:3, :3]).magnitude()):.3f}°` |

Drag the yellow object gizmo beside the **new** robot. It is mirrored into the
old robot's base frame. Red markers identify `link7`; green markers identify
the authored `right_palm_link`. Both frames also show their local XYZ axes.
"""
    )
    del install_summary

    virtual_frames = {}
    palm_spheres = {}
    with server.gui.add_folder("Virtual palm markers · 虚拟掌心球", expand_by_default=True):
        palm_marker_size = server.gui.add_dropdown(
            "球体大小（全部掌心）",
            ("大球 · 半径 25 mm", "小球 · 半径 4 mm", "隐藏球体"),
            initial_value="大球 · 半径 25 mm",
        )
        show_virtual_axes = server.gui.add_checkbox("显示虚拟掌心坐标轴", initial_value=True)
        server.gui.add_markdown("紫色球：两版 Wuji；绿色球：Sharpa 历史虚拟掌心。球心才是参考点，半径仅用于显示，不代表碰撞或接触范围。小球接近坐标轴线宽。")
    for name in ("old", "new", "simtoolreal"):
        color = (70, 255, 100) if name == "simtoolreal" else (230, 80, 255)
        palm_spheres[name] = {
            size: server.scene.add_icosphere(
                f"/{name}/virtual_palm_sphere_{size}", radius=radius,
                color=color, visible=size == "large",
            )
            for size, radius in (("large", 0.025), ("small", 0.004))
        }
    virtual_controls = {}
    virtual_initial = {}
    with server.gui.add_folder("Virtual palm tuning · 手掌局部坐标", expand_by_default=True):
        server.gui.add_markdown("紫色球心为 URDF palm_center，坐标轴跟随 right_palm_link。X 已调为 +4 mm，Y/Z 保留掌体包围盒中心，并非接触面标定点。单位 m，步长 1 mm。调整仅用于预览；下方 XML 可复制到对应 URDF。训练 observation 尚未切换。")
        for name, model in (("old", old_urdf), ("new", new_urdf)):
            virtual_initial[name] = np.array(model.get_transform("palm_center", "right_palm_link"), copy=True)
            virtual_controls[name] = server.gui.add_vector3(f"{name} palm xyz (m)", initial_value=tuple(virtual_initial[name][:3, 3]), step=0.001)
            virtual_frames[name] = server.scene.add_frame(f"/{name}/virtual_palm", axes_length=0.065, axes_radius=0.003)
        reset_virtual = server.gui.add_button("Reset virtual palms to URDF")
        virtual_summary = server.gui.add_markdown("")

    anchor_mode = server.gui.add_dropdown(
        "Relative-vector anchor",
        ("current code: link7", "semantic palm: right_palm_link", "virtual palm: palm_center"),
        initial_value="virtual palm: palm_center",
    )
    show_visual = server.gui.add_checkbox("Show visual meshes", initial_value=True)
    show_collision = server.gui.add_checkbox(
        "Show collision meshes", initial_value=False
    )
    show_link7 = server.gui.add_checkbox("Show link7 axes", initial_value=True)
    show_palm = server.gui.add_checkbox("Show true palm axes", initial_value=True)
    show_tips = server.gui.add_checkbox("Show fingertip points/vectors", initial_value=True)
    show_keypoints = server.gui.add_checkbox(
        "Show object keypoints/vectors", initial_value=True
    )

    with server.gui.add_folder("Object/keypoint geometry"):
        object_size = server.gui.add_vector3(
            "Object size xyz (m)", initial_value=(0.04, 0.04, 0.04), step=0.005
        )
        keypoint_scale = server.gui.add_number(
            "Keypoint scale", initial_value=1.5, min=0.1, max=4.0, step=0.05
        )

    with server.gui.add_folder("Velocity thought experiment", expand_by_default=False):
        linear_velocity = server.gui.add_vector3(
            "link7 linear velocity", initial_value=(0.0, 0.0, 0.0), step=0.01
        )
        angular_velocity = server.gui.add_vector3(
            "link7 angular velocity", initial_value=(0.0, 0.0, 0.0), step=0.1
        )

    old_arm_handles: dict[str, Any] = {}
    new_arm_handles: dict[str, Any] = {}
    hand_handles: dict[str, Any] = {}
    old_limits = old_robot.get_actuated_joint_limits()
    new_limits = new_robot.get_actuated_joint_limits()
    with server.gui.add_folder("Old arm joints", expand_by_default=True):
        for index, joint_name in enumerate(JOINT_NAMES[:7]):
            lower, upper = old_limits[joint_name]
            lower = -2.0 * np.pi if lower is None else lower
            upper = 2.0 * np.pi if upper is None else upper
            old_arm_handles[joint_name] = server.gui.add_slider(
                joint_name,
                min=float(lower),
                max=float(upper),
                step=0.01,
                initial_value=float(
                    np.clip(OLD_START_ARM_POSE[index], lower, upper)
                ),
            )
    with server.gui.add_folder("New arm joints", expand_by_default=True):
        for index, joint_name in enumerate(JOINT_NAMES[:7]):
            lower, upper = new_limits[joint_name]
            lower = -2.0 * np.pi if lower is None else lower
            upper = 2.0 * np.pi if upper is None else upper
            new_arm_handles[joint_name] = server.gui.add_slider(
                joint_name,
                min=float(lower),
                max=float(upper),
                step=0.01,
                initial_value=float(
                    np.clip(NEW_START_ARM_POSE[index], lower, upper)
                ),
            )
    with server.gui.add_folder("Shared hand joints", expand_by_default=False):
        for joint_name in JOINT_NAMES[7:]:
            old_lower, old_upper = old_limits[joint_name]
            new_lower, new_upper = new_limits[joint_name]
            lower = max(
                -2.0 * np.pi if old_lower is None else old_lower,
                -2.0 * np.pi if new_lower is None else new_lower,
            )
            upper = min(
                2.0 * np.pi if old_upper is None else old_upper,
                2.0 * np.pi if new_upper is None else new_upper,
            )
            hand_handles[joint_name] = server.gui.add_slider(
                joint_name,
                min=float(lower),
                max=float(upper),
                step=0.01,
                initial_value=float(np.clip(0.0, lower, upper)),
            )

    live_summary = server.gui.add_markdown("Waiting for first update...")
    print_snapshot = server.gui.add_button("Print snapshot JSON")
    reset_profiles = server.gui.add_button("Reset to matched setup start poses")
    copy_old_to_new = server.gui.add_button("Copy old arm joints to new")
    copy_new_to_old = server.gui.add_button("Copy new arm joints to old")
    reset_zero = server.gui.add_button("Reset both arms and hand to zero")

    state_lock = threading.Lock()
    last_snapshot: dict[str, Any] = {}

    def update_scene() -> None:
        nonlocal last_snapshot
        with state_lock, server.atomic():
            hand_values = {
                name: float(handle.value) for name, handle in hand_handles.items()
            }
            old_joint_values = {
                **{
                    name: float(handle.value)
                    for name, handle in old_arm_handles.items()
                },
                **hand_values,
            }
            new_joint_values = {
                **{
                    name: float(handle.value)
                    for name, handle in new_arm_handles.items()
                },
                **hand_values,
            }
            position = np.asarray(object_control.position, dtype=np.float64)
            object_wxyz = np.asarray(object_control.wxyz, dtype=np.float64)
            size = np.asarray(object_size.value, dtype=np.float64)
            if np.any(size <= 0.0):
                raise ValueError("object dimensions must be positive")

            common_kwargs = {
                "object_position": position,
                "object_wxyz": object_wxyz,
                "object_size": size,
                "keypoint_scale": float(keypoint_scale.value),
                "link7_linear_velocity": np.asarray(linear_velocity.value),
                "link7_angular_velocity": np.asarray(angular_velocity.value),
            }
            old_obs = compute_observation_geometry(
                old_urdf, joint_values=old_joint_values, **common_kwargs
            )
            new_obs = compute_observation_geometry(
                new_urdf, joint_values=new_joint_values, **common_kwargs
            )
            virtual_world = {}
            virtual_report = []
            for name, obs in (("old", old_obs), ("new", new_obs)):
                local = virtual_initial[name].copy()
                local[:3, 3] = virtual_controls[name].value
                world = obs.palm_transform @ local
                virtual_world[name] = world
                virtual_frames[name].position = world[:3, 3]
                virtual_frames[name].wxyz = xyzw_to_wxyz(rotation_matrix_to_xyzw(world[:3, :3]))
                for sphere in palm_spheres[name].values():
                    sphere.position = world[:3, 3]
                xyz = " ".join(f"{v:.7f}" for v in local[:3, 3])
                rpy = " ".join(f"{v:.7f}" for v in Rotation.from_matrix(local[:3, :3]).as_euler("xyz"))
                virtual_report.append(f'{name}: right_palm_link → palm_center\n```xml\n<origin xyz="{xyz}" rpy="{rpy}"/>\n```\nworld/base xyz: `{_array_text(world[:3, 3])}`')
            virtual_summary.content = "\n\n".join(virtual_report)
            sharpa_values = {name: float(handle.value) for name, handle in sharpa_handles.items()}
            wrist, center, tips = compute_sharpa_geometry(sharpa_urdf, sharpa_values)
            sharpa_cfg = np.array([sharpa_values[name] for name in sharpa_urdf.actuated_joint_names])
            sharpa_robot.update_cfg(sharpa_cfg)
            sharpa_collision.update_cfg(sharpa_cfg)
            sharpa_wrist.position = wrist[:3, 3]
            sharpa_wrist.wxyz = xyzw_to_wxyz(rotation_matrix_to_xyzw(wrist[:3, :3]))
            sharpa_center.position = center
            for sphere in palm_spheres["simtoolreal"].values():
                sphere.position = center
            sharpa_center.wxyz = sharpa_wrist.wxyz
            sharpa_tips.points = tips.astype(np.float32)
            for handle, tip in zip(sharpa_vectors, tips, strict=True):
                handle.positions = (tuple(center), tuple(tip))
            sharpa_keypoints.points = new_obs.object_keypoints.astype(np.float32)
            for handle, point in zip(sharpa_keypoint_vectors, new_obs.object_keypoints, strict=True):
                handle.positions = (tuple(center), tuple(point))
            kuka_object_control.position = position
            kuka_object_control.wxyz = object_wxyz
            sharpa_object.dimensions = size
            sharpa_summary.content = f"SimToolReal palm_pos: `{_array_text(center)}` m; palm_rot xyzw: `{_array_text(rotation_matrix_to_xyzw(wrist[:3, :3]))}`.\n\n指尖顺序 index / middle / ring / thumb / pinky；相对 palm（m）：\n```\n{np.array2string(tips - center, precision=4)}\n```\n共享物体用于几何对照；Sharpa 始终使用历史 offset palm，Wuji anchor 切换不作用于 Sharpa。"
            old_robot.update_cfg(
                np.asarray(
                    [old_joint_values[name] for name in old_urdf.actuated_joint_names]
                )
            )
            new_robot.update_cfg(
                np.asarray(
                    [new_joint_values[name] for name in new_urdf.actuated_joint_names]
                )
            )
            old_collision_robot.update_cfg(
                np.asarray(
                    [old_joint_values[name] for name in old_urdf.actuated_joint_names]
                )
            )
            new_collision_robot.update_cfg(
                np.asarray(
                    [new_joint_values[name] for name in new_urdf.actuated_joint_names]
                )
            )

            true_palm = anchor_mode.value.startswith("semantic")
            for name, obs in (("old", old_obs), ("new", new_obs)):
                handles = overlay_handles[name]
                handles["link7"].position = obs.code_palm_position
                handles["link7"].wxyz = xyzw_to_wxyz(obs.code_palm_quat_xyzw)
                handles["link7_point"].points = obs.code_palm_position[None].astype(
                    np.float32
                )
                handles["palm"].position = obs.true_palm_position
                handles["palm"].wxyz = xyzw_to_wxyz(obs.true_palm_quat_xyzw)
                handles["palm_point"].points = obs.true_palm_position[None].astype(
                    np.float32
                )
                handles["tips"].points = obs.fingertip_positions.astype(np.float32)
                handles["keypoints"].points = obs.object_keypoints.astype(np.float32)
                anchor = obs.true_palm_position if true_palm else obs.code_palm_position
                if anchor_mode.value.startswith("virtual"):
                    anchor = virtual_world[name][:3, 3]
                for handle, endpoint in zip(
                    handles["tip_vectors"], obs.fingertip_positions, strict=True
                ):
                    handle.positions = (tuple(anchor), tuple(endpoint))
                for handle, endpoint in zip(
                    handles["keypoint_vectors"], obs.object_keypoints, strict=True
                ):
                    handle.positions = (tuple(anchor), tuple(endpoint))

            old_object_box.position = position
            old_object_box.wxyz = object_wxyz
            old_object_box.dimensions = size
            new_object_box.dimensions = size

            last_snapshot = build_snapshot(
                old_obs,
                new_obs,
                true_palm=true_palm,
                old_installation=old_install,
                new_installation=new_install,
            )
            last_snapshot["old_joint_values"] = old_joint_values
            last_snapshot["vector_anchor"] = anchor_mode.value
            last_snapshot["virtual_palms"] = {
                name: {"local_xyz": list(virtual_controls[name].value), "world_transform": virtual_world[name].tolist()}
                for name in ("old", "new")
            }
            last_snapshot["simtoolreal"] = {"joint_values": sharpa_values, "palm_pos": center.tolist(), "palm_rot_xyzw": rotation_matrix_to_xyzw(wrist[:3, :3]).tolist(), "fingertip_pos_rel_palm": (tips - center).tolist(), "keypoints_rel_palm": (new_obs.object_keypoints - center).tolist()}
            last_snapshot["new_joint_values"] = new_joint_values
            last_snapshot["object_pose_wxyz"] = np.r_[position, object_wxyz].tolist()
            last_snapshot["object_size"] = size.tolist()
            last_snapshot["true_palm_rotation_delta_degrees"] = _rotation_angle_degrees(
                old_obs.palm_transform, new_obs.palm_transform
            )
            live_summary.content = snapshot_markdown(last_snapshot)
            if anchor_mode.value.startswith("virtual"):
                live_summary.content = "当前连线使用紫色 palm_center。以下保留现有 link7 policy 的数值作为对照；调试参数尚未应用到训练。\n\n" + live_summary.content

    def update_visibility() -> None:
        for spheres in palm_spheres.values():
            spheres["large"].visible = palm_marker_size.value.startswith("大球")
            spheres["small"].visible = palm_marker_size.value.startswith("小球")
        for frame in virtual_frames.values():
            frame.visible = bool(show_virtual_axes.value)
        sharpa_visual_root.visible = bool(show_visual.value)
        sharpa_collision_root.visible = bool(show_collision.value)
        sharpa_wrist.visible = bool(show_link7.value)
        sharpa_center.visible = bool(show_virtual_axes.value)
        sharpa_tips.visible = bool(show_tips.value)
        for handle in sharpa_vectors:
            handle.visible = bool(show_tips.value)
        sharpa_keypoints.visible = bool(show_keypoints.value)
        for handle in sharpa_keypoint_vectors:
            handle.visible = bool(show_keypoints.value)
        old_visual_root.visible = bool(show_visual.value)
        new_visual_root.visible = bool(show_visual.value)
        old_collision_root.visible = bool(show_collision.value)
        new_collision_root.visible = bool(show_collision.value)
        for handles in overlay_handles.values():
            handles["link7"].visible = bool(show_link7.value)
            handles["link7_point"].visible = bool(show_link7.value)
            handles["palm"].visible = bool(show_palm.value)
            handles["palm_point"].visible = bool(show_palm.value)
            handles["tips"].visible = bool(show_tips.value)
            handles["keypoints"].visible = bool(show_keypoints.value)
            for handle in handles["tip_vectors"]:
                handle.visible = bool(show_tips.value)
            for handle in handles["keypoint_vectors"]:
                handle.visible = bool(show_keypoints.value)

    for handle in (
        anchor_mode,
        object_size,
        keypoint_scale,
        linear_velocity,
        angular_velocity,
        *old_arm_handles.values(),
        *new_arm_handles.values(),
        *hand_handles.values(),
        *sharpa_handles.values(),
        *virtual_controls.values(),
    ):
        handle.on_update(lambda _event: update_scene())
    object_control.on_update(lambda _event: update_scene())

    @kuka_object_control.on_update
    def _move_shared_object_from_kuka(_event: Any) -> None:
        if (np.allclose(object_control.position, kuka_object_control.position)
                and np.allclose(object_control.wxyz, kuka_object_control.wxyz)):
            return
        with server.atomic():
            object_control.position = kuka_object_control.position
            object_control.wxyz = kuka_object_control.wxyz
        update_scene()

    def set_kuka_joints(arm: np.ndarray) -> None:
        with server.atomic():
            for index, (name, (lower, upper)) in enumerate(sharpa_limits):
                value = arm[index] if index < 7 else 0.0
                sharpa_handles[name].value = float(np.clip(value, lower, upper))
        update_scene()

    reset_kuka.on_click(lambda _event: set_kuka_joints(SHARPA_START_ARM))
    zero_kuka.on_click(lambda _event: set_kuka_joints(np.zeros(7)))

    @reset_virtual.on_click
    def _reset_virtual(_event: Any) -> None:
        for name in ("old", "new"):
            virtual_controls[name].value = tuple(virtual_initial[name][:3, 3])
        update_scene()
    for handle in (
        show_visual,
        palm_marker_size,
        show_virtual_axes,
        show_collision,
        show_link7,
        show_palm,
        show_tips,
        show_keypoints,
    ):
        handle.on_update(lambda _event: update_visibility())

    @print_snapshot.on_click
    def _print_snapshot(_event: Any) -> None:
        print(json.dumps(last_snapshot, indent=2), flush=True)

    def set_joint_values(
        old_arm: np.ndarray, new_arm: np.ndarray, hand: np.ndarray
    ) -> None:
        for name, value in zip(JOINT_NAMES[:7], old_arm, strict=True):
            old_arm_handles[name].value = float(value)
        for name, value in zip(JOINT_NAMES[:7], new_arm, strict=True):
            new_arm_handles[name].value = float(value)
        for name, value in zip(JOINT_NAMES[7:], hand, strict=True):
            hand_handles[name].value = float(value)
        update_scene()

    @reset_profiles.on_click
    def _reset_profiles(_event: Any) -> None:
        set_joint_values(OLD_START_ARM_POSE, NEW_START_ARM_POSE, np.zeros(20))

    @copy_old_to_new.on_click
    def _copy_old_to_new(_event: Any) -> None:
        for name in JOINT_NAMES[:7]:
            new_arm_handles[name].value = old_arm_handles[name].value
        update_scene()

    @copy_new_to_old.on_click
    def _copy_new_to_old(_event: Any) -> None:
        for name in JOINT_NAMES[:7]:
            old_arm_handles[name].value = new_arm_handles[name].value
        update_scene()

    @reset_zero.on_click
    def _reset_zero(_event: Any) -> None:
        set_joint_values(np.zeros(7), np.zeros(7), np.zeros(20))

    @server.on_client_connect
    def _set_camera(client: Any) -> None:
        client.camera.up_direction = (0.0, 0.0, 1.0)
        client.camera.position = (2.8, -3.4, 2.0)
        client.camera.look_at = (0.25, -0.55, 0.48)

    update_visibility()
    update_scene()
    print(
        "[urdf-observation-viewer] ready; open the Viser URL printed above. "
        "Press Ctrl-C to stop.",
        flush=True,
    )
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[urdf-observation-viewer] stopped", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--old-urdf", type=Path, default=OLD_URDF)
    parser.add_argument("--new-urdf", type=Path, default=NEW_URDF)
    parser.add_argument("--simtoolreal-urdf", type=Path, default=SIMTOOLREAL_URDF)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    return parser.parse_args()


def main() -> None:
    run_viewer(parse_args())


if __name__ == "__main__":
    main()
