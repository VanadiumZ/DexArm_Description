import numpy as np

from tools import urdf_observation_viewer as viewer


def test_sharpa_legacy_observation_offsets() -> None:
    model = viewer._load_urdf(viewer.SIMTOOLREAL_URDF, load_meshes=False)
    assert len(model.actuated_joint_names) == 29
    values = dict(zip(model.actuated_joint_names, np.r_[viewer.SHARPA_START_ARM, np.zeros(22)], strict=True))
    wrist, center, tips = viewer.compute_sharpa_geometry(model, values)
    np.testing.assert_allclose(wrist[:3, :3].T @ (center - wrist[:3, 3]), [0, -0.02, 0.16], atol=1e-10)
    assert tips.shape == (5, 3)
    for name, tip in zip(viewer.SHARPA_TIP_LINKS, tips, strict=True):
        transform = model.get_transform(name)
        np.testing.assert_allclose(transform[:3, :3].T @ (tip - transform[:3, 3]), [0.02, 0.002, 0], atol=1e-10)


def test_virtual_palm_frame_on_both_wuji_mounts() -> None:
    for path in (viewer.OLD_URDF, viewer.NEW_URDF):
        model = viewer._load_urdf(path, load_meshes=False)
        assert set(model.actuated_joint_names) == set(viewer.JOINT_NAMES)
        local = model.get_transform("palm_center", "right_palm_link")
        np.testing.assert_allclose(local[:3, 3], [0.004, -0.0023529, 0.0434161])
        np.testing.assert_allclose(local[:3, :3], np.eye(3))
        model.update_cfg(dict(zip(viewer.JOINT_NAMES[:7], viewer.NEW_START_ARM_POSE, strict=True)))
        np.testing.assert_allclose(model.get_transform("palm_center"), model.get_transform("right_palm_link") @ local, atol=1e-10)


def _models_and_kwargs():
    old = viewer._load_urdf(viewer.OLD_URDF, load_meshes=False)
    new = viewer._load_urdf(viewer.NEW_URDF, load_meshes=False)
    joint_values = dict(
        zip(
            viewer.JOINT_NAMES,
            np.r_[viewer.NEW_START_ARM_POSE, np.zeros(20)],
            strict=True,
        )
    )
    kwargs = {
        "joint_values": joint_values,
        "object_position": np.array([0.45, 0.0, 0.56]),
        "object_wxyz": np.array([1.0, 0.0, 0.0, 0.0]),
        "object_size": np.array([0.04, 0.04, 0.04]),
        "keypoint_scale": 1.5,
        "link7_linear_velocity": np.array([0.1, 0.2, 0.3]),
        "link7_angular_velocity": np.array([1.0, 2.0, 3.0]),
    }
    return old, new, kwargs


def test_authored_link7_to_palm_transforms() -> None:
    old, new, _ = _models_and_kwargs()

    np.testing.assert_allclose(
        viewer.installation_transform(old)[:3, 3],
        [0.075, -0.0005, 0.045],
        atol=1.0e-7,
    )
    np.testing.assert_allclose(
        viewer.installation_transform(new)[:3, 3],
        [0.0, 0.0005, 0.04],
        atol=2.0e-7,
    )


def test_observation_geometry_matches_link7_as_palm_definition() -> None:
    old, new, kwargs = _models_and_kwargs()
    old_obs = viewer.compute_observation_geometry(old, **kwargs)
    new_obs = viewer.compute_observation_geometry(new, **kwargs)

    # The arm kinematic chain is unchanged, so identical arm joints produce
    # the same link7 observation even though the attached hands differ.
    np.testing.assert_allclose(
        old_obs.link7_transform, new_obs.link7_transform, atol=1.0e-8
    )
    np.testing.assert_allclose(
        old_obs.fingertip_rel(true_palm=False),
        old_obs.fingertip_positions - old_obs.code_palm_position,
    )
    np.testing.assert_allclose(
        new_obs.keypoints_rel(true_palm=False),
        new_obs.object_keypoints - new_obs.code_palm_position,
    )
    assert old_obs.critical_actor_block(true_palm=False).shape == (34,)
    assert new_obs.critical_actor_block(true_palm=True).shape == (34,)

    expected_true_linear = (
        kwargs["link7_linear_velocity"]
        + np.cross(
            kwargs["link7_angular_velocity"],
            old_obs.true_palm_position - old_obs.code_palm_position,
        )
    )
    np.testing.assert_allclose(old_obs.true_palm_velocity[:3], expected_true_linear)
    np.testing.assert_allclose(
        old_obs.current_palm_velocity[:3], kwargs["link7_linear_velocity"]
    )


def test_paired_setup_start_poses_align_semantic_palm_orientation() -> None:
    old, new, kwargs = _models_and_kwargs()
    old_kwargs = {
        **kwargs,
        "joint_values": dict(
            zip(
                viewer.JOINT_NAMES,
                np.r_[viewer.OLD_START_ARM_POSE, np.zeros(20)],
                strict=True,
            )
        ),
    }
    old_obs = viewer.compute_observation_geometry(old, **old_kwargs)
    new_obs = viewer.compute_observation_geometry(new, **kwargs)

    relative_rotation = (
        old_obs.palm_transform[:3, :3].T @ new_obs.palm_transform[:3, :3]
    )
    trace_cosine = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    assert np.arccos(trace_cosine) < np.deg2rad(0.2)
