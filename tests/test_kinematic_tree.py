"""Expected values come from the G1 MJCF (mujoco_menagerie/unitree_g1/g1.xml), not from MjModel."""

import mujoco as mj
import numpy as np
import pytest
from numpy.testing import assert_allclose

from humkit.error import ModelLoadError
from humkit.kinematics import JointType, KinematicTree


@pytest.fixture(scope="module")
def tree(g1):
    return KinematicTree.from_robot(g1)


def test_size_is_world_plus_pelvis_plus_one_body_per_actuated_joint(tree):
    assert len(tree) == 1 + 1 + 29


def test_root_is_the_world(tree):
    assert tree.root.name == "world"
    assert tree.root.is_root
    assert tree.root.joint is None
    assert [c.name for c in tree.root.children] == ["pelvis"]


def test_pelvis_floats_on_a_free_joint(tree):
    pelvis = tree["pelvis"]
    assert pelvis.parent is tree.root
    assert pelvis.joint.type is JointType.FREE
    assert pelvis.joint.qpos_adr == 0 and pelvis.joint.dof_adr == 0
    # <body name="pelvis" pos="0 0 0.793">, no quat → identity
    assert_allclose(pelvis.offset.translation, [0.0, 0.0, 0.793])
    assert_allclose(pelvis.offset.rotation.quat, [1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_left_hip_roll_link_matches_the_xml(tree):
    link = tree["left_hip_roll_link"]
    assert link.parent is tree["left_hip_pitch_link"]
    assert link.joint.type is JointType.HINGE
    assert link.joint.name == "left_hip_roll_joint"
    assert_allclose(link.joint.axis, [1.0, 0.0, 0.0], atol=1e-12)
    assert_allclose(link.offset.translation, [0.0, 0.052, -0.030465])
    # quat="0.996179 0 -0.0873386 0" is a ~-10° pitch, rounded in the XML to 6 digits
    assert_allclose(link.offset.rotation.quat, [0.996179, 0.0, -0.0873386, 0.0], atol=1e-6)


def test_hinge_qpos_addresses_follow_the_7_entry_free_joint(tree):
    hinge = tree["left_hip_pitch_link"].joint  # first joint after the floating base
    assert hinge.qpos_adr == 7
    assert hinge.dof_adr == 6


def test_joint_counts(tree):
    types = [link.joint.type for link in tree if link.joint is not None]
    assert types.count(JointType.FREE) == 1
    assert types.count(JointType.HINGE) == 29


def test_leaves_are_the_feet_and_wrists(tree):
    leaves = {link.name for link in tree if link.is_leaf}
    assert leaves == {
        "left_ankle_roll_link",
        "right_ankle_roll_link",
        "left_wrist_yaw_link",
        "right_wrist_yaw_link",
    }


def test_iteration_is_topological(tree):
    seen = set()
    for link in tree:
        assert link.is_root or link.parent.name in seen
        seen.add(link.name)


def test_parent_and_children_agree(tree):
    for link in tree:
        for child in link.children:
            assert child.parent is link


def test_render_indents_by_depth(tree):
    lines = tree.render().splitlines()
    assert len(lines) == 31
    assert lines[0] == "world"
    assert lines[1].startswith("  pelvis")
    # world → pelvis → left_hip_pitch_link → left_hip_roll_link: depth 3
    roll = next(line for line in lines if "left_hip_roll_link" in line)
    assert roll.startswith("      left_hip_roll_link")
    assert "axis (1 0 0)" in roll


def test_unknown_link_name_raises(tree):
    with pytest.raises(KeyError, match="no_such_link"):
        tree["no_such_link"]


def test_body_with_two_joints_is_rejected():
    spec = mj.MjSpec()
    body = spec.worldbody.add_body(name="arm")
    body.add_geom(size=[0.1, 0, 0])
    body.add_joint(name="a", type=mj.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0])
    body.add_joint(name="b", type=mj.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    with pytest.raises(ModelLoadError, match="2 joints"):
        KinematicTree.from_model(spec.compile())


def test_tiny_two_link_chain():
    """A hand-built chain checks the builder without relying on the G1."""
    spec = mj.MjSpec()
    upper = spec.worldbody.add_body(name="upper", pos=[0, 0, 1])
    upper.add_geom(size=[0.1, 0, 0])
    upper.add_joint(name="shoulder", type=mj.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
    lower = upper.add_body(name="lower", pos=[0.5, 0, 0])
    lower.add_geom(size=[0.1, 0, 0])
    lower.add_joint(name="elbow", type=mj.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0])
    tree = KinematicTree.from_model(spec.compile())

    assert [link.name for link in tree] == ["world", "upper", "lower"]
    assert tree["lower"].parent is tree["upper"]
    assert tree["lower"].joint.type is JointType.SLIDE
    assert tree["lower"].joint.qpos_adr == 1
    assert_allclose(tree["lower"].offset.translation, [0.5, 0.0, 0.0])
    assert tree.render() == (
        "world\n  upper  [hinge shoulder, axis (0 0 1)]\n    lower  [slide elbow, axis (1 0 0)]"
    )
    assert np.all(tree["upper"].joint.anchor == 0.0)
