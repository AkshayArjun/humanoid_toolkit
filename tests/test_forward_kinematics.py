import numpy as np
import pytest
from numpy.testing import assert_allclose

from humkit.error import StateDimensionError
from humkit.kinematics import KinematicTree
from humkit.kinematics.reference import fk_topological
from humkit.robot import Robot


@pytest.fixture(scope="module")
def free_g1():
    return Robot.from_menagerie("unitree_g1")


def random_q(rng):
    q = rng.uniform(-1.0, 1.0, 36)
    q[3:7] /= np.linalg.norm(q[3:7])
    return q


def test_matches_reference_fk(free_g1):
    tree = KinematicTree.from_robot(free_g1)
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = random_q(rng)
        poses = free_g1.forward_kinematics(q)
        expected = fk_topological(tree, q)
        assert poses.keys() == expected.keys()
        for name, pose in poses.items():
            assert_allclose(pose.translation, expected[name].translation, atol=1e-9)
            quat, ref = pose.rotation.quat, expected[name].rotation.quat
            assert_allclose(np.sign(np.dot(quat, ref)) * quat, ref, atol=1e-9)


def test_does_not_touch_live_state(free_g1):
    rng = np.random.default_rng(2)
    free_g1.set_state(random_q(rng), np.zeros(35))
    q_before = free_g1.q
    pelvis_before = free_g1.body_pose("pelvis")

    free_g1.forward_kinematics(random_q(rng))

    assert_allclose(free_g1.q, q_before, atol=0)
    assert_allclose(free_g1.body_pose("pelvis").translation, pelvis_before.translation, atol=0)


def test_returned_poses_do_not_change_on_the_next_call(free_g1):
    rng = np.random.default_rng(3)
    first = free_g1.forward_kinematics(random_q(rng))["left_wrist_yaw_link"]
    saved = first.translation.copy()
    free_g1.forward_kinematics(random_q(rng))
    assert_allclose(first.translation, saved, atol=0)


def test_base_at_q_with_joints_zero(free_g1):
    q = np.zeros(36)
    q[0:3] = [1.0, 2.0, 0.5]
    q[3] = 1.0
    poses = free_g1.forward_kinematics(q)
    assert_allclose(poses["world"].translation, [0.0, 0.0, 0.0])
    assert_allclose(poses["pelvis"].translation, [1.0, 2.0, 0.5])
    # g1.xml: left_hip_pitch_link pos="0 0.064452 -0.1027" relative to the pelvis
    assert_allclose(poses["left_hip_pitch_link"].translation, [1.0, 2.064452, 0.3973], atol=1e-12)


@pytest.mark.parametrize("bad_q", [0.5, np.zeros(35), np.zeros(37), np.zeros((36, 1))])
def test_wrong_shape_raises(free_g1, bad_q):
    with pytest.raises(StateDimensionError):
        free_g1.forward_kinematics(bad_q)
