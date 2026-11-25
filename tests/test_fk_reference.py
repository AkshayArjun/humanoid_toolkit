import numpy as np
import pytest
from numpy.testing import assert_allclose

from humkit.kinematics import KinematicTree
from humkit.kinematics.reference import fk_recursive, fk_stack, fk_topological
from humkit.robot import Robot


@pytest.fixture(scope="module")
def free_g1():
    return Robot.from_menagerie("unitree_g1")


def random_configs(n, seed=0):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        q = rng.uniform(-1.0, 1.0, 36)
        q[3:7] /= np.linalg.norm(q[3:7])
        yield q


def assert_same_rotation(quat, expected):
    sign = 1.0 if np.dot(quat, expected) >= 0 else -1.0
    assert_allclose(sign * quat, expected, atol=1e-9)


@pytest.mark.parametrize("fk", [fk_recursive, fk_stack, fk_topological])
def test_fk_matches_mujoco(free_g1, fk):
    tree = KinematicTree.from_robot(free_g1)
    for q in random_configs(50):
        free_g1.set_state(q, np.zeros(35))
        poses = fk(tree, q)
        assert len(poses) == len(tree)
        for name, pose in poses.items():
            expected = free_g1.body_pose(name)
            assert_allclose(pose.translation, expected.translation, atol=1e-9)
            assert_same_rotation(pose.rotation.quat, expected.rotation.quat)


def test_zero_config_pelvis_sits_at_q(free_g1):
    """With q = 0 except the base, the pelvis pose is exactly the base part of q."""
    tree = KinematicTree.from_robot(free_g1)
    q = np.zeros(36)
    q[0:3] = [1.0, 2.0, 0.5]
    q[3:7] = [1.0, 0.0, 0.0, 0.0]
    poses = fk_topological(tree, q)
    assert_allclose(poses["pelvis"].translation, [1.0, 2.0, 0.5])
    # left_hip_pitch_link sits at pos="0 0.064452 -0.1027" from the pelvis
    assert_allclose(poses["left_hip_pitch_link"].translation, [1.0, 2.064452, 0.3973], atol=1e-12)
