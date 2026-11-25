import numpy as np
import pytest
from numpy.testing import assert_allclose

from humkit.geometry.se3 import SE3
from humkit.geometry.so3 import SO3
from humkit.kinematics.ik import dls_step, ik, world_error
from humkit.robot import Robot

HAND = "left_wrist_yaw_link"


@pytest.fixture(scope="module")
def free_g1():
    return Robot.from_menagerie("unitree_g1")


def standing_q():
    q = np.zeros(36)
    q[2] = 0.793
    q[3] = 1.0
    return q


def test_world_error_is_in_world_frame():
    rot = SO3.Exp(np.array([0.0, 0.0, np.pi / 2]))  # hand turned 90° about z
    current = SE3(rot, np.zeros(3))
    target = SE3(rot, np.array([0.01, 0.0, 0.0]))  # 1 cm along world x
    assert_allclose(world_error(current, target), [0.01, 0, 0, 0, 0, 0], atol=1e-12)


def test_world_error_rotation_about_world_z():
    current = SE3(SO3.Exp(np.array([np.pi / 2, 0.0, 0.0])), np.zeros(3))  # tilted about x
    target = SE3(SO3.Exp(np.array([0.0, 0.0, 0.1])) @ current.rotation, np.zeros(3))
    assert_allclose(world_error(current, target), [0, 0, 0, 0, 0, 0.1], atol=1e-12)


def test_dls_step_with_no_damping_inverts_a_square_jacobian():
    J = np.diag([2.0, 4.0])  # noqa: N806
    assert_allclose(dls_step(J, np.array([1.0, 1.0]), damping=0.0), [0.5, 0.25])


def test_dls_step_damping_shrinks_the_step():
    # 1x1 case by hand: dq = j·e / (j² + λ²) = 1·1 / (1 + 1) = 0.5
    assert_allclose(dls_step(np.array([[1.0]]), np.array([1.0]), damping=1.0), [0.5])


def test_already_at_target_returns_immediately(free_g1):
    q0 = standing_q()
    target = free_g1.fk(q0)[HAND]
    q, converged = ik(free_g1, HAND, target, q0)
    assert converged
    assert_allclose(q, q0, atol=0)


def test_reaches_random_reachable_targets(free_g1):
    rng = np.random.default_rng(0)
    q0 = standing_q()
    for _ in range(20):
        q_true = q0.copy()
        q_true[7:] += rng.uniform(-0.3, 0.3, 29)
        target = free_g1.fk(q_true)[HAND]

        q, converged = ik(free_g1, HAND, target, q0)

        assert converged
        assert_allclose(q[:7], q0[:7], atol=0)  # the floating base never moves
        assert np.linalg.norm(world_error(free_g1.fk(q)[HAND], target)) < 1e-6


def test_unreachable_target_reports_failure(free_g1):
    far_away = SE3(SO3.identity(), np.array([5.0, 0.0, 0.0]))
    _, converged = ik(free_g1, HAND, far_away, standing_q(), max_iters=20)
    assert not converged
