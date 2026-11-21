import numpy as np


def test_robot_initialization(g1):
    assert len(g1.joint_names) == 29


def test_set_state(g1):
    q = g1.q
    q[7:] = 0.5
    dq = np.zeros(35)
    dq[6:] = 0.5

    g1.set_state(q, dq)
    np.testing.assert_allclose(g1.q, q)
    np.testing.assert_allclose(g1.dq, dq)


def test_set_state_normalises_base_quaternion(g1):
    q = g1.q
    q[3:7] = [1.0, 1.0, 1.0, 1.0]
    g1.set_state(q, g1.dq)
    np.testing.assert_allclose(g1.base_pose[1], [0.5, 0.5, 0.5, 0.5])
