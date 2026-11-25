import mujoco as mj
import numpy as np
import pytest
from numpy.testing import assert_allclose

from humkit.kinematics import KinematicTree
from humkit.kinematics.reference_jacobians import body_jacobian, num_dofs
from humkit.robot import Robot

BODIES = ["pelvis", "left_wrist_yaw_link", "right_ankle_roll_link", "torso_link"]


@pytest.fixture(scope="module")
def free_g1():
    return Robot.from_menagerie("unitree_g1")


@pytest.fixture(scope="module")
def planar_arm():
    """Two hinges about z: shoulder at the origin, elbow 1 m along x, hand 0.5 m further."""
    spec = mj.MjSpec()
    upper = spec.worldbody.add_body(name="upper")
    upper.add_geom(size=[0.1, 0, 0])
    upper.add_joint(name="shoulder", type=mj.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
    lower = upper.add_body(name="lower", pos=[1, 0, 0])
    lower.add_geom(size=[0.1, 0, 0])
    lower.add_joint(name="elbow", type=mj.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
    hand = lower.add_body(name="hand", pos=[0.5, 0, 0])
    hand.add_geom(size=[0.1, 0, 0])
    return KinematicTree.from_model(spec.compile())


def test_planar_arm_straight(planar_arm):
    # hand at (1.5, 0, 0); z × (1.5, 0, 0) = (0, 1.5, 0) and z × (0.5, 0, 0) = (0, 0.5, 0)
    jac = body_jacobian(planar_arm, np.zeros(2), "hand")
    expected = np.array(
        [
            [0.0, 0.0],
            [1.5, 0.5],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [1.0, 1.0],
        ]
    )
    assert_allclose(jac, expected, atol=1e-12)


def test_planar_arm_shoulder_at_90_degrees(planar_arm):
    # hand at (0, 1.5, 0); z × (0, 1.5, 0) = (-1.5, 0, 0); elbow at (0, 1, 0) gives (-0.5, 0, 0)
    jac = body_jacobian(planar_arm, np.array([np.pi / 2, 0.0]), "hand")
    assert_allclose(jac[:3], [[-1.5, -0.5], [0.0, 0.0], [0.0, 0.0]], atol=1e-12)
    assert_allclose(jac[3:], [[0.0, 0.0], [0.0, 0.0], [1.0, 1.0]], atol=1e-12)


def test_planar_arm_folded_elbow(planar_arm):
    # elbow at 180°: hand back at (0.5, 0, 0) and the elbow column points the other way
    jac = body_jacobian(planar_arm, np.array([0.0, np.pi]), "hand")
    assert_allclose(jac[:3, 0], [0.0, 0.5, 0.0], atol=1e-12)
    assert_allclose(jac[:3, 1], [0.0, -0.5, 0.0], atol=1e-12)


def test_g1_has_35_dofs(free_g1):
    assert num_dofs(KinematicTree.from_robot(free_g1)) == 6 + 29


def test_matches_mj_jac_body(free_g1):
    tree = KinematicTree.from_robot(free_g1)
    model, data = free_g1._mujoco_handles()
    rng = np.random.default_rng(0)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    for _ in range(20):
        q = rng.uniform(-1.0, 1.0, model.nq)
        q[3:7] /= np.linalg.norm(q[3:7])
        free_g1.set_state(q, np.zeros(model.nv))
        for body in BODIES:
            mj.mj_jacBody(model, data, jacp, jacr, model.body(body).id)
            jac = body_jacobian(tree, q, body)
            assert_allclose(jac[:3], jacp, atol=1e-9)
            assert_allclose(jac[3:], jacr, atol=1e-9)


def test_joints_off_the_path_have_zero_columns(free_g1):
    tree = KinematicTree.from_robot(free_g1)
    q = free_g1.q
    jac = body_jacobian(tree, q, "left_wrist_yaw_link")
    for leg_joint in ["left_hip_pitch_link", "right_knee_link", "left_ankle_roll_link"]:
        assert np.all(jac[:, tree[leg_joint].joint.dof_adr] == 0.0)
    assert np.any(jac[:, tree["left_elbow_link"].joint.dof_adr] != 0.0)


def test_jacobian_predicts_small_motions(free_g1):
    """J·dq ≈ the change in FK for a small dq (finite-difference check)."""
    tree = KinematicTree.from_robot(free_g1)
    rng = np.random.default_rng(1)
    q = rng.uniform(-1.0, 1.0, 36)
    q[3:7] = [1.0, 0.0, 0.0, 0.0]
    dq = np.zeros(35)
    dq[6:] = rng.uniform(-1.0, 1.0, 29) * 1e-6
    q_next = q.copy()
    q_next[7:] += dq[6:]
    before = free_g1.fk(q)["left_wrist_yaw_link"].translation
    after = free_g1.fk(q_next)["left_wrist_yaw_link"].translation
    jac = body_jacobian(tree, q, "left_wrist_yaw_link")
    assert_allclose(after - before, jac[:3] @ dq, atol=1e-10)


def test_robot_jac_matches_reference(free_g1):
    tree = KinematicTree.from_robot(free_g1)
    rng = np.random.default_rng(2)
    for _ in range(20):
        q = rng.uniform(-1.0, 1.0, 36)
        q[3:7] /= np.linalg.norm(q[3:7])
        for body in BODIES:
            assert_allclose(free_g1.jac(q, body), body_jacobian(tree, q, body), atol=1e-9)


def test_robot_jac_does_not_touch_live_state(free_g1):
    free_g1.set_state(free_g1.q, np.zeros(35))
    q_before = free_g1.q
    q_query = np.zeros(36)
    q_query[3] = 1.0  # identity base orientation
    free_g1.jac(q_query, "left_wrist_yaw_link")
    assert_allclose(free_g1.q, q_before, atol=0)


def test_robot_jac_unknown_body_raises(free_g1):
    with pytest.raises(KeyError):
        free_g1.jac(free_g1.q, "no_such_body")
