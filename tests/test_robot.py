import numpy as np

from humkit.robot import Robot


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


def test_torque_control_sets_nonzero_limits(g1):
    model, _ = g1._mujoco_handles()
    assert np.all(model.actuator_ctrlrange[:, 1] > 0)


def test_fresh_robot_has_valid_derived_state():
    """Poses and bias torques are usable straight after loading, before any set_state or step."""
    robot = Robot.from_menagerie("unitree_g1")
    pelvis = robot.body_pose("pelvis")
    # g1.xml: <body name="pelvis" pos="0 0 0.793">, no rotation
    np.testing.assert_allclose(pelvis.translation, [0.0, 0.0, 0.793])
    np.testing.assert_allclose(pelvis.rotation.quat, [1.0, 0.0, 0.0, 0.0], atol=1e-12)
    assert np.any(robot.bias_torques != 0.0)  # gravity loads the joints
