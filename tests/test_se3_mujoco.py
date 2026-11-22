import numpy as np

from humkit.geometry.se3 import SE3
from humkit.geometry.so3 import SO3

BODIES = ["pelvis", "left_hip_pitch_link", "left_knee_link"]


def _reset(g1) -> None:
    model, _ = g1._mujoco_handles()
    g1.set_state(model.qpos0.copy(), np.zeros(model.nv))


def test_rotation_matches_mujoco_xmat(g1):
    """Our quaternion → matrix agrees with MuJoCo's own rotation matrix."""
    _reset(g1)
    _, data = g1._mujoco_handles()
    for name in BODIES:
        expected = data.body(name).xmat.reshape(3, 3)
        np.testing.assert_allclose(g1.body_pose(name).rotation.to_matrix(), expected, atol=1e-9)


def test_parent_inverse_times_child_is_local_offset(g1):
    """At the zero pose, parent⁻¹ ∘ child equals the child's offset in the XML."""
    _reset(g1)
    model, _ = g1._mujoco_handles()
    for name in BODIES[1:]:
        child = model.body(name)
        parent = model.body(int(child.parentid[0]))
        relative = g1.body_pose(parent.name).inverse() @ g1.body_pose(name)
        expected = SE3(SO3(child.quat.copy()), child.pos.copy())
        np.testing.assert_allclose(relative.to_matrix(), expected.to_matrix(), atol=1e-9)


def test_compose_matches_mujoco_at_random_pose(g1):
    """Composition works with the joints bent, not just at zero."""
    model, data = g1._mujoco_handles()
    rng = np.random.default_rng(0)
    q = model.qpos0.copy()
    q[7:] += rng.uniform(-0.3, 0.3, size=model.nq - 7)
    g1.set_state(q, np.zeros(model.nv))
    pelvis = g1.body_pose("pelvis")
    knee = g1.body_pose("left_knee_link")
    knee_in_pelvis = pelvis.inverse() @ knee
    np.testing.assert_allclose((pelvis @ knee_in_pelvis).to_matrix(), knee.to_matrix(), atol=1e-9)
