import numpy as np

from humkit.geometry.so3 import SO3

S = np.sqrt(0.5)


def test_inverse_of_z90_is_minus_z90():
    R = SO3(np.array([S, 0.0, 0.0, S]))  # +90° about z
    np.testing.assert_allclose(R.inverse().quat, [S, 0.0, 0.0, -S], atol=1e-12)


def test_rotation_times_inverse_is_identity():
    R = SO3.Exp(np.array([0.3, -0.7, 1.1]))  # an arbitrary rotation
    np.testing.assert_allclose((R @ R.inverse()).quat, [1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_identity_leaves_rotation_unchanged():
    R = SO3.Exp(np.array([0.3, -0.7, 1.1]))
    np.testing.assert_allclose((R @ SO3.identity()).quat, R.quat, atol=1e-12)


def test_exp_of_x90_gives_known_quaternion():
    R = SO3.Exp(np.array([np.pi / 2, 0.0, 0.0]))
    s = np.sqrt(0.5)
    np.testing.assert_allclose(R.quat, [s, s, 0.0, 0.0], atol=1e-12)
