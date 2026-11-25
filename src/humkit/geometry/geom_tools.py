"""Low-level SO(3) maths on raw NumPy arrays.

Quaternions are stored as (w, x, y, z), matching MuJoCo.

Notation follows Solà, Deray and Atchuthan, "A micro Lie theory for state
estimation in robotics" (2018): hat/vee from Example 3 (SO(3)), and the
quaternion Exp/Log from Table I (S³ row), Exp(θ) = exp(uθ/2).
Small-angle series follow Sophus.
"""

import math

import numpy as np

from humkit.types import FloatArray

_EPS = 1e-8  # below this, use small-angle series to avoid dividing by ~0


def hat(omega: FloatArray) -> FloatArray:
    """ℝ³ → so(3): the skew-symmetric matrix with hat(a) @ b == a × b."""
    x, y, z = omega
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )


def vee(Omega: FloatArray) -> FloatArray:  # noqa: N803
    """so(3) → ℝ³. Inverse of hat."""
    return np.array([Omega[2, 1], Omega[0, 2], Omega[1, 0]])


def quat_normalize(q: FloatArray) -> FloatArray:
    """Scale q to unit length."""
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError(f"Cannot normalise a near-zero quaternion: {q}")
    return q / norm


def quat_conj(q: FloatArray) -> FloatArray:
    """Conjugate (w, -x, -y, -z). For a unit quaternion this is the inverse."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a: FloatArray, b: FloatArray) -> FloatArray:
    """Quaternion product a ⊗ b: rotate by b first, then by a."""
    w1, v1 = a[0], a[1:]
    w2, v2 = b[0], b[1:]
    w = w1 * w2 - np.dot(v1, v2)
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return np.concatenate([[w], v])


def quat_exp(omega: FloatArray) -> FloatArray:
    """Exp: rotation vector ω ∈ ℝ³ → unit quaternion. q = (cos(θ/2), sin(θ/2)·u)."""
    omega = np.asarray(omega, dtype=np.float64)
    theta2 = float(np.dot(omega, omega))
    theta = np.sqrt(theta2)
    if theta < _EPS:
        theta4 = theta2 * theta2
        real = 1.0 - theta2 / 8.0 + theta4 / 384.0
        imag = 0.5 - theta2 / 48.0 + theta4 / 3840.0
    else:
        real = np.cos(0.5 * theta)
        imag = np.sin(0.5 * theta) / theta
    return np.concatenate([[real], imag * omega])


def quat_log(q: FloatArray) -> FloatArray:
    """Log: unit quaternion → rotation vector ω ∈ ℝ³, with angle in [0, π]."""
    q = np.asarray(q, dtype=np.float64)
    if q[0] < 0.0:
        q = -q  # q and -q are the same rotation; pick the one with the short angle
    w, v = float(q[0]), q[1:]
    n2 = float(np.dot(v, v))
    n = np.sqrt(n2)
    if n < _EPS:
        # series of 2·atan(n/w)/n around n = 0
        scale = 2.0 / w - (2.0 / 3.0) * n2 / (w**3)
    else:
        scale = 2.0 * np.arctan2(n, w) / n
    return scale * v


def quat_to_matrix(q: FloatArray) -> FloatArray:
    """Unit quaternion → 3×3 rotation matrix."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_rotate(q: FloatArray, v: FloatArray) -> FloatArray:
    """Rotate the 3-vector v by the unit quaternion q."""
    return quat_to_matrix(q) @ np.asarray(v, dtype=np.float64)


def so3_left_jacobian(theta: FloatArray) -> FloatArray:
    """Left Jacobian of SO(3), Jl(θ). Also the V matrix in the SE(3) Exp.

    Jl = I + ((1 - cos a) / a²)·[θ]× + ((a - sin a) / a³)·[θ]×²,  a = |θ|
    """
    theta = np.asarray(theta, dtype=np.float64)
    a2 = float(np.dot(theta, theta))
    a = math.sqrt(a2)
    K = hat(theta)  # noqa: N806
    eye: FloatArray = np.eye(3)
    if a < _EPS:
        return eye + 0.5 * K + (1.0 / 6.0) * (K @ K)
    return eye + ((1.0 - math.cos(a)) / a2) * K + ((a - math.sin(a)) / (a2 * a)) * (K @ K)


def se3_hat(tau: FloatArray) -> FloatArray:
    """ℝ⁶ → se(3): twist [ρ, θ] → 4×4 matrix [[ [θ]×, ρ ], [0, 0]]."""
    tau = np.asarray(tau, dtype=np.float64)
    out = np.zeros((4, 4))
    out[:3, :3] = hat(tau[3:])
    out[:3, 3] = tau[:3]
    return out


def se3_vee(Tau: FloatArray) -> FloatArray:  # noqa: N803
    """se(3) → ℝ⁶. Inverse of se3_hat: returns [ρ, θ]."""
    return np.concatenate([Tau[:3, 3], vee(Tau[:3, :3])])
