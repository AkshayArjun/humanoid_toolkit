"""Damped least-squares inverse kinematics for one body of a floating-base robot."""

from typing import Protocol

import numpy as np

from humkit.geometry.se3 import SE3
from humkit.types import FloatArray


class KinematicModel(Protocol):
    def fk(self, q: FloatArray) -> dict[str, SE3]: ...
    def jac(self, q: FloatArray, body: str) -> FloatArray: ...
    @property
    def joint_limits(self) -> tuple[FloatArray, FloatArray]: ...


def world_error(current: SE3, target: SE3) -> FloatArray:
    """[linear; angular] error from current to target, in the world frame like robot.jac."""
    e_lin = target.translation - current.translation
    e_ang = current.rotation.to_matrix() @ (current.rotation.inverse() @ target.rotation).Log()
    return np.concatenate([e_lin, e_ang])


def dls_step(J: FloatArray, e: FloatArray, damping: float) -> FloatArray:  # noqa: N803
    """dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e"""
    I = np.eye(J.shape[0])  # noqa: N806, E741
    step: FloatArray = J.T @ np.linalg.solve(J @ J.T + damping**2 * I, e)
    return step


def ik(
    robot: KinematicModel,
    body: str,
    target: SE3,
    q0: FloatArray,
    tol: float = 1e-6,
    max_iters: int = 100,
    damping: float = 1e-2,
) -> tuple[FloatArray, bool]:

    lower, upper = robot.joint_limits
    q = np.array(q0, dtype=np.float64)
    for _ in range(max_iters):
        e = world_error(robot.fk(q)[body], target)
        if np.linalg.norm(e) < tol:
            return q, True
        J = robot.jac(q, body)[:, 6:]
        q[7:] += dls_step(J, e, damping)
        q[7:] = np.clip(q[7:], lower, upper)
    return q, False
