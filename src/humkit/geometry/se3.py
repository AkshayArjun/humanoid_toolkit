"""Rigid poses in SE(3): a rotation (SO3) plus a translation.

SE3 *has an* SO3 (composition) and delegates the rotation work to it.

Method names and conventions follow Solà, Deray and Atchuthan, "A micro Lie
theory for state estimation in robotics" (2018):

    group element  M = [[R, t], [0, 1]]                  (4×4, bottom-right 1)
    tangent        τ = [ρ, θ] ∈ ℝ⁶                       (linear first, then angular)
    Lie algebra    τ^ = [[ [θ]×, ρ ], [0, 0]] ∈ se(3)    (4×4, bottom-right 0)

    Exp : ℝ⁶    → SE(3)     Exp(τ) = ( Exp(θ), Jl(θ)·ρ )
    Log : SE(3) → ℝ⁶
    exp : se(3) → SE(3)     (lowercase: works on the 4×4 matrix τ^)
    log : SE(3) → se(3)
"""

from dataclasses import dataclass

import numpy as np

import humkit.geometry.geom_tools as gt
from humkit.geometry.so3 import SO3
from humkit.types import FloatArray


@dataclass(frozen=True)
class SE3:
    """An element of SE(3): rotation R ∈ SO(3) and translation t ∈ ℝ³."""

    rotation: SO3
    translation: FloatArray

    def __post_init__(self) -> None:
        if not isinstance(self.rotation, SO3):
            raise TypeError(f"rotation must be an SO3, got {type(self.rotation).__name__}")
        t = np.array(self.translation, dtype=np.float64)
        if t.shape != (3,):
            raise ValueError(f"translation must have shape (3,), got {t.shape}")
        t.setflags(write=False)
        object.__setattr__(self, "translation", t)

    @classmethod
    def identity(cls) -> "SE3":
        return cls(SO3.identity(), np.zeros(3))

    # --- exponential and logarithm maps -------------------------------------

    @classmethod
    def Exp(cls, tau: FloatArray) -> "SE3":  # noqa: N802
        """ℝ⁶ → SE(3): twist τ = [ρ, θ] → pose ( Exp(θ), Jl(θ)·ρ )."""
        tau = np.asarray(tau, dtype=np.float64)
        if tau.shape != (6,):
            raise ValueError(f"tau must have shape (6,), got {tau.shape}")
        rho, theta = tau[:3], tau[3:]
        return cls(SO3.Exp(theta), gt.so3_left_jacobian(theta) @ rho)

    def Log(self) -> FloatArray:  # noqa: N802
        """SE(3) → ℝ⁶: pose → twist τ = [ρ, θ], with ρ = Jl(θ)⁻¹·t."""
        theta = self.rotation.Log()
        rho = np.linalg.solve(gt.so3_left_jacobian(theta), self.translation)
        return np.concatenate([rho, theta])

    @classmethod
    def exp(cls, tau_hat: FloatArray) -> "SE3":
        """se(3) → SE(3): exp(τ^) = Exp(τ)."""
        return cls.Exp(gt.se3_vee(tau_hat))

    def log(self) -> FloatArray:
        """SE(3) → se(3): log(M) = Log(M)^."""
        return gt.se3_hat(self.Log())

    # --- group operations ---------------------------------------------------

    def inverse(self) -> "SE3":
        """M⁻¹ = ( R⁻¹, −R⁻¹·t )."""
        r_inv = self.rotation.inverse()
        return SE3(r_inv, -r_inv.act(self.translation))

    def __matmul__(self, other: "SE3") -> "SE3":
        """Composition M1 ∘ M2 = ( R1·R2, t1 + R1·t2 )."""
        if not isinstance(other, SE3):
            return NotImplemented
        return SE3(
            self.rotation @ other.rotation,
            self.translation + self.rotation.act(other.translation),
        )

    def act(self, p: FloatArray) -> FloatArray:
        """Group action on a point: R·p + t."""
        return self.rotation.act(p) + self.translation

    # --- plus, minus and adjoint --------------------------------------------

    def plus(self, tau: FloatArray) -> "SE3":
        """Right ⊕: M ∘ Exp(τ)."""
        return self @ SE3.Exp(tau)

    def minus(self, other: "SE3") -> FloatArray:
        """Right ⊖: Log(other⁻¹ ∘ self)."""
        return (other.inverse() @ self).Log()

    def Adj(self) -> FloatArray:  # noqa: N802
        """6×6 adjoint for τ = [ρ, θ]:  [[R, [t]×·R], [0, R]]."""
        R = self.rotation.to_matrix()  # noqa: N806
        out = np.zeros((6, 6))
        out[:3, :3] = R
        out[:3, 3:] = gt.hat(self.translation) @ R
        out[3:, 3:] = R
        return out

    # --- conversions --------------------------------------------------------

    def to_matrix(self) -> FloatArray:
        """4×4 homogeneous matrix [[R, t], [0, 1]]."""
        M = np.eye(4)  # noqa: N806
        M[:3, :3] = self.rotation.to_matrix()
        M[:3, 3] = self.translation
        return M
