"""Rotations in SO(3), stored as a unit quaternion (w, x, y, z).

Method names follow Solà, Deray and Atchuthan, "A micro Lie theory for state
estimation in robotics" (2018):

    Exp : ℝ³    → SO(3)     (capital: works on the vector θ)
    Log : SO(3) → ℝ³
    exp : so(3) → SO(3)     (lowercase: works on the 3×3 skew matrix [θ]×)
    log : SO(3) → so(3)
"""

from dataclasses import dataclass

import numpy as np

import humkit.geometry.geom_tools as gt
from humkit.types import FloatArray


@dataclass(frozen=True)
class SO3:
    """An element of SO(3), stored as a unit quaternion (w, x, y, z)."""

    quat: FloatArray

    def __post_init__(self) -> None:
        q = np.asarray(self.quat, dtype=np.float64)
        if q.shape != (4,):
            raise ValueError(f"quat must have shape (4,), got {q.shape}")
        q = gt.quat_normalize(q)
        q.setflags(write=False)
        object.__setattr__(self, "quat", q)

    @classmethod
    def identity(cls) -> "SO3":
        return cls(np.array([1.0, 0.0, 0.0, 0.0]))

    # --- exponential and logarithm maps -------------------------------------

    @classmethod
    def Exp(cls, theta: FloatArray) -> "SO3":  # noqa: N802
        """ℝ³ → SO(3): rotation vector θ (axis × angle) → rotation."""
        return cls(gt.quat_exp(theta))

    def Log(self) -> FloatArray:  # noqa: N802
        """SO(3) → ℝ³: rotation → rotation vector θ, with |θ| in [0, π]."""
        return gt.quat_log(self.quat)

    @classmethod
    def exp(cls, theta_hat: FloatArray) -> "SO3":
        """so(3) → SO(3): exp([θ]×) = Exp(θ)."""
        return cls.Exp(gt.vee(theta_hat))

    def log(self) -> FloatArray:
        """SO(3) → so(3): log(R) = [Log(R)]×."""
        return gt.hat(self.Log())

    # --- group operations ---------------------------------------------------

    def inverse(self) -> "SO3":
        """R⁻¹ (the quaternion conjugate)."""
        return SO3(gt.quat_conj(self.quat))

    def __matmul__(self, other: "SO3") -> "SO3":
        """Composition R1 ∘ R2."""
        if not isinstance(other, SO3):
            return NotImplemented
        return SO3(gt.quat_mul(self.quat, other.quat))

    def act(self, v: FloatArray) -> FloatArray:
        """Group action on a vector: R·v."""
        return gt.quat_rotate(self.quat, v)

    # --- plus, minus and adjoint --------------------------------------------

    def plus(self, theta: FloatArray) -> "SO3":
        """Right ⊕: R ∘ Exp(θ)."""
        return self @ SO3.Exp(theta)

    def minus(self, other: "SO3") -> FloatArray:
        """Right ⊖: Log(other⁻¹ ∘ self)."""
        return (other.inverse() @ self).Log()

    def Adj(self) -> FloatArray:  # noqa: N802
        """Adjoint matrix: for SO(3), Ad_R = R."""
        return self.to_matrix()

    # --- conversions --------------------------------------------------------

    def to_matrix(self) -> FloatArray:
        """3×3 rotation matrix."""
        return gt.quat_to_matrix(self.quat)
