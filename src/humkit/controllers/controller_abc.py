from abc import ABC, abstractmethod
from typing import Protocol

from humkit.types import FloatArray


class RobotLike(Protocol):
    """What a controller needs from a robot: read-only state.

    Structural typing: any object with these attributes satisfies RobotLike,
    without inheriting from it. `Robot` does, and so does the test `FakeRobot`.
    """

    @property
    def q(self) -> FloatArray: ...
    @property
    def dq(self) -> FloatArray: ...
    @property
    def nu(self) -> int: ...
    @property
    def bias_torques(self) -> FloatArray: ...


class ControllerABC(ABC):
    @abstractmethod
    def compute(self, robot: RobotLike) -> FloatArray:
        """Compute joint torques (length nu) for the given robot state."""
