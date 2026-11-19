from abc import ABC, abstractmethod

import numpy as np

from humkit.robot import Robot


class ControllerABC(ABC):
    @abstractmethod
    def compute(self, robot: Robot) -> np.ndarray:
        """Compute the control input for the given robot state."""
        pass
