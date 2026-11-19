import numpy as np

from humkit.controllers.controller_abc import ControllerABC
from humkit.robot import Robot


class GravityCompController(ControllerABC):
    def compute(self, robot: Robot) -> np.ndarray:
        bias_torques = robot.bias_torques
        return bias_torques
