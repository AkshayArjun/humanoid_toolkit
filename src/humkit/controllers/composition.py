import numpy as np

from humkit.controllers.controller_abc import ControllerABC, RobotLike
from humkit.error import ControllerError
from humkit.types import FloatArray

"""Sums child controllers' outputs. All children must return joint torques (length nu)."""


class CompositeController(ControllerABC):
    def __init__(self, controllers: list[ControllerABC]):
        if not controllers:
            raise ControllerError("composite controllers needs atleast one child controller")
        self.controllers = controllers

    def compute(self, robot: RobotLike) -> FloatArray:
        return sum(
            (c.compute(robot) for c in self.controllers),
            start=np.zeros(robot.nu),
        )
