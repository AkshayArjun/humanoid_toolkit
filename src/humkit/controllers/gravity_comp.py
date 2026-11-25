from humkit.controllers.controller_abc import ControllerABC, RobotLike
from humkit.types import FloatArray


class GravityCompController(ControllerABC):
    def compute(self, robot: RobotLike) -> FloatArray:
        bias_torques = robot.bias_torques
        return bias_torques
