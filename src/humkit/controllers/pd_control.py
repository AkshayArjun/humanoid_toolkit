import numpy as np

from humkit.controllers.controller_abc import ControllerABC
from humkit.error import ControllerError
from humkit.robot import Robot


class PDController(ControllerABC):
    def __init__(self, kp: float, kd: float, q_ref: np.ndarray):
        self.kp = kp
        self.kd = kd
        self.q_ref = np.asarray(q_ref, dtype=float).copy()

    def compute(self, robot: Robot) -> np.ndarray:
        if self.q_ref.shape != (robot.nu,):
            raise ControllerError(
                f"PD q_ref has shape {self.q_ref.shape}, robot has {robot.nu} actuators"
            )
        q = robot.q[7:]  # Exclude the base position and orientation
        dq = robot.dq[6:]  # Exclude the base linear and angular velocities
        error = self.q_ref - q
        control_input = self.kp * error - self.kd * dq
        return control_input
