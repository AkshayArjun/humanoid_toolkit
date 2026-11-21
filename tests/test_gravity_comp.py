import numpy as np

from humkit.controllers.gravity_comp import GravityCompController
from tests.fakes import FakeRobot


def test_gravity_comp_controller():
    robot = FakeRobot(nu=3)
    robot.bias_torques = np.array([1.0, 2.0, 3.0])
    gravity_comp_controller = GravityCompController()

    computed_torques = gravity_comp_controller.compute(robot)
    np.testing.assert_allclose(computed_torques, robot.bias_torques)
