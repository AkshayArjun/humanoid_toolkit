import numpy as np

from humkit.controllers.pd_control import PDController
from tests.fakes import FakeRobot


def test_pd_pushes_towards_reference():
    robot = FakeRobot(nu=3)
    pd = PDController(kp=10.0, kd=0.0, q_ref=np.array([1.0, 0.0, -1.0]))

    np.testing.assert_allclose(pd.compute(robot), [10.0, 0.0, -10.0])


def test_pd_damps_velocity():
    robot = FakeRobot(nu=3)
    robot.dq[6:] = [1.0, 2.0, 3.0]
    pd = PDController(kp=0.0, kd=2.0, q_ref=np.zeros(3))

    np.testing.assert_allclose(pd.compute(robot), [-2.0, -4.0, -6.0])
