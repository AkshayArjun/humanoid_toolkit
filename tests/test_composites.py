import numpy as np

from humkit.controllers.composition import CompositeController
from humkit.controllers.controller_abc import ControllerABC
from tests.fakes import FakeRobot


class StubController(ControllerABC):
    def __init__(self, output: list[float]) -> None:
        self.output = np.array(output)

    def compute(self, robot) -> np.ndarray:
        return self.output


def test_composite_sums_children():
    robot = FakeRobot(nu=3)
    comp = CompositeController(
        [
            StubController([1.0, 0.0, -1.0]),
            StubController([-1.0, 2.0, 1.0]),
        ]
    )
    np.testing.assert_allclose(comp.compute(robot), [0.0, 2.0, 0.0])
