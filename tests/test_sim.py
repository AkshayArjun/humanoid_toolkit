from unittest.mock import Mock

import numpy as np

from humkit.robot import Robot
from humkit.sim import run_viewer


def test_headless_run_viewer():
    robot = Mock(spec=Robot)
    controller = Mock()
    robot.nu = 3
    controller.compute.return_value = np.zeros(robot.nu)

    run_viewer(robot, controller, viewer=False, steps=5)

    assert robot.step.call_count == 5
    assert robot.set_ctrl.call_count == 5
