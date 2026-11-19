import mujoco.viewer

from humkit.controllers.controller_abc import ControllerABC
from humkit.robot import Robot


def run_viewer(
    robot: Robot, controller: ControllerABC, *, viewer: bool = True, steps: int | None = None
) -> None:
    """Run the control loop: read state, compute, apply, step."""

    def tick() -> None:
        u = controller.compute(robot)
        robot.set_ctrl(u)
        robot.step()

    if not viewer:
        if steps is None:
            raise ValueError("Must specify number of steps when not using viewer")
        for _ in range(steps):
            tick()
        return

    model, data = robot._mujoco_handles()
    with mujoco.viewer.launch_passive(model, data) as v:
        n = 0
        while v.is_running() and (steps is None or n < steps):
            tick()
            v.sync()
            n += 1
