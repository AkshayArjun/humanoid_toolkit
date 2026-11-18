import mujoco.viewer as mjv

from humkit.robot import Robot


def run_viewer(robot: Robot) -> None:
    model, data = robot._mujoco_handles()
    with mjv.launch_passive(model, data) as v:
        while v.is_running():
            robot.step()
            v.sync()


def main() -> None:
    robot = Robot.from_menagerie("unitree_g1")
    print(robot)
    print(robot.joint_names)
    run_viewer(robot)


if __name__ == "__main__":
    main()
