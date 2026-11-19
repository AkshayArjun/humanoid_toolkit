from humkit.controllers.pd_control import PDController
from humkit.robot import Robot
from humkit.sim import run_viewer

robot = Robot.from_menagerie("unitree_g1", hanging=True, torque_control=True)
pd = PDController(kp=100.0, kd=10.0, q_ref=robot.q[7:])

run_viewer(robot, pd, viewer=True, steps=1000)
