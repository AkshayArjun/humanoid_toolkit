from humkit.controllers.composition import CompositeController
from humkit.controllers.gravity_comp import GravityCompController
from humkit.controllers.pd_control import PDController
from humkit.robot import Robot
from humkit.sim import run_viewer

robot = Robot.from_menagerie("unitree_g1", hanging=True, torque_control=True)
pd = PDController(kp=100.0, kd=10.0, q_ref=robot.q[7:])
comp = CompositeController([pd, GravityCompController()])
run_viewer(robot, comp, viewer=True)
