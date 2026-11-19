from humkit.controllers.gravity_comp import GravityCompController
from humkit.robot import Robot
from humkit.sim import run_viewer

robot = Robot.from_menagerie("unitree_g1", hanging=False, torque_control=True)
gravity_comp = GravityCompController()

run_viewer(robot, gravity_comp, viewer=True, steps=1000)
