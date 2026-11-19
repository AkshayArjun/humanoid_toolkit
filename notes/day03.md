# Class setup for the robot.

### the goal of the setup is to create a class setup for the robot so that the class handles the mujoco to bot env

the result should be the class called Robot, which takes has a method called 'from_menagerie'

it shoudl include joint_names, q, de, base_pose and another method called step(q,dq)

joint_limmits is a dict that returns model.jnt_range

__repr__ outupts necesasry inf
