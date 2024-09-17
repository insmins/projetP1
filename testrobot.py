"""
test bouger robot
"""

# imports
from Robot import Robot

robot = Robot()

pos =robot.calcul_pos_relative(dy=0.1)
robot.robot_c.moveL(pos, 0.5, 0.3)