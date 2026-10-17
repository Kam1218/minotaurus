import mujoco
import numpy as np
import time
import mujoco.viewer
import map.map as map
from robot import Robot
from planner import Planner
from controler import Controler
from simControler import SimControler

world = map.World(100, 100)
world.add_goal(map.Goal(30, 30))
world.generate_obstacles(30, 1.0)
# world.view()

robot = Robot()

sim_ctrl = SimControler(world, robot)

sim_ctrl.run()
