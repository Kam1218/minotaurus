import mujoco
import numpy as np
import time
import mujoco.viewer
import map.map as map
from robot.robot import Robot
from planner.planner import Planner
from controler.controler import Controler

import matplotlib.pyplot as plt

world = map.World(100, 100)
world.add_goal(map.Goal(30, 30))
world.generate_obstacles(30, 1.0)
# world.view()
scene_spec = world.to_mjspec()

robot = Robot()
robot_spec = robot.to_mjspec()

frame = scene_spec.worldbody.add_frame(pos=[0, 0, 0.0])
scene_spec.attach(robot_spec, frame=frame, prefix="robot_")


m = scene_spec.compile()
d = mujoco.MjData(m)

robot.setModelData(m, d)

pl = Planner()
pl.circle(0.5, 2.0)

ctrl = Controler()

d4p = []

control_t = 1.0
with mujoco.viewer.launch_passive(m, d) as viewer:

    last_update = 0

    while viewer.is_running():
        if d.time - last_update >= control_t:

            last_update = d.time
            robot.ctrl(0.5, 1.0)

            d4p.append(robot.x)

        mujoco.mj_step(m, d)

        viewer.sync()
