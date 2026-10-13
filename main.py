import mujoco
import numpy as np
import time
import mujoco.viewer
import map.map as map
from robot.robot import Robot

world = map.World(100, 100)
world.add_goal(map.Goal(30, 30))
# world.generate_obstacles(30, 1.0)
scene_spec = world.to_mjspec()

robot = Robot()
robot_spec = robot.to_mjspec()

frame = scene_spec.worldbody.add_frame(pos=[0, 0, 0.0])
scene_spec.attach(robot_spec, frame=frame, prefix="robot_")


m = scene_spec.compile()
d = mujoco.MjData(m)


# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running():
#         step_start = time.time()

#         mujoco.mj_step(model, data)

#         viewer.sync()

#         # Rudimentary time keeping, will drift relative to wall clock.
#         time_until_next_step = model.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)
control_t = 0
with mujoco.viewer.launch_passive(m, d) as viewer:

    last_update = 0

    while viewer.is_running():
        if d.time - last_update >= control_t:
            last_update = d.time
            d.ctrl[0] = 2.0
            d.ctrl[1] = -2.0

        mujoco.mj_step(m, d)

        viewer.sync()
