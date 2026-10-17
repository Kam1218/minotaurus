import mujoco
import numpy as np
import time
import mujoco.viewer
import map.map as map
from robot import Robot
from planner import Planner
from controler import Controler
from simControler import SimControler


def syncTime(start_sim_t, start_wall_t):
    wall_elapsed = time.time() - start_wall_t

    sim_elapsed = d.time - start_sim_t

    if sim_elapsed > wall_elapsed:
        time.sleep(sim_elapsed - wall_elapsed)


world = map.World(100, 100)
world.safe_r = 10.0
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
pl.circle(0.2, 2.5)
# pl.square(0.2, 5.0, phi=np.pi / 4.0)

ctrl = Controler()

k_k = np.array([[2], [10], [3]])
k_s = np.array([[0.1], [1], [1]])

control_t = 0.01
flip = False
with mujoco.viewer.launch_passive(m, d) as viewer:

    last_update = 0
    path = 0

    viewer.cam.lookat = [
        0.0,
        0.0,
        0.0,
    ]
    viewer.cam.elevation = -90.0
    viewer.cam.distance = 30.0
    viewer.cam.azimuth = 90.0

    start_sim_t = d.time
    start_wall_t = time.time()
    end = True

    while viewer.is_running():
        if d.time - last_update >= control_t:

            last_update = d.time

            if end:
                if flip:
                    pl.circle(0.2, 2.5)
                    print("teraz kółeczko")
                else:
                    pl.square(0.2, 2 * 2.5 / np.sqrt(2), 3 * np.pi / 4.0)
                    print("a teraz kwadracik")
                flip = not flip
                start_t = d.time

            xr, dxr, end = pl.get(d.time - start_t)

            v, omega = ctrl.kanayama(robot.x, xr, dxr, k_k)
            # v, omega = ctrl.samson(robot.x, xr, dxr, k_s)

            robot.ctrl(v, omega)

        mujoco.mj_step(m, d)

        viewer.sync()

        syncTime(start_sim_t, start_wall_t)
