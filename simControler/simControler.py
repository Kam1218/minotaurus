import mujoco
import mujoco.viewer
import time
from map import World, Goal
from robot import Robot
from planner import Planner
from controler import Controler
import numpy as np


class SimControler:
    def __init__(self, world: World, robot: Robot):
        self.world = world
        self.robot = robot

        scene_spec = world.to_mjspec()
        robot_spec = robot.to_mjspec()

        frame = scene_spec.worldbody.add_frame(pos=[0, 0, 0.0])
        scene_spec.attach(robot_spec, frame=frame, prefix="robot_")

        self.m = scene_spec.compile()
        self.d = mujoco.MjData(self.m)

        robot.setModelData(self.m, self.d)

        self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
        self.configure_viewier()

    def run(self):
        try:
            self.last_update = 0
            self.control_t = 0.01

            self.start_sim_t = self.d.time
            self.start_wall_t = time.time()
            self.end = True
            self.flip = True

            self.k_k = np.array([[2], [10], [3]])
            self.k_s = np.array([[0.1], [1], [1]])

            self.pl = Planner()
            # self.pl.square(0.2, 5.0, phi=np.pi / 4.0)

            self.ctrl = Controler()

            while self.viewer.is_running():
                if self.d.time - self.last_update >= self.control_t:

                    self.last_update = self.d.time

                    self.sim_step()

                mujoco.mj_step(self.m, self.d)

                self.viewer.sync()

                self.syncTime(self.start_sim_t, self.start_wall_t)
        finally:
            self.viewer.close()

    def sim_step(self):
        if self.end:
            if self.flip:
                self.pl.circle(0.2, 2.5)
                print("teraz kółeczko")
            else:
                self.pl.square(0.2, 2 * 2.5 / np.sqrt(2), 3 * np.pi / 4.0)
                print("a teraz kwadracik")
            self.flip = not self.flip
            self.start_t = self.d.time

        xr, dxr, self.end = self.pl.get(self.d.time - self.start_t)

        v, omega = self.ctrl.kanayama(self.robot.x, xr, dxr, self.k_k)
        # v, omega = self.ctrl.samson(robot.x, xr, dxr, k_s)

        self.robot.ctrl(v, omega)

    def syncTime(self, start_sim_t, start_wall_t):
        wall_elapsed = time.time() - start_wall_t

        sim_elapsed = self.d.time - start_sim_t

        if sim_elapsed > wall_elapsed:
            time.sleep(sim_elapsed - wall_elapsed)

    def configure_viewier(self):
        self.viewer.cam.lookat = [0.0, 0.0, 0.0]
        self.viewer.cam.elevation = -90.0
        self.viewer.cam.distance = 30.0
        self.viewer.cam.azimuth = 90.0
