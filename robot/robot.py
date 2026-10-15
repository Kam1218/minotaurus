import mujoco
import numpy as np
import math


class Robot:
    def __init__(self):
        self.spec = mujoco.MjSpec.from_file(r"robot/robot.xml")
        self.b = 0.15  # 2b - rozstaw kół
        self.r = 0.12  # r - promień koła

        self.world_data = None
        self.model = None

        self.pos_adr = None
        self.quat_adr = None
        self.vel_adr = None
        self.gyro_adr = None

    def to_mjspec(self):
        return self.spec

    def setModelData(self, model, data):
        self.world_data = data
        self.model = model

        pos_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "robot_pos")
        quat_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "robot_quat"
        )
        vel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "robot_vel")
        gyro_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "robot_gyro"
        )

        self.pos_adr = model.sensor_adr[pos_id]
        self.quat_adr = model.sensor_adr[quat_id]
        self.vel_adr = model.sensor_adr[vel_id]
        self.gyro_adr = model.sensor_adr[gyro_id]

    @property
    def x(self):
        x, y, _ = self.world_data.sensordata[self.pos_adr : self.pos_adr + 3]
        w, qx, qy, qz = self.world_data.sensordata[self.quat_adr : self.quat_adr + 4]

        theta = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        theta = math.atan2(math.sin(theta), math.cos(theta))

        return np.array([[x], [y], [theta]])

    @property
    def dx(self):
        dx, dy, _ = self.world_data.sensordata[self.vel_adr : self.vel_adr + 3]
        _, _, dtheta = self.world_data.sensordata[self.gyro_adr : self.gyro_adr + 3]

        return np.array([[dx], [dy], [dtheta]])

    @property
    def left_motor(self):
        return self.world_data.ctrl[0]

    @left_motor.setter
    def left_motor(self, value):
        self.world_data.ctrl[0] = value

    @property
    def right_motor(self):
        return self.world_data.ctrl[1]

    @right_motor.setter
    def right_motor(self, value):
        self.world_data.ctrl[1] = value

    def _vom2omom(self, v, omega):
        om_R = (v + omega * self.b) / self.r
        om_L = (v - omega * self.b) / self.r

        return om_R, om_L

    def ctrl(self, v, omega):
        om_R, om_L = self._vom2omom(v, omega)

        self.left_motor = om_L
        self.right_motor = om_R
