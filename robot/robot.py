import mujoco


class Robot:
    def __init__(self):
        self.spec = mujoco.MjSpec.from_file(r"robot/robot.xml")
        self.b = 0.15  # 2b - rozstaw kół
        self.r = 0.12  # r - promień koła

        self.world_data = None

    def to_mjspec(self):
        return self.spec

    def setData(self, data):
        self.world_data = data

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
        print(f"ustawiono {om_L} {om_R}")
        print(f"motor {self.left_motor} {self.right_motor}")
