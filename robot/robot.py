import mujoco


class Robot:
    def __init__(self):
        self.spec = mujoco.MjSpec.from_file(r"robot/robot.xml")

    def to_mjspec(self):
        return self.spec
