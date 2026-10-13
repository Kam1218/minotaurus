import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import random
import math
import mujoco


class Body:
    def __init__(self, x, y, r, h):
        self.x = x
        self.y = y
        self.r = r
        self.h = h

    def intersection(self, b: "Body"):
        if math.sqrt((self.x - b.x) ** 2 + (self.y - b.y) ** 2) <= (self.r + b.r):
            return True

        return False


class Goal(Body):
    def __init__(self, x, y):
        super().__init__(x, y, r=1.0, h=0.01)


class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.bodies: list[Body] = []
        self.safe_r = 5.0

    def add_goal(self, g: Goal):
        self.bodies.append(g)

    def add_obstacle(self, o: Body) -> bool:
        possible = True

        for b in [*self.bodies, Body(0, 0, self.safe_r, 0)]:
            if b.intersection(o):
                possible = False
                break

        if possible:
            self.bodies.append(o)

        return possible

    def generate_obstacles(self, n, r, h=2.0):
        i = 0
        while i < n:
            x = random.random() * (self.width - 2 * r) - (self.width / 2)
            y = random.random() * (self.height - 2 * r) - (self.height / 2)

            if self.add_obstacle(Body(x, y, r, h)):
                i += 1

        return

    def to_mjspec(self) -> mujoco.MjSpec:
        scene_spec = mujoco.MjSpec()
        world_mjc = scene_spec.worldbody

        world_mjc.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[50, 50, 0.1])
        world_mjc.add_light(pos=[0, 0, 10], dir=[0, 0, -1])

        for b in self.bodies:
            if isinstance(b, Goal):
                world_mjc.add_geom(
                    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=[b.r, b.h / 2.0, 0.0],  # [promień, połowa wysokości, 0]
                    pos=[b.x, b.y, b.h / 2.0],
                    rgba=[0.2, 0.8, 0.2, 1],
                )

            if not isinstance(b, Goal):
                world_mjc.add_geom(
                    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=[b.r, b.h / 2.0, 0.0],  # [promień, połowa wysokości, 0]
                    pos=[b.x, b.y, b.h / 2.0],
                    rgba=[0.8, 0.2, 0.2, 1],
                )

        return scene_spec

    def view(self):
        fig, ax = plt.subplots(figsize=(6, 6))

        for b in self.bodies:
            c = "black"

            if isinstance(b, Goal):
                c = "green"
            elif isinstance(b, Body):
                c = "red"

            tmp = Circle(
                (b.x, b.y), b.r, color=c, fill=True, linewidth=2, linestyle="-"
            )
            ax.add_patch(tmp)

        ax.add_patch(
            Circle(
                (0.0, 0, 0),
                self.safe_r,
                color="blue",
                fill=False,
                linewidth=2,
                linestyle=":",
            )
        )

        ax.set_aspect("equal")
        ax.set_xlim(-self.width / 2, self.width / 2)
        ax.set_ylim(-self.height / 2, self.height / 2)

        plt.title("WORLD")
        plt.grid(True)
        plt.show()


# w = World(100, 100)
# w.add_goal(Goal(25, 25))
# w.generate_obstacles(100, 1.0)
# w.view()
