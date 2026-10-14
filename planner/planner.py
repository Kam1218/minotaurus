import numpy as np
import matplotlib.pyplot as plt


class Planner:
    def __init__(self):
        self.traj = lambda t: np.array([[0.0], [0.0], [0.0]])

    def get(self, t):
        return self.traf(t)

    def view(self, T):
        t_val = np.linspace(0, T, 200)

        x_list = []
        y_list = []
        theta_list = []

        for t in t_val:
            pose = self.traj(t)
            x_list.append(pose[0, 0])
            y_list.append(pose[1, 0])
            theta_list.append(pose[2, 0])

        x = np.array(x_list)
        y = np.array(y_list)
        theta = np.array(theta_list)

        skip = 10
        plt.quiver(
            x[::skip],
            y[::skip],
            np.cos(theta[::skip]),
            np.sin(theta[::skip]),
            color="red",
            scale=15,
            width=0.004,
        )

        plt.grid(True, linestyle="--", alpha=0.7)
        plt.axis("equal")

        plt.show()

    def circle(self, v, r, c=(0.0, 0.0)):

        omega = v / r

        self.traj = lambda t: np.array(
            [
                [r * np.cos(omega * t) + c[0]],
                [r * np.sin(omega * t) + c[1]],
                [omega * t + np.pi / 2],
            ]
        )


# pl = Planner()

# pl.circle(1, 3)

# pl.view(30)
