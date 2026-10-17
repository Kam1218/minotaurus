import numpy as np
import matplotlib.pyplot as plt
import math


class Planner:
    def __init__(self):
        self.traj = (
            lambda t: np.array([[0.0], [0.0], [0.0]]),
            np.array([[0.0], [0.0], [0.0]]),
            True,
        )

    def get(self, t):
        return self.traj(t)

    def view(self, t_step=0.1):

        x_list = []
        y_list = []
        theta_list = []

        t = 0
        while True:
            pose, _, end = self.traj(t)
            t += t_step
            x_list.append(pose[0, 0])
            y_list.append(pose[1, 0])
            theta_list.append(pose[2, 0])

            if end:
                break

        x = np.array(x_list)
        y = np.array(y_list)
        theta = np.array(theta_list)

        # skip = 10
        # plt.quiver(
        #     x[::skip],
        #     y[::skip],
        #     np.cos(theta[::skip]),
        #     np.sin(theta[::skip]),
        #     color="red",
        #     scale=15,
        #     width=0.004,
        # )

        plt.scatter(x, y, color="red", linewidths=1)
        plt.scatter(x[0], y[0], color="green", linewidths=3)

        plt.grid(True, linestyle="--", alpha=0.7)
        plt.axis("equal")

        plt.show()

    def circle(self, v, r, c=(0.0, 0.0)):

        omega = v / r

        T = 2 * np.pi / omega

        x0, y0 = c

        def traj_func(t):
            end = False if 0 <= t <= T else True

            if end:
                x, dx, _ = traj_func(T)
            else:
                x = np.array(
                    [
                        [r * np.cos(omega * t) + x0],
                        [r * np.sin(omega * t) + y0],
                        [omega * t + np.pi / 2],
                    ]
                )
                dx = np.array(
                    [
                        [v * np.cos(omega * t + np.pi / 2)],
                        [v * np.sin(omega * t + np.pi / 2)],
                        [omega],
                    ]
                )

            return x, dx, end

        self.traj = traj_func

    def square(self, v, a, phi=0.0, c=(0.0, 0.0)):
        x0, y0 = c
        t_side = a / v

        T = 4 * t_side

        def traj_func(t):
            end = False if 0 <= t <= T else True

            if end:
                xr, dxr, _ = traj_func(T)
            else:

                if t <= t_side:
                    s = t * v
                    x_local = -a / 2 + s
                    y_local = -a / 2
                    theta_local = 0.0
                    dx_local, dy_local, dtheta_local = v, 0.0, 0.0

                elif t <= 2 * t_side:
                    dt = t - (t_side)
                    s = dt * v
                    x_local = a / 2
                    y_local = -a / 2 + s
                    theta_local = np.pi / 2
                    dx_local, dy_local, dtheta_local = 0.0, v, 0.0

                elif t <= 3 * t_side:
                    dt = t - (2 * t_side)
                    s = dt * v
                    x_local = a / 2 - s
                    y_local = a / 2
                    theta_local = np.pi
                    dx_local, dy_local, dtheta_local = -v, 0.0, 0.0

                elif t <= 4 * t_side:
                    dt = t - (3 * t_side)
                    s = dt * v
                    x_local = -a / 2
                    y_local = a / 2 - s
                    theta_local = 3 * np.pi / 2
                    dx_local, dy_local, dtheta_local = 0.0, -v, 0.0

                # rot phi
                cos_phi = np.cos(phi)
                sin_phi = np.sin(phi)

                x_glob = x0 + x_local * cos_phi - y_local * sin_phi
                y_glob = y0 + x_local * sin_phi + y_local * cos_phi
                theta_glob = theta_local + phi

                theta_glob = math.atan2(math.sin(theta_glob), math.cos(theta_glob))

                dx_glob = dx_local * cos_phi - dy_local * sin_phi
                dy_glob = dx_local * sin_phi + dy_local * cos_phi
                dtheta_glob = dtheta_local

                xr = np.array([[x_glob], [y_glob], [theta_glob]])
                dxr = np.array([[dx_glob], [dy_glob], [dtheta_glob]])

            return xr, dxr, end

        self.traj = traj_func


# pl = Planner()
# pl.circle(0.5, 5.0)
# pl.square(0.5, 5.0 / np.sqrt(2), 3 * np.pi / 4.0)


# pl.view()
