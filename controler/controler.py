import numpy as np


class Controler:
    def samson(self, x, xr, dxr, k):
        e = xr - x

        ex = np.cos(x[2, 0]) * e[0, 0] + np.sin(x[2, 0]) * e[1, 0]

        ey = -np.sin(x[2, 0]) * e[0, 0] + np.cos(x[2, 0]) * e[1, 0]

        etheta = e[2, 0]

        vr = dxr[0, 0] ** 2 + dxr[1, 0] ** 2
        omegar = dxr[2, 0]

        v = vr * np.cos(etheta) + k[0, 0] * ex
        omega = omegar + k[1, 0] * vr * ey + k[2, 0] * vr * np.sin(etheta)

        return v, omega
