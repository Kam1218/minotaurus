import numpy as np
import math
from scipy.spatial.transform import Rotation


class Controler:
    def samson(self, x, xr, dxr, k):
        e = xr - x

        ex = np.cos(x[2, 0]) * e[0, 0] + np.sin(x[2, 0]) * e[1, 0]

        ey = -np.sin(x[2, 0]) * e[0, 0] + np.cos(x[2, 0]) * e[1, 0]

        etheta = e[2, 0]
        etheta = math.atan2(math.sin(etheta), math.cos(etheta))

        vr = dxr[0, 0] ** 2 + dxr[1, 0] ** 2
        omegar = dxr[2, 0]

        v = vr * np.cos(etheta) + k[0, 0] * ex
        omega = omegar + k[1, 0] * vr * ey + k[2, 0] * vr * np.sin(etheta)

        return v, omega

    def kanayama(self, x, xr, dxr, k):
        e = xr - x

        # transf. błędu do ukł. robota
        R = Rotation.from_euler("z", -x[2, 0]).as_matrix()
        e = np.dot(R, e)

        # normalizacja -pi pi
        e[2, 0] = math.atan2(math.sin(e[2, 0]), math.cos(e[2, 0]))

        vr = np.sqrt(dxr[0, 0] ** 2 + dxr[1, 0] ** 2)
        omegar = dxr[2, 0]

        # Prawo sterowania Kanayamy
        v = vr * np.cos(e[2, 0]) + k[0, 0] * e[0, 0]
        omega = omegar + k[1, 0] * vr * e[1, 0] + k[2, 0] * np.sin(e[2, 0])

        return v, omega
