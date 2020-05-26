import numpy as np


def scaleForSaturation(U, d, Nu, vmax):

    Ua = np.zeros((2, Nu))

    for i in range(0, Nu):
        v = U[0, i]
        w = U[1, i]

        # Cinematica Inversa
        v1 = v + ((d*w)/2)
        v2 = v - ((d*w)/2)

        # Proportional Saturation
        maxv = max(v1, v2)
        minv = min(v1, v2)

        if maxv > vmax:
            scalemax = maxv / vmax
        else:
            scalemax = 1

        if minv < -vmax:
            scalemin = minv / (-vmax)
        else:
            scalemin = 1

        scale = max(scalemin, scalemax)

        v1a = v1 / scale
        v2a = v2 / scale

        # Cinematica Direta
        vf = (v1a + v2a) / 2
        wf = (v1a - v2a) / d

        Ua[0, i] = vf
        Ua[1, i] = wf
        pass

    return Ua
