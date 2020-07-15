# -*- coding: utf-8 -*-

import numpy as np


def scaleForSaturation(U, d_Rob, Nu, vmax):

    Ua = np.zeros((2, Nu))
    # U = np.array(U)

    for i in range(0, Nu):
        v = U[0, i]
        w = U[1, i]

        # Cinematica Inversa
        v1 = v + ((d_Rob*w)/2)
        v2 = v - ((d_Rob*w)/2)

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

        scale = max(scalemax, scalemin)

        v1 = v1 / scale
        v2 = v2 / scale

        Ua[0, i] = (v1 + v2) / 2
        Ua[1, i] = (v1 - v2) / d_Rob
        pass

    return Ua
