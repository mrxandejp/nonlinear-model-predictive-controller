# -*- coding: utf-8 -*-

import numpy as np


def calcUsteps(U, Nu, delta):

    dU = np.zeros((2, (Nu*4)))

    for i in range(0, Nu):
        #col 1 (v+d,vn,w)
        dU[0, 0+4*i] = U[0, i]+delta
        dU[1, 0+4*i] = U[1, i]
        #col 2 (v-d,vn,w)
        dU[0, 1+4*i] = U[0, i]-delta
        dU[1, 1+4*i] = U[1, i]
        #col 3 (v,vn+EditDelta,w)
        dU[0, 2+4*i] = U[0, i]
        dU[1, 2+4*i] = U[1, i]+delta
        #col 4 ...
        dU[0, 3+4*i] = U[0, i]
        dU[1, 3+4*i] = U[1, i]-delta

    return dU
