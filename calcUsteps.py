import numpy as np


def calcUsteps(U, Nu, delta):

    dU = np.zeros((2, (Nu*4)))

    for i in range(0, Nu):
        dU[0, 0+3*i] = U[0, i]+delta
        dU[1, 0+3*i] = U[1, i]
        dU[0, 1+3*i] = U[0, i]-delta
        dU[1, 1+3*i] = U[1, i]
        dU[0, 2+3*i] = U[0, i]
        dU[1, 2+3*i] = U[1, i]+delta
        dU[0, 3+3*i] = U[0, i]
        dU[1, 3+3*i] = U[1, i]-delta

    return dU
