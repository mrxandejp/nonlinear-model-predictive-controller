import math
from diffAngle import diffAngle

PI = math.pi


def costFunction(SimRobx, SimRoby, SimRobteta, SimRobv, SimRobw, Uref, tPX, tPY, tPTeta, N1, Np, Nu, L1, L2, L3):
    sumX = 0
    sumY = 0
    sumT = 0
    #sumDU  = 0
    t = 0.01

    for i in range(N1, Np+1):  # Executa 10 vezes
        # corrigindo problemas com angulos
        if i <= Nu:
            v = Uref[0, i-1]
            w = Uref[1, i-1]
        else:
            v = Uref[0, Nu]
            w = Uref[1, Nu]

        j = 0
        while j < 4:
            if SimRobteta > PI:
                SimRobteta = SimRobteta - 2*PI

            SimRobteta = SimRobteta + t*w
            SimRobx = SimRobx + t * math.cos(SimRobteta) * v
            SimRoby = SimRoby + t * math.sin(SimRobteta) * v
            j += 1

        sumX = sumX + pow((tPX[i-1] - SimRobx), 2)
        sumY = sumY + pow((tPY[i-1] - SimRoby), 2)

        #erroTeta = diffAngle(tPTeta,SimRobteta)
        erroTeta = diffAngle(tPTeta[i-1], SimRobteta)
        sumT = sumT + pow(erroTeta, 2)
        pass

    sumDU = pow((SimRobv - Uref[0, 0]), 2) + pow((SimRobw - Uref[0, 1]), 2)
    # Horizonte de controle, por isso fica fora do FOR
    J = (L1 * (sumX + sumY)) + (L2 * sumT) + (L3 * sumDU)

    return J
