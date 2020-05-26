import numpy as np
import math
from diffAngle import diffAngle

# Cálculo da trajetória de referência

PI = math.pi


def calcRefTraj(lTi, SRx, SRy, SRt, trajX, trajY, trajTeta, V, W, Np, trajXp, trajYp):
    N = 2001
    i = 0
    deltaD = V*0.04  # 0.1 para Pioneer e 0.04 para Turtlebot
    deltaW = W*0.04
    dTotal = 0

    trajPX = np.zeros((Np+1))
    trajPY = np.zeros((Np+1))
    trajPTeta = np.zeros((Np+1))

    # angle of current segment to world
    teta = math.atan2(trajYp-trajY, trajXp-trajX)

    # angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
    al = diffAngle(trajTeta, SRt)-teta
    d1_3 = math.sqrt(((trajX - SRx) * (trajX - SRx)) +
                     ((trajY - SRy) * (trajY - SRy)))

    # distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
    dl = d1_3 * math.cos(al)

    segmentSize = math.sqrt(
        ((trajXp - trajX) * (trajXp - trajX)) + ((trajYp - trajY) * (trajYp-trajY)))

    # sum of distances from segments (of main trajectory)
    dSegments = segmentSize - dl

    # initial point of predictive controller trajectory
    trajPX[i] = trajX + (dl * math.cos(teta))
    trajPY[i] = trajY + (dl * math.sin(teta))
    trajPTeta[i] = trajTeta

    for j in range(1, len(Np)):
        # reached the end of the trajectory
        if lTi >= N-1:
            trajPX[j] = trajX
            trajPY[j] = trajY
            trajPTeta[j] = trajTeta
            pass
        else:
            # add trajectory points along current segment
            trajPX[j] = trajPX[j-1] + deltaD * math.cos(teta)
            trajPY[j] = trajPY[j-1] + deltaD * math.sin(teta)
            trajPTeta[j] = trajPTeta[j-1] + deltaW

            dTotal = dTotal + deltaD

            # change segment of the main trajectory thats being tracked
            if dTotal >= dSegments:
                lTi = lTi + 1
                teta = math.atan2(trajYp-trajY, trajXp-trajX)
                segmentSize = math.sqrt(
                    ((trajXp-trajX) * (trajXp-trajX)) + ((trajYp-trajY) * (trajYp-trajY)))

                # add point (already in next segment)
                trajPX[j] = trajX + (dTotal-dSegments) * math.cos(teta)
                trajPY[j] = trajY + (dTotal-dSegments) * math.sin(teta)
                trajPTeta[j] = trajTeta

                dSegments = dSegments + segmentSize

    # handle teta references
    for ab in range(0, Np):
        if trajPTeta[ab] > PI:
            trajPTeta[ab] = trajPTeta[ab] - (2 * PI)
            pass

    return trajPX, trajPY, trajPTeta
