# -*- coding: utf-8 -*-

import math
from diffAngle import diffAngle
import numpy as np

PI = math.pi


def costFunction(SimRobx, SimRoby, SimRobteta, SimRobv, SimRobw, Ut, SimTargetx, SimTargety, SimTargetvx, SimTargetvy, N1, Np, Nu, L1, L2, L3):

    sum_cost = 0

    # Others
    sim_time_step = 0.01
    bfc = 1
    dval = 0

    for i in range(N1, Np+1):  # Executa 10 vezes
        # corrigindo problemas com angulos
        if i <= Nu:
            v = Ut[0, i-1]
            w = Ut[1, i-1]
        else:
            v = Ut[0, Nu-1]
            w = Ut[1, Nu-1]

        for j in range(0, 4):
            cteta = math.cos(SimRobteta)
            steta = math.sin(SimRobteta)

            if SimRobteta > PI:
                SimRobteta = SimRobteta - 2*PI

            SimRobteta = SimRobteta + w*sim_time_step
            SimRobx = SimRobx + sim_time_step*(v*cteta)
            SimRoby = SimRoby + sim_time_step*(v*steta)

            SimTargetx = SimTargetx + sim_time_step*SimTargetvx
            SimTargety = SimTargety + sim_time_step*SimTargetvy

            SimTargetvx = SimTargetvx * bfc
            SimTargetvy = SimTargetvy * bfc

        RobotTargetDist = math.sqrt(
            pow((SimTargetx - SimRobx), 2) + pow((SimTargety - SimRoby), 2))

        RBx = (SimTargetx - SimRobx) / RobotTargetDist
        RBy = (SimTargety - SimRoby) / RobotTargetDist

        RobotTargetAngle = math.atan2(RBy, RBx)

        sum_cost = sum_cost + L1*abs(dval - RobotTargetDist)

        sum_cost = sum_cost + L2*abs(diffAngle(RobotTargetAngle, SimRobteta))
        # sum_cost = sum_cost + L2*diffAngle(RobotTargetAngle, SimRobteta)

        pass
    # ALEXANDRE
    deltaU = L3*(abs(SimRobv - Ut[0, 0]) + abs(SimRobw - Ut[1, 0]))
    J = (sum_cost + deltaU)

    return J
