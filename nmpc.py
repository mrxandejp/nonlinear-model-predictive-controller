# -*- coding: utf-8 -*-

import numpy as np
from calcRefTraj import calcRefTraj
from scaleForSaturation import scaleForSaturation
from costFunction import costFunction
from calcUsteps import calcUsteps

def mysign(x):
    if x > 0.0:
        return 1
    if x < 0.0:
        return -1
    return 0

def Nmpc(Xrefp, Yrefp, TRsx, TRsy, TRst, TRsv, TRsw, Xref, Yref, PHIref, Vref, Wref,VXrefp, VYrefp, L1, L2, L3):
    # Inicializando parâmetros do controlador
    Vmax = 0.5
    d_Rob = 0.2236
    N1 = 1
    Np = 10  # Horizonte de predição
    Nu = 2  # Horizonte de controle

    # # L1, L2 e L3 são os pesos para cada componente da função de custo
    # L1 = 800
    # L2 = 600
    # L3 = 0.1

    #RPROP
    JStop = 0.05
    Imax = 15
    delta = 0.1
    eta_M = 0.5
    eta_P = 1.2
    step_Max = 0.1
    step_Min = 0.000001
    curr_Step = 0.1

    # Inicializando vetores do otimizador
    Jsteps = np.zeros((8, 1))
    Jgradient = np.zeros((4, 1))
    Jgradient_prev = np.zeros((4, 1))
    Jsteps_prev = np.empty([4,1])
    Jsteps_prev.fill(0.1)    
    iterationCount = 0
    prevGrad = 0
    currStep = 0

    Uaux = np.zeros((2, 2))
    # Uref = np.zeros((2, 2))
    Ubest = np.zeros((2, 2))
    Usteps = np.zeros((2, 8))

    simRobot = {'x': 0,
              'y': 0,
              'teta': 0,
              'v': 0,
              'w': 0
              }

    simTarget = {'x': 0,
              'y': 0,
              'vx': 0,
              'vy': 0
              }

    Uref = np.array([
        [Vref, Vref],
        [Wref, Wref]
    ])
    
    simRobot['x'] = TRsx
    simRobot['y'] = TRsy
    simRobot['teta'] = TRst
    simRobot['v'] = TRsv
    simRobot['w'] = TRsw

    simTarget['x'] = Xrefp
    simTarget['y'] = Yrefp
    simTarget['vx'] = VXrefp
    simTarget['vy'] = VYrefp
    
    Uref = scaleForSaturation(Uref, d_Rob, Nu, Vmax)

    Jcurrent = costFunction(simRobot['x'], simRobot['y'], simRobot['teta'], simRobot['v'],
                          simRobot['w'], Uref,simTarget['x'], simTarget['y'] ,simTarget['vx'], simTarget['vy'] , N1, Np, Nu, L1, L2, L3)

    Jbest = Jcurrent

    #---------------------------------------------------
    #              Optimization loop
    #---------------------------------------------------
    while (iterationCount < Imax) and (Jcurrent > JStop):
        #get Usteps matrix
        Usteps = calcUsteps(Uref, Nu, delta)

        #Calculate Jsteps vector (do one simulation for each input set)
        for k in range(0, Nu):
            for j in range(0, 4):
                for m in range(0, Nu):
                    if m == k:
                        Uaux[0, m] = Usteps[0, (j+4*k)]
                        Uaux[1, m] = Usteps[1, (j+4*k)]
                    else:
                        Uaux[0, m] = Uref[0, 0]
                        Uaux[1, m] = Uref[1, 0]

                #Reset robot initial state for each simulation
                simRobot['x'] = TRsx
                simRobot['y'] = TRsy
                simRobot['teta'] = TRst
                simRobot['v'] = TRsv
                simRobot['w'] = TRsw

                simTarget['x'] = Xrefp
                simTarget['y'] = Yrefp
                simTarget['vx'] = VXrefp
                simTarget['vy'] = VYrefp

                #Limit wheel speed references in case of motor saturation (update references)
                Uaux = scaleForSaturation(Uaux, d_Rob, Nu, Vmax)
                
                #Do simulation with current Uaux and add to Jsteps vector
                #Switches between trajectory controller and formation controller
                J = costFunction(simRobot['x'], simRobot['y'], simRobot['teta'], simRobot['v'],
                          simRobot['w'], Uaux,simTarget['x'], simTarget['y'] ,simTarget['vx'], simTarget['vy'] , N1, Np, Nu, L1, L2, L3)

                #Add J to Jsteps
                Jsteps[j + (4*k), 0] = J
        
        #Compute gradient of J from Jsteps
        for i in range(0, Nu):
            Jgradient_prev[2*i+0,0] = Jgradient[2*i+0,0]
            Jgradient[2*i+0,0] = Jsteps[4*i+0,0] - Jsteps[4*i+1,0]

            Jgradient_prev[2*i+1,0] = Jgradient[2*i+1,0]
            Jgradient[2*i+1,0] = Jsteps[4*i+2,0] - Jsteps[4*i+3,0]

        #Minimization algorithm
        for i in range(0, Nu):
            for j in range(0, 2):
                prevStep = Jsteps_prev[2*i+j,0]
                currGrad = Jgradient[2*i+j,0]

                if prevGrad*currGrad > 0:
                    curr_Step = min(prevStep*eta_P, step_Max)
                    Uref[j,i] = (Uref[j,i] - (mysign(currGrad)*currStep))
                    prevGrad = currGrad
                else:
                    if prevGrad*currGrad < 0:
                        currStep = max(prevStep*eta_M, step_Min)
                        prevGrad = 0
                    
                    if prevGrad*currGrad == 0:
                        Uref[j,i] = (Uref[j,i] - (mysign(currGrad)*prevStep))
                        prevGrad = currGrad
                    
                    Jsteps_prev[2*i+j,0] = currStep

        #Reset robot initial state for each simulation
        simRobot['x'] = TRsx
        simRobot['y'] = TRsy
        simRobot['teta'] = TRst
        simRobot['v'] = TRsv
        simRobot['w'] = TRsw

        simTarget['x'] = Xrefp
        simTarget['y'] = Yrefp
        simTarget['vx'] = VXrefp
        simTarget['vy'] = VYrefp

        Uref = scaleForSaturation(Uref, d_Rob, Nu, Vmax)

        #Calculate new current cost (do simulation)
        Jcurrent = costFunction(simRobot['x'], simRobot['y'], simRobot['teta'], simRobot['v'],
                          simRobot['w'], Uref,simTarget['x'], simTarget['y'] ,simTarget['vx'], simTarget['vy'] , N1, Np, Nu, L1, L2, L3)

        #Update JBest
        if Jcurrent < Jbest:
            Jbest = Jcurrent
            Ubest[0,0] = Uref[0,0]
            Ubest[1,0] = Uref[1,0]

        iterationCount = iterationCount+1

    # FIM DO WHILE

    Vout_MPC = Ubest[0, 0]
    Wout_MPC = Ubest[1, 0]
    return Vout_MPC, Wout_MPC
