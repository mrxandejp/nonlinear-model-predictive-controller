import numpy as np
from calcRefTraj import calcRefTraj
from scaleForSaturation import scaleForSaturation
from costFunction import costFunction
from calcUsteps import calcUsteps


def NMPC(Xrefp, Yrefp, TRsx, TRsy, TRst, TRsv, TRsw, Xref, Yref, PHIref, Vref, Wref):
    # Inicializando parâmetros do controlador
    Vmax = 0.4
    d = 0.23
    N1 = 1
    Np = 10  # Horizonte de predição
    Nu = 2  # Horizonte de controle

    # L1, L2 e L3 são os pesos para cada componente da função de custo
    L1 = 10
    L2 = 2000
    L3 = 0.05

    eta = 0.1
    Imax = 15
    I = 0
    Delta = 0.1

    # Inicializando parâmetro do otimizador
    alpha = 0.015

    # Inicializando vetores do otimizador
    Jgrad = np.zeros((4, 1))
    Jgrad_prev = np.zeros((4, 1))
    Jsteps = np.zeros((8, 1))

    # Inicializando variáveis de controle
    Ubest = np.zeros((2, 2))
    Uref = np.zeros((2, 2))
    Uaux = np.zeros((2, 2))

    # Criando uma estrutura em forma de dicionários para guardar o valor da iteração anterior do controlador
    SimRob = {'x': 0,
              'y': 4139,
              'teta': 0,
              'v': 0,
              'w': 0
              }

    # Inicialização das variáveis do controlador
    # Saída de controle desejada
    Uref = [
        [Vref, Vref],
        [Wref, Wref]
    ]

    # Cálculo da trajetória de referência
    lTi = 0

    tPX, tPY, tPTeta = calcRefTraj(
        lTi, TRsx, TRsy, TRst, Xref, Yref, PHIref, Vref, Wref, Np, Xrefp, Yrefp)

    # Atualiza o modelo do preditor
    SimRob['x'] = TRsx
    SimRob['y'] = TRsy
    SimRob['teta'] = TRst
    SimRob['v'] = TRsv
    SimRob['w'] = TRsw

    # Loop de controle
    # Reliza a saturação da velocidade das rodas com base na velocidade máxima
    Uref = scaleForSaturation(Uref, d, Nu, Vmax)

    Jatual = costFunction(SimRob['x'], SimRob['y'], SimRob['teta'], SimRob['v'],
                          SimRob['w'], Uref, tPX, tPY, tPTeta, N1, Np, Nu, L1, L2, L3)

    Jbest = Jatual

    # Loop de otimização do Ubest que minimiza o Jbest
    while (I < Imax) and (Jatual > eta):
        Usteps = calcUsteps(Uref, Nu, Delta)

        for k in range(0, Nu):
            for j in range(0, 4):
                # Atribui velocidade para cada passo U
                for m in range(0, Nu):
                    if m == k:
                        Uaux[0, m] = Usteps[0, (j+3*k)]
                        Uaux[1, m] = Usteps[1, (j+3*k)]
                    else:
                        Uaux[0, m] = Usteps[0, 0]
                        Uaux[1, m] = Usteps[1, 0]

                SimRob['x'] = TRsx
                SimRob['y'] = TRsy
                SimRob['teta'] = TRst
                SimRob['v'] = TRsv
                SimRob['w'] = TRsw

                Uaux = scaleForSaturation(Uaux, d, Nu, Vmax)

                J = costFunction(SimRob['x'], SimRob['y'], SimRob['teta'], SimRob['v'],
                                 SimRob['w'], Uaux, tPX, tPY, tPTeta, N1, Np, Nu, L1, L2, L3)
                Jsteps[j + (3*k), 0] = J

        # Cálculo do gradiente de J baseado no Jsteps
        #Jgrad[3, 0]
        for h in range(0, Nu):
            Jgrad_prev[0+(2*h), 0] = Jgrad[0+(2*h), 0]
            Jgrad_prev[0+(2*h), 0] = Jsteps[0+(4*h), 0] - Jsteps[1+(4*h), 0]
            Jgrad_prev[1+(2*h), 0] = Jgrad[1+(2*h), 0]
            Jgrad_prev[1+(2*h), 0] = Jsteps[2+(4*h), 0] - Jsteps[4+(4*h), 0]

        # Gradiente conjugado Algoritmo de Polak-Bibieri
        di = [0, 0]
        x1 = di[:]

        for z in range(0, Nu):
            di[0] = Jgrad[(2*z)+0, 0]
            di[1] = Jgrad[(2*z)+1, 0]

            x1[0] = (Uref[0, z] - (alpha * di[0]))
            x1[1] = (Uref[1, z] - (alpha * di[1]))

            Jgrad_prev[(2*z)+0, 0] = Jgrad[(2*z)+0, 0]
            Jgrad[(2*z)+0, 0] = Jsteps[(4*z)+0, 0] - Jsteps[(4*z)+1, 0]

            Jgrad_prev[(2*z)+1, 0] = Jgrad[(2*z)+1, 0]
            Jgrad[(2*z)+1, 0] = Jsteps[(4*z)+2, 0] - Jsteps[(4*z)+3, 0]

            beta = 0

            if (Jgrad[(2*z)+0, 0] >= eta) or (Jgrad[(2*z)+1, 0] >= eta):

                t1 = Jgrad[(2*z)+0, 0] - Jgrad_prev[(2*z)+0, 0]
                t2 = Jgrad[(2*z)+1, 0] - Jgrad_prev[(2*z)+1, 0]

                a1 = Jgrad[(2*z)+0, 0] * t1
                a2 = Jgrad[(2*z)+1, 0] * t2

                b1 = Jgrad_prev[(2*z)+0, 0] * Jgrad_prev[(2*z)+0, 0]
                b2 = Jgrad_prev[(2*z)+1, 0] * Jgrad_prev[(2*z)+1, 0]

                beta = (a1 + a2) / (b1 + b2)

            Uref[0, z] = x1[0] + \
                (alpha * (-Jgrad[(2*z)+0, 0] + (beta * Jgrad[(2*z)+0, 0])))
            Uref[1, z] = x1[1] + \
                (alpha * (-Jgrad[(2*z)+1, 0] + (beta * Jgrad[(2*z)+1, 0])))

        SimRob['x'] = TRsx
        SimRob['y'] = TRsy
        SimRob['teta'] = TRst
        SimRob['v'] = TRsv
        SimRob['w'] = TRsw

        # ix SATURA A VELOCIDADE DAS RODAS QUE SERÃO USADAS NO PRÓXIMO LOOP DE CONTROLE (WHILE)

        Uref = scaleForSaturation(Uref, d, Nu, Vmax)

        Jatual = costFunction(SimRob['x'], SimRob['y'], SimRob['teta'], SimRob['v'],
                              SimRob['w'], Uref, tPX, tPY, tPTeta, N1, Np, Nu, L1, L2, L3)

        if Jatual < Jbest:
            Jbest = Jatual
            Ubest = Uref

        I = I+1

    # FIM DO WHILE

    Vout_MPC = Ubest[0, 0]
    Wout_MPC = Ubest[1, 0]
    return Vout_MPC, Wout_MPC
