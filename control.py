#! /usr/bin/env python

import matplotlib
# matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nmpc import Nmpc
from calcUsteps import calcUsteps

INTERVALOS = 15
pi = math.pi
Xrefp = 0
Yrefp = 0
TRst = 0
TRsv = 0
TRsw = 0
Xref = 0
Yref = 0
PHIref = 0
Vref = 0
Wref = 0
TRsx = 0
TRsy = 0

# L1, L2 e L3 sao os pesos para cada componente da funcao de custo
# Pesos em linha reta
L1 = 800
L2 = 600
L3 = 0.1

# Pesos na rotacao
L1_rot = 800
L2_rot = 600
L3_rot = 0.1

# x e y pecorrido
x_pecorrido = []
y_pecorrido = []

def OdometryValues(msg):
    global TRsx, TRsy, TRst
    TRsx = msg.pose.pose.position.x
    TRsy = msg.pose.pose.position.y
    TRst = 2 * math.atan2(msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w)

def VelocityValues(msg):
    global TRsv, TRsw
    TRsv = msg.linear.x
    TRsw = msg.angular.z


def CalcTetaVW(Vx, aX, Vy, aY):
    tetaRef_ = math.atan2(Vy, Vx)
    Vref_ = math.sqrt(pow(Vx, 2)+pow(Vy, 2))
    Wref_ = ((Vx*aY)-(Vy*aX))/(pow(Vx, 2)+pow(Vy, 2))
    return tetaRef_, Vref_, Wref_

def loopControl(Vx, Vy, aX, aY, xref, yref):
    global TRsx, TRsy, TRst, TRsv, TRsw, L1, L2, L3, x_pecorrido, y_pecorrido, velMsg, velPub, rate, Xref, Yref, PHIref, Vref, Wref
    for i in range(0, len(aX)):
        print(" ")

        print(i)
        Xrefp = xref[i]
        Yrefp = yref[i]
        VXrefp = Vx[i]
        VYrefp = Vy[i]

        print("Xrefp, Yrefp")
        print(Xrefp, Yrefp)
        print("TRsx, TRsy")
        print(TRsx, TRsy)

        Vout_MPC, Wout_MPC = Nmpc(
            Xrefp, Yrefp, TRsx, TRsy, TRst, TRsv, TRsw, Xref, Yref, PHIref, Vref, Wref, VXrefp, VYrefp, L1, L2, L3)

        velMsg.linear.x = Vout_MPC
        velMsg.angular.z = Wout_MPC
        # print("Vout_MPC, Wout_MPC")
        # print(Vout_MPC,Wout_MPC)
        velPub.publish(velMsg)

        rate.sleep()

        x_pecorrido.append(TRsx)
        y_pecorrido.append(TRsy)

        PHIref, Vref, Wref = CalcTetaVW(Vx[i], aX[i], Vy[i], aY[i])
        Xref = xref[i]
        Yref = yref[i]
        print(" ")

def rotControl(angulo):
    global TRsx, TRsy, TRst, TRsv, TRsw, L1_rot, L2_rot, L3_rot, velMsg, velPub, rate, Wref, pi

    anguloAnterior = TRst
    anguloRef = pi/2
    angulos = np.empty(0)
    wref = np.empty(0)
    if angulo == 90:
        angulos = np.linspace(0, anguloRef, 6)
        angulos = np.append(angulos, [anguloRef, anguloRef,anguloRef])
        wref = np.diff(angulos)
    elif angulo == -90:
        angulos = np.linspace(0, -anguloRef, 6)
        angulos = np.append(angulos, [anguloRef, anguloRef, anguloRef])
        wref = np.diff(angulos)
    else:
        anguloRef = pi
        angulos = np.linspace(0, -anguloRef, 12)
        angulos = np.append(angulos, [anguloRef, anguloRef, anguloRef])
        wref = np.diff(angulos)

    print("angulos ", angulos)
    # while abs(TRst - anguloAnterior) < anguloRef:
    for i in range(0, len(wref)):
        Vout_MPC, Wout_MPC = Nmpc(
                        0, 0, TRsx, TRsy, TRst, TRsv, TRsw, 0, 0, 0, 0, wref[i], 0, 0, L1_rot, L2_rot, L3_rot)
        velMsg.linear.x = 0
        velMsg.angular.z = Wout_MPC

        print("Valor TRst - anguloAnterior")
        print(abs(TRst - anguloAnterior))
        print("anguloRef ", anguloRef)
        velPub.publish(velMsg)

        rate.sleep()
        pass
    pass



rospy.init_node('control_turtlebot')

velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
rospy.Subscriber('/odom', Odometry, OdometryValues, queue_size=1)
rospy.Subscriber('/mobile_base/commands/velocity', Twist, VelocityValues)
velMsg = Twist()
# rate = rospy.Rate(0.8)
rate = rospy.Rate(0.8)

rotacao = [ False, False, True, False, True ]
angulo = [0, 90, 90]
# angulo = [0, 0, 90, 0, 90]
vet_direcao = ['R', 'U', 'L']
# vet_direcao = ['R', 'R', 'U', 'U', 'L']
x_vet = [0, 1, 1, 0]
y_vet = [0, 0, 1, 1]
# x_vet = [0, 1, 2, 2, 2, 1]
# y_vet = [0, 0, 0, 1, 2, 2]

x_trajetoria = []
y_trajetoria = []

fig, gra = plt.subplots()
plotou = False
while not rospy.is_shutdown():
    if not plotou:
        for j in range(0, len(vet_direcao)):

            if rotacao[j]:
                print("############################ COMECO ROTACAO " + str(angulo[j]) + " ############################")
                rotControl(angulo[j])
                print("############################ FIM ROTACAO " + str(angulo[j]) + " ############################")
                pass
            xref = np.empty(0)
            yref = np.empty(0)

            if vet_direcao[j] == 'R' or vet_direcao[j] == 'L':
                xref = np.linspace(x_vet[j], x_vet[j+1], INTERVALOS)
                # np.append(xref, x_vet[j+1])
                yref = np.linspace(y_vet[j], y_vet[j], INTERVALOS)

            if vet_direcao[j] == 'U' or vet_direcao[j] == 'D':
                xref = np.linspace(x_vet[j], x_vet[j], INTERVALOS)
                yref = np.linspace(y_vet[j], y_vet[j+1], INTERVALOS)
                # np.append(yref, y_vet[j+1])

            xref = np.append(xref, [x_vet[j+1], x_vet[j+1], x_vet[j+1], x_vet[j+1]])
            yref = np.append(yref, [y_vet[j+1], y_vet[j+1], y_vet[j+1], y_vet[j+1]])
            print(xref)

            x_trajetoria.append(xref)
            y_trajetoria.append(yref)

            Vx = np.diff(xref)
            aX = np.diff(Vx)

            Vy = np.diff(yref)
            aY = np.diff(Vy)

            print("############################ COMECO QUADRANTE " + str(j) + " ############################")
            loopControl(Vx, Vy, aX, aY, xref, yref)
            print("############################ FIM QUADRANTE " + str(j) + " ############################")

        gra.plot(x_trajetoria, y_trajetoria, 'bo',
                 x_pecorrido, y_pecorrido, 'rx')
        gra.grid()
        # fig.savefig("test.png")
        plt.show()
        plotou = True
