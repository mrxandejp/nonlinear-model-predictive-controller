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

# def update_line(hl, x_data, y_data):
#     hl.set_xdata(np.append(hl.get_xdata(), x_data))
#     hl.set_ydata(np.append(hl.get_ydata(), y_data))
#     plt.draw()


def PointsInCircum(r, n=200):
    return [(math.cos(2*pi/n*x)*r-r, math.sin(2*pi/n*x)*r) for x in range(0, n+1)]


def OdometryValues(msg):
    global TRsx, TRsy, TRst
    TRsx = msg.pose.pose.position.x
    TRsy = msg.pose.pose.position.y
    TRst = 2 * math.atan2(msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w)


def VelocityValues(msg):
    global TRsx, TRsy
    TRsv = msg.linear.x
    TRsw = msg.angular.z


def CalcTetaVW(Vx, aX, Vy, aY):
    tetaRef = math.atan2(Vy, Vx)
    Vref = math.sqrt(pow(Vx, 2)+pow(Vy, 2))
    Wref = ((Vx*aY)-(Vy*aX))/(pow(Vx, 2)+pow(Vy, 2))
    return tetaRef, Vref, Wref


rospy.init_node('control_turtlebot')

velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
rospy.Subscriber('/odom', Odometry, OdometryValues, queue_size=1)
rospy.Subscriber('/mobile_base/commands/velocity', Twist, VelocityValues)
velMsg = Twist()
# rate = rospy.Rate(0.8)
rate = rospy.Rate(1)

circle_points = PointsInCircum(1)
circle_points = np.array(circle_points)

xref = circle_points[:, 0]
yref = circle_points[:, 1]

Vx = np.diff(xref)
aX = np.diff(Vx)

Vy = np.diff(yref)
aY = np.diff(Vy)

x_pecorrido = []
y_pecorrido = []

fig, gra = plt.subplots()
plotou = False
while not rospy.is_shutdown():
    if not plotou:
        for i in range(0, len(aX)):
            print("############################ COMECO ############################")

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
                Xrefp, Yrefp, TRsx, TRsy, TRst, TRsv, TRsw, Xref, Yref, PHIref, Vref, Wref, VXrefp, VYrefp)

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
            print("############################ FIM  ############################")
            print(" ")

        gra.plot(xref, yref, 'bo', x_pecorrido, y_pecorrido, 'rx')
        gra.grid()
        # fig.savefig("test.png")
        plt.show()
        plotou = True
