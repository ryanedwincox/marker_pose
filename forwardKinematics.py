import numpy as np
from sympy import *
import math as m

def main():
    # create physical arm parmeters
    # thetas in radians
    t1 = m.pi/4
    t2 = m.pi/4
    t3 = m.pi/4
    t4 = m.pi/4
    t5 = m.pi/4
    t6 = m.pi/4

    # Lengths in meters
    r0 = 0.05
    r2 = 0.08
    r3 = 0.01

    d0 = 0.05
    d4 = 0.02
    d5 = 0.05

    l2 = 0.5
    l3 = 0.3
    l4 = 0.3

    # create DH table in matricies
    # 0    , 1, 2, 3
    # alpha, a, d, theta
    dh0 = np.array([0, 0, 0, t1])
    dh1 = np.array([m.pi/2, r0, d0, t2])
    dh2 = np.array([0, l2, 0, t3])
    dh3 = np.array([m.pi/2, r2, 0, t4])
    dh4 = np.array([-m.pi/2, r3, -l3, t5])
    dh5 = np.array([m.pi/2, l4, d4, t6])
    dh6 = np.array([0, 0, d5, 0])

    T01 = createTransformFromDH(dh0)
    T12 = createTransformFromDH(dh1)
    T23 = createTransformFromDH(dh2)
    T34 = createTransformFromDH(dh3)
    T45 = createTransformFromDH(dh4)
    T56 = createTransformFromDH(dh5)
    T67 = createTransformFromDH(dh6)

    T07 = T01*T12*T23*T34*T45*T56*T67
    print str(T07)

# 0    , 1, 2, 3
# alpha, a, d, theta
def createTransformFromDH(dh):
    T = np.matrix([[m.cos(dh[3]), -m.sin(dh[3]), 0, dh[1]],
                [m.sin(dh[3])*m.cos(dh[0]), m.cos(dh[3])*m.cos(dh[0]), -m.sin(dh[0]), -m.sin(dh[0])*dh[2]],
                [m.sin(dh[3])*m.sin(dh[0]), m.cos(dh[3])*m.sin(dh[0]), m.cos(dh[0]), m.cos(dh[0])*dh[2]],
                [0, 0, 0, 1]])
    return T

main()
