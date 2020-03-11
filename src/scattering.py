# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np


def calc_scattering(start_pos, angle, N):
    pos = start_pos
    scattering_positions = []
    for n in range(N):
        print(f'Scattering #{n}')
        traj = calc_trajectory(pos, angle)
        print(f'Trajectory {traj}')
        
        
    return scattering_positions


l1 = lambda x: x
l2 = lambda x: 0.25*x

d2r = lambda t: t * (np.pi/180)

xrange = np.arange(0, 20, 0.1)
y1 = [l1(x) for x in xrange]
y2 = [l2(x) for x in xrange]


starting_pos = [3, l2(3)]
starting_angle = 30 #degs

def calc_trajectory(pos, angle):
    m = 1 / np.tan(d2r(angle)) #Gradient
    x, y = pos
    c = -m*x
    l = lambda x: m*x+c
    y = [l(x) + y for x in xrange] 
    return y



    

y3 = calc_trajectory(starting_pos, starting_angle)

plt.plot(xrange, y1, label='bound_1')
plt.plot(xrange, y2, label='bound_2')
plt.plot(xrange, y3, label='trajectory', c='r')

plt.scatter(*starting_pos, label='starting pos')

plt.legend()