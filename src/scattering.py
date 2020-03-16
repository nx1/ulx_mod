# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np


class Photon:
    def __init__(self, pos, angle):
        self.pos = pos
        self.angle = pos
    def calc_trajectory(xrange):
        return None
    
    def calc_scattering(N, xrange):
        return None

class Line2D:
    def __init__(self, m, c):
        self.m = m
        self.c = c
        self.func = lambda x: self.m * x + self.c
        self.xdata = None
        self.ydata = None
            
    def create_data(self, start, stop, num):
        self.xdata = np.linspace(start, stop, num)
        self.ydata = self.func(self.xdata)
        
    def plot(self):
        plt.plot(self.xdata, self.ydata)
        


def calc_gradient(angle, deg=False):
    """Calculate the gradient (m) based on an angle."""
    if deg:
        m = 1 / np.tan(d2r(angle))
    else:
        m = 1 / np.tan(angle)
    return m
    

def calc_trajectory(pos, angle):
    """
    Calculate the trajectory based on a starting position and angle.
    
    Parameters
    ----------
    pos : 1d array-like
        Starting position [x,y]
    angle : float
        Starting angle in (degrees for now)
    
    Returns
    -------
    trajectory : Line2D
        Line2D object corresponding to the trajectory
    """
    m = calc_gradient(angle, deg=True)
    x, y = pos
    c = -m*x
    trajectory = Line2D(m=m, c=c)
    return trajectory


def calc_scattering(start_pos, angle, N, xrange):
    pos = start_pos
    scattering_positions = []
    for n in range(N):
        print(f'Scattering #{n}')
        traj = calc_trajectory(pos, angle)
        
        intersections = find_intersections(traj, l1)
        
        print(f'Trajectory {traj}')
        
    return scattering_positions

def find_intersections(line1, line2):
    """
    Find the intersections between two lines.
    
    Parameters
    ----------
    line1, line2 : 
        
        np linalg example
        3 * x0 + x1 = 9
        x0 + 2 * x1 = 8
        [3, 1] = 9
        [1, 2] = 8
        
        y = mx + c
        y - m1 * x = c1
        y - m2 * x = c2
        
        [m1, 1] = c1
        [m2, 1] = c2
        
        let c1 = -5
            m1 = 2
            c2 = 3
            m2 = -4
        
    """
    a = np.array([[2, 1],
                  [-4, 1]])
    b = np.array([-5, 3])
    x, y = np.linalg.solve(a, b)
    
    
    

def plot():
    l1.plot()
    l2.plot()
    
    plt.scatter(*starting_pos, label='starting pos')
    
    
    plt.show()


l1 = Line2D(m=1, c=0)
l2 = Line2D(m=0.25, c=0)

l1.create_data(start=0, stop=50, num=100)
l2.create_data(start=0, stop=50, num=100)


d2r = lambda t: t * (np.pi/180) #degrees to radians


starting_pos = [3, l2(3)]
starting_angle = 30 #degs

y3 = calc_trajectory(starting_pos, starting_angle, xrange)


plot()