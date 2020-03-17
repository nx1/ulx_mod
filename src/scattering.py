# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

from auxil import d2r, r2d #angle conversions


class Line2D:
    def __init___(self):
        self.m = None
        self.c = None
        self.func = None
        self.angle = None


    @classmethod
    def from_function(cls, m, c):
        """Create line object from gradient and y-intersection."""
        self.m = m
        self.c = c
        self.func = lambda x: self.m * x + self.c
        self.angle = np.arcsin(m)

        line2d = cls(first_name, last_name)
        return line2d

    @classmethod
    def from_points(cls, pos1, pos2):
        """
        Create line object from two datapoints.
        
        Parameters
        ----------
        pos1, pos2 : [x, y]
        """
        self.m = np.sin(angle)
        self.c = c
        self.func = lambda x: self.m * x + self.c
        self.angle = np.arcsin(m)

        line2d = cls(first_name, last_name)
        return line2d

    @classmethod
    def from_pos_angle(cls, pos, angle):
        """
        Create a line from a position and angle.

        Parameters
        ----------
        pos : [x, y]
            position
        angle : float
            Angle in radians.
        """
        self.m = np.sin(angle)
        self.c = c
        self.func = lambda x: self.m * x + self.c
        self.angle = np.arcsin(m)

        line2d = cls(first_name, last_name)
        return line2d


    def create_data(self, start, stop, num):
        """Create datapoints between a specified range."""
        self.xdata = np.linspace(start, stop, num)
        self.ydata = self.func(self.xdata)

    def plot(self, **kwargs):
        plt.plot(self.xdata, self.ydata, **kwargs)


def calc_gradient(angle, deg=False):
    """Calculate the gradient (m) based on an angle."""
    if deg:
        m = 1 / np.tan(d2r(angle))
    else:
        m = 1 / np.tan(angle)
    return m
    

def calc_trajectory(x, y, angle):
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
    c = -m*x
    trajectory = Line2D(m=m, c=c)
    return trajectory


def calc_scattering(starting_trajectory, N):
    trajectory = starting_trajectory
    scattering_positions = []
    angle = 0
    for n in range(N):
        print(f'Scattering #{n}')

        l1_int = find_intersection(trajectory, l1)
        l2_int = find_intersection(trajectory, l2)

        print('l1 intersection:', l1_int)
        print('l2 intersection:', l2_int)

        plt.scatter(*l1_int, label='l1_intersection')
        plt.scatter(*l2_int, label='l2_intersection')

        angle = angle_between(trajectory, l2)
        print('old trajectory angle:', trajectory.angle)

        print('angle between tracjecory and l2:', angle)

        angle = calc_new_angle(trajectory.angle, 2*angle)

        print('new_angle:', angle)

        trajectory = calc_trajectory(*l1_int, angle)

        scattering_positions.append(l1_int)
        trajectory.create_data(start=l1_int[0], stop=50, num=100)
        trajectory.plot(label='traj')
        print(f'Trajectory {trajectory}')

    return scattering_positions

def calc_new_angle(angle, by):
    return angle + by

def find_intersection(line1, line2):
    """
    Find the intersection between two straight lines.
    Using np.linalg.solve()
    
    Parameters
    ----------
    line1, line2 : Line2D
        line objects to calculate intersection for.

    Notes:
        Two straight lines can only have 1 intersection.

    Examples
    --------
    3 * x0 + x1 = 9
    x0 + 2 * x1 = 8
    [3, 1] = 9
    [1, 2] = 8
        
    y = mx + c
    y - m1 * x = c1
    y - m2 * x = c2
        
    [-m1, 1] = c1
    [-m2, 1] = c2
    """
    a = np.array([[-line1.m, 1],
                  [-line2.m, 1]])

    b = np.array([line1.c, line2.c])

    x, y = np.linalg.solve(a, b)
    return [x, y]


def angle_between(line1, line2):
    """
    Calculate the angle between two straight lines.

    :math: `tan( \theta ) = \left | \frac{m_{2} - m_{1}} {1 + m_{1}m_{2}} \right |`

    Parameters
    ----------
    line1, line2 : Line2D
        line objects to calculate angle between.
    
    Returns
    -------
    angle : float
        angle in radians.
    """
    m1 = line1.m
    m2 = line2.m

    angle = np.arctan( abs((m2 - m1) / (1 + m1 * m2) ) )
    return angle



def plot():
    l1.plot(label='l1', c='b')
    l2.plot(label='l2', c='b')

    l3_traj.plot(label='l3', c='r')
    
    plt.scatter(*starting_pos, label='starting pos')
    
    plt.legend()
    plt.show()


l1 = Line2D(m=1, c=0)
l2 = Line2D(m=0.25, c=0)

l1.create_data(start=0, stop=50, num=100)
l2.create_data(start=0, stop=50, num=100)

starting_pos = [3, l2.func(3)]

starting_angle = 30 #degs

l3_traj = calc_trajectory(*starting_pos, angle=starting_angle)

l3_traj.create_data(start=0, stop=50, num=100)

calc_scattering(l3_traj, 2)

plot()