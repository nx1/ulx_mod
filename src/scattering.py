# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

from auxil import d2r, r2d #angle conversions


class Line2D:
    def __init__(self, m, c, angle):
        self.m = m
        self.c = c
        self.angle = angle
        self.func = lambda x: self.m * x + self.c
        self.direction = None

    def __repr__(self):
        return repr(f'Line2D : y = {self.m}x + {self.c} | angle = {r2d(self.angle)} | direction = {self.direction}')

    @classmethod
    def from_function(cls, m, c):
        """Create line object from gradient and y-intersection."""
        angle = np.arctan(m)
        return cls(m, c, angle)

    @classmethod
    def from_points(cls, pos1, pos2):
        """
        Create line object from two datapoints.
        
        Parameters
        ----------
        pos1, pos2 : [x, y]
        """
        x1, y1 = pos1
        x2, y2 = pos2
        print(f'pos1: {pos1}, pos2: {pos2}')

        m = abs(y2-y1) / abs(x2-x1)
        c = y1 - m*x1
        angle = np.arctan(m)

        print(f'm: {m}, c: {c} angle: {angle}')
        return cls(m, c, angle)

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
        x, y = pos
        m = np.tan(angle)
        c = y - m*x
        return cls(m, c, angle)

    def set_x_direction(self, direction):
        """
        Set the direction of the line.
        Call with no args for reset.
        """
        directions = {'None' : None,
                      'x+' : +1,
                      'x-' : -1}
        self.direction = directions[direction]

    def create_data(self, start, stop, num):
        """Create datapoints between a specified range."""
        self.xdata = np.linspace(start, stop, num)
        self.ydata = self.func(self.xdata)

    def plot(self, **kwargs):
        plt.plot(self.xdata, self.ydata, **kwargs)

    def reflect(self, Line2D):
        """
        Calculate new path after reflection off a line.

        Parameters
        ----------
        Line2D : Line2D
            Boundry to reflect off.

        Returns
        -------
        direction

        """
        if self.direction == None:
            print(f'{Line2D} has no direction!')


        return Line2D


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
    trajectory = Line2D.from_function(m=m, c=c)
    return trajectory


# scattering_positions = [[x,y]], [x1,y1] + ...
# scattering_lines = [Line2D, Line2D] + ...

def calc_scattering(Line2D, N):
    for i in range(N):
        scattering_positions = calc_scattering_positions(Line2D, N)
        scattering_paths = calc_scattering_path(Line2D, N)


def calc_scattering_positions(Line2D, N, keep=True):
    """
    Parameters
    ----------
    Line2D : Line2D
        Starting path
    N : int
        Nth scatter
    keep : bool
        Return all scatters or just last.

    Returns
    -------
    scattering_position : list
        [x,y]
    """
    traj = Line2D

    l1

    traj = l3_traj
    traj.set_x_direction('x+')
    t.plot()

    l1_int = find_intersection(traj, l1)
    l2_int = find_intersection(traj, l2)

    collisions = determine_collision(l1_int, l2_int)

    calc_new_path = Line2D.reflect(l1_int)


    plt.scatter(*scattering_positions[0])
    plt.scatter(*scattering_positions[1])

    l1.plot()
    l2.plot()
    l3_traj.plot()

    if keep:
        return scattering_positions
    else:
        return scattering_positions[-1]
# =============================================================================
#
# =============================================================================
    for n in range(N):
        print(f'Scattering #{n}')

        l1_int = find_intersection(trajectory, l1)
        l2_int = find_intersection(trajectory, l2)

        print('l1 intersection:', l1_int)
        print('l2 intersection:', l2_int)

        # plt.scatter(*l1_int, label='l1_intersection')
        # plt.scatter(*l2_int, label='l2_intersection')

        angle = angle_between(trajectory, l2)
        print('old trajectory angle:', trajectory.angle)

        print('angle between tracjecory and l2:', angle)

        angle = calc_new_angle(trajectory.angle, 2*angle)

        print('new_angle:', angle)

        trajectory = Line2D.from_pos_angle(l1_int, angle)

        scattering_positions.append(l1_int)
        trajectory.create_data(start=l1_int[0], stop=50, num=100)
        trajectory.plot(label='traj')
        print(f'Trajectory {trajectory}')
        print('=======')

    return scattering_position





# =============================================================================
#
# =============================================================================
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


l1 = Line2D.from_function(m=1, c=0)
l2 = Line2D.from_function(m=0.25, c=0)

l1.create_data(start=0, stop=50, num=100)
l2.create_data(start=0, stop=50, num=100)

starting_pos = [3, l2.func(3)]

starting_angle = 30 #degs

l3_traj = Line2D.from_pos_angle(starting_pos, starting_angle)


l3_traj.create_data(start=0, stop=50, num=100)

calc_scattering(l3_traj, 2)


def calc_scattering(Line2D, N):

    return Line2D


plot()