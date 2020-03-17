"""
Created on Mon Mar  9 14:25:23 2020

@author: Norman Khan ~nk7g14 
Conical ULX model.

We use the following convention as commonly used in physics

psi : polar angle (angle to the z axis)
theta : azimuthial angle (angle to the x axis)

https://en.wikipedia.org/wiki/Spherical_coordinate_system
"""

import matplotlib.pyplot as plt
import numpy as np

# 3-D plotting will not work without the following input
from mpl_toolkits.mplot3d import Axes3D

#angle conversions
from auxil import d2r, r2d

def spherical_to_cartesian(r, psi, theta):
    """
    Convert a set of spherical coordinates to cartesian.

    Parameters
    ----------
    r : float
        Radial Distance.
    psi : float
        Azimuthial angle.
    theta : float
        Polar angle.

    """
    x = r * np.sin(psi) * np.cos(theta)
    y = r * np.sin(psi) * np.sin(theta)
    z = r * np.cos(psi)
    return x, y, z

def cartesian_to_spherical(x,y,z):
    """
    Convert a set of cartesian coordinates to Spherical.

    Parameters
    ----------
    x, y, z : float
        Coordinates.
    """
    r = np.sqrt(x**2 + y**2 + z**2)
    psi = np.arccos(z/r)
    theta = np.arctan(y/x)
    return r, psi, theta

def distance_between(v1, v2, spherical=False):
    """
    Calculates the straight line distance between two vectors, v1 and v2.

    cartesian:
        v1 = (x1, y1, z1)
        v2 = (x2, y2, z2)    
    spherical:
        v1 = (r1, psi1, theta1)
        v2 = (r2, psi2, theta2)
        
    |v1 - v2| = sqrt[(x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2]

    Parameters
    ----------
    v1, v2 : 1d arrays
        Vector values.

    spherical : bool
        Use input coordinates in spherical coords.
    """
    if spherical:
        x1, y1, z1 = spherical_to_cartesian(*v1)
        x2, y2, z2 = spherical_to_cartesian(*v2)
    else:
        x1, y1, z1 = v1
        x2, y2, z2 = v2

    dist = np.sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
    return dist

def create_cone_mesh(r, theta, psi):
    """
    Create cone mesh for plotting purposes
    
    Parameters
    ----------
    r : 1d array
        Radial distances over to create the cone.
    theta : 1d array
        Azimuthal angles over which to create the cone (0 - 2pi).
    psi : float
        Polar coordinate, fixed for a cone.
    """
    
    R, T = np.meshgrid(r, theta)
    
    x = R * np.sin(psi) * np.cos(T)
    y = R * np.sin(psi) * np.sin(T)
    z = R * np.cos(psi)
    return x, y, z
    
def create_line(*args):
    """
    Create line joining parsed vectors.
    
    Parameters
    ----------
    *args : 1d-array-like
        parsed vectors in the form v = [x, y, z]
    """
    
    xs = [v[0] for v in args]
    ys = [v[1] for v in args]
    zs = [v[2] for v in args]
    return xs, ys, zs

def calc_inclination(observer, deg=False):
    """
    Calculate the inclination to the x-y plane
    
    Parameters
    ----------
    observer : 1d-array
        Position of observer.
    deg : bool
        return inclination in radians or degrees.
    """
    
    x_y_plane = [1,0,0] # Arbitrary vector in the x-y plane
    
    obs_norm = np.linalg.norm(observer)
    obs_norm = np.linalg.norm(observer)
    x_y_plane_norm = np.linalg.norm(x_y_plane)
    
    dot = np.dot(observer, x_y_plane)
    
    inclination = np.arccos(dot /  (obs_norm * x_y_plane_norm))
    if deg:
        inclination = inclination * (180 / np.pi)
    return inclination

def plot():
    # Setup figure and axes
    fig = plt.figure(figsize=(10,10))
    ax_3d = fig.add_subplot(221, projection='3d')
    ax_top = fig.add_subplot(222)
    ax_side = fig.add_subplot(223)
    ax_info = fig.add_subplot(224)

    # 3D
    ax_3d.set_title('3D')
    ax_3d.set_xlim(-10,10)
    ax_3d.set_ylim(-10,10)
    
    ax_3d.plot_wireframe(x, y, z, color='black', linewidth=0.5)
    ax_3d.scatter(*P, label='P', color='r')
    ax_3d.scatter(*observer, label='observer', color='g')
    ax_3d.plot(xs=xs, ys=ys, zs=zs, c='r')
    
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    
    # Top
    ax_top.set_title('Top')
    ax_top.set_xlim(-10,10)
    ax_top.set_ylim(-10,10)
    
    ax_top.pcolor(x, y, z)
    ax_top.scatter(*P, label='P', color='r')
    ax_top.scatter(*observer, label='observer', color='g')
    ax_top.plot(xs, ys, c='r')
    
    ax_top.set_xlabel('X')
    ax_top.set_ylabel('Y')
    
    # Side
    ax_side.set_title('Side')
    ax_side.set_xlim(-10,10)
    ax_side.set_ylim(0,10)
    
    ax_side.pcolor(x, z, y)
    ax_side.scatter(P[0], P[2], label='P', color='r')
    ax_side.scatter(observer[0], observer[2], label='observer', color='g')
    ax_side.plot(xs, zs, c='r')
    ax_side.set_xlabel('X')
    ax_side.set_ylabel('Z')

    #Information penel
    plot_info(ax_info, None)

    plt.show()



def plot_info(ax_info, info):
    """
    Plot information of simulation onto specified window
    """
    ax_info.set_title('Info')
    ax_info.set_xticks([])
    ax_info.set_yticks([])


    info = {'inclination' : inclination,
        'H' : H,
        'R' : R,
        r'$\theta \ / \ 2$' : opening_angle,
        r'$\mid r \mid$' : r_mag,
        'number_of_anuli' : sampling_level,
        'observer' : observer,
        'P' : P}

    xpos = 0.05
    ypos = 0.95

    for k, v in info.items():
        ax_info.text(xpos, ypos, f'{k} : {v}', horizontalalignment='left',
                 verticalalignment='center', transform=ax_info.transAxes, fontsize=8,
                 fontname='monospace')
        ypos-=0.05





if __name__ == "__main__":
    opening_angle = 40
    
    H = 5
    R = 20
    H_R = H / R
    
    observer = (-9, 0, 9)
    inclination = calc_inclination(observer, deg=True)
    
    psi = opening_angle * np.pi/180 # Opening angle in rads

    r_mag = 10                      # radial distance to calculate to
    
    sampling_level = 100 #Number of samples to genereate for r and theta (number of annuli)


    r = np.linspace(0, r_mag, sampling_level)
    theta = np.linspace(0, 2*np.pi, sampling_level)

    x, y, z = create_cone_mesh(r, theta, psi)
    
    
    # P is a point on our cone
    P = (8 * np.sin(psi) * np.cos(0),
         8 * np.sin(psi) * np.sin(0),
         8 * np.cos(psi))
    
    xs, ys, zs = create_line(P, observer)




    plot()
    