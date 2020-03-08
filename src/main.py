"""
Conical ULX model.


We use the following convention
psi : polar angle
theta : azimuthial angle
https://en.wikipedia.org/wiki/Spherical_coordinate_system

"""
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
import numpy as np


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



    
class shape:
    def __init__(self):
        self.coords = None


3d_line = np.array([[0,1,0],
                    [2,2,2],
                    [4,5,6]])

class cone:



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


opening_angle = 10

psi = opening_angle * np.pi/180 # Opening angle in rads
number_of_anuli = 30   # Number of Annuli in Z
z_height = 10          # Total height of the cone in Z

theta_range = np.linspace(0, 2*np.pi, 100) #0 to 2pi for circles

# P is a point inside and on our cone
P = [8 * np.sin(psi) * np.cos(0),
     8 * np.sin(psi) * np.sin(0),
     8 * np.cos(psi)]


#Plot concentric rings at each annulus
# for r in np.linspace(0, z_height, number_of_anuli):
r =  np.linspace(0, z_height, number_of_anuli)

# print(f'height: {r}')
x = r * np.sin(psi) * np.cos(theta_range)
y = r * np.sin(psi) * np.sin(theta_range)
z = r * np.cos(psi)
v2 = [x, y, z]
ax.plot_wireframe(x, y, z)
    
    




ax.scatter(*P, label='P')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.legend()
plt.show()
