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
    x = r * np.sin(psi) * np.cos(theta)
    y = r * np.sin(psi) * np.sin(theta)
    z = r * np.cos(psi)
    return x, y, z

def cartesian_to_spherical(x,y,z):
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
    """
    if spherical:
        x1, y1, z1 = spherical_to_cartesian(*v1)
        x2, y2, z2 = spherical_to_cartesian(*v2)
    else:
        x1, y1, z1 = v1
        x2, y2, z2 = v2

    dist = np.sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
    return dist



    

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

psi = 10 * 360/2*np.pi # Opening angle in rads
number_of_anuli = 30   # Number of Annuli in Z
z_height = 10          # Total height of the cone in Z

theta_range = np.linspace(0, 2*np.pi, 100) #0 to 2pi for circles

# P is a point inside and on our cone
P = [8 * np.sin(psi) * np.cos(0),
     8 * np.sin(psi) * np.sin(0),
     8 * np.cos(psi)]


#Plot concentric rings at each annulus
for r in np.linspace(0, z_height, number_of_anuli):
    print(f'height: {r}')
    x = r * np.sin(psi) * np.cos(theta_range)
    y = r * np.sin(psi) * np.sin(theta_range)
    z = r * np.cos(psi)
    v2 = [x, y, z]
    ax.plot(x, y, z)
    
    




ax.scatter(*P, label='P')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.legend()
plt.show()