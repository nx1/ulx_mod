# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
import numpy as np



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
    x = r * np.sin(psi) * np.cos(theta_range)
    y = r * np.sin(psi) * np.sin(theta_range)
    z = r * np.cos(psi)
    ax.plot(x, y, z)
    


ax.scatter(*P, label='P')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.legend()
plt.show()