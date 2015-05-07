__author__ = 'zackory'

import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
ax = fig.gca(projection='3d')

n = 1.0/3
x = np.asarray([0, n, 2*n, 3*n, 2*n, n, 0, -n, -2*n, -3*n, -2*n, -n, 0])
y = np.asarray([0, n, n, 0, -n, -n, 0, n, n, 0, -n, -n, 0])
# z = np.ones(x.shape)
z = np.asarray([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0, 0.1, 0.0])

# tck, u = interpolate.splprep([x, y], s=0.0)
# x_i, y_i = interpolate.splev(np.linspace(0, 1, 100), tck)
# z_i = np.ones(x_i.shape)
tck, u = interpolate.splprep([x, y, z], s=0.0)
x_i, y_i, z_i = interpolate.splev(np.linspace(0, 1, 100), tck)

ax.plot(x_i, y_i, z_i, color='green', label='desired')
ax.scatter(x, y, z, color='blue', label='given')
ax.legend()
plt.show()

# print x, y, z

