__author__ = 'zackory'

import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
ax = fig.gca(projection='3d')

# make ascending spiral in 3-space
t = np.linspace(0, 1.75*2*np.pi, 100)

x = np.sin(t)
y = np.cos(t)
z = t

# add noise
x += np.random.normal(scale=0.1, size=x.shape)
y += np.random.normal(scale=0.1, size=y.shape)
z += np.random.normal(scale=0.1, size=z.shape)

# spline parameters
s = 3.0 # smoothness parameter
k = 3 # spline order
nest = -1 # estimate of number of knots needed (-1 = maximal)

# find the knot points
tckp,u = splprep([x,y,z], s=s, k=k, nest=nest)

# evaluate spline, including interpolated points
xnew,ynew,znew = splev(np.linspace(0,1,400), tckp)

ax.plot(xnew, ynew, znew, label='Spline')
ax.scatter(x, y, z)
ax.legend()
plt.show()

