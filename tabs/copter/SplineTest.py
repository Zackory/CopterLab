__author__ = 'zackory'


import numpy as np
import matplotlib.pyplot as plt
import math

error = 0.1
x0 = 1
y0 = 1
r0 = 0.5

alpha = np.linspace(0, 2*math.pi, 40, endpoint=False)
r = r0 + error * np.random.random(len(alpha))
x = x0 + r * np.cos(alpha)
y = x0 + r * np.sin(alpha)
plt.scatter(x, y, color='blue', label='given')

from scipy import interpolate
tck,u=interpolate.splprep([x,y],s=0.0)
x_i, y_i = interpolate.splev(np.linspace(0,1,100),tck)
plt.plot(x_i, y_i, color='green', label='desired')

plt.legend()
plt.show()

