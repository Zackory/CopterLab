import numpy as np
from scipy import interpolate
from Vector import Vector

__author__ = 'CopterLab'


class SplineTrajectory():

    def __init__(self):
        self.currentSpline = (None, None, None)
        self.step = 0
        self.referencePoint = Vector((0, 0, 0))

    # steps = number of equally spaced points to generate around smooth spline curve
    # refPoint = frame of reference when building this spline (usually (0, 0, 0))
    def buildSpline(self, steps, refPoint = Vector((0, 0, 0))):
        self.referencePoint = refPoint
        # Generate a few points that we want to build a smooth trajectory through
        n = 1.0/5
        x = np.asarray([0, n, 2*n, 3*n, 2*n, n, 0, -n, -2*n, -3*n, -2*n, -n, 0])
        y = np.asarray([0, n, n, 0, -n, -n, 0, n, n, 0, -n, -n, 0])
        # z = np.ones(x.shape)*0.25
        z = np.asarray([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0, 0.1, 0.0])

        # Create the spline interpolant through the specified points
        tck, u = interpolate.splprep([x, y, z], s=0.0)
        self.currentSpline = interpolate.splev(np.linspace(0, 1, steps), tck)

    # Requests the next point on the spline trajectory
    def nextPoint(self):
        # print self.step, [axis.size for axis in self.currentSpline]
        point = self.referencePoint + Vector((axis[self.step]) for axis in self.currentSpline)
        self.step = (self.step + 1) % self.currentSpline[0].size
        # return self.referencePoint
        return point
