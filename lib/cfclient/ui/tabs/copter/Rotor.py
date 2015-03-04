import math
from time import time
from Vector import Vector

__author__ = 'CopterLab'


class Rotor():
    def __init__(self):
        self.motors = (0, 0, 0, 0)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.targetRoll = 0
        self.targetPitch = 0
        self.targetYaw = 0
        self.targetThrust = 0
        self.trackerFrame = 0
        self.trackerPosition = Vector((0, 0, 0))
        self.trackerOrientation = (0, 0, 0, 0)
        self.velocity = Vector((0, 0, 0))
        self.targetPosition = Vector((0, 0, 0))
        self.currentThrust = 0
        self.lastTime = 0

    def calculateControl(self):
        slowDownExp = 1.0
        if self.lastTime != 0:
            # 5 second ramp-down based on a 1 - x^3 function
            timeDiff = (time() - self.lastTime) / 5.0
            slowDownExp = 1.0 - timeDiff ** 3
        if self.targetThrust > self.currentThrust:
            # Thrust can increase safely
            self.currentThrust = self.targetThrust
            self.lastTime = 0
            slowDownExp = 1.0
        elif self.targetThrust < self.currentThrust and self.lastTime == 0:
            # Thrust decreasing, begin ramp-down
            self.lastTime = time()
        elif self.currentThrust * slowDownExp <= 0:
            # We have finished ramping-down thrust, reset everything to 0
            self.currentThrust = 0
            self.lastTime = 0
        calcThrust = self.currentThrust * slowDownExp
        calcRoll = -self.roll / 30.0 / 2
        calcPitch = -self.pitch / 30.0 / 2 - 8.0 / 30.0
        calcYaw = 0

        return {"thrust": calcThrust, "roll": calcRoll, "pitch": calcPitch, "yaw": calcYaw}

    def setTargetX(self, x):
        self.targetPosition = Vector((x, self.targetPosition.y, self.targetPosition.z))

    def setTargetY(self, y):
        self.targetPosition = Vector((self.targetPosition.x, y, self.targetPosition.z))

    def setTargetZ(self, z):
        self.targetPosition = Vector((self.targetPosition.x, self.targetPosition.y, z))

    def resetTarget(self):
        self.targetPosition = self.trackerPosition

    def constrain(self, value, minVal, maxVal):
        if value < minVal:
            value = minVal
        if value > maxVal:
            value = maxVal
        return value

    def updateTracking(self, frame, bodies, unknown):
        timediff = frame - self.trackerFrame
        if timediff <= 0:
            # print "same frame number"
            return

        if len(bodies) == 0:
            print "no bodies - not tracking"
            return

        position = bodies[0]["pos"]
        orientation = bodies[0]["orient"]

        self.trackerFrame = frame
        self.trackerPosition = Vector(position)
        self.trackerOrientation = orientation

        self.velocity = (Vector(position) - self.trackerPosition) * (120 / float(timediff))
