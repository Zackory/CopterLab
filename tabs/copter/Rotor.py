import os
import math
from time import time
from Vector import Vector
from datetime import datetime

__author__ = 'CopterLab'


class Rotor():
    def __init__(self):
        self.motors = (0, 0, 0, 0)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.targetThrust = 0
        self.prevThrust = 0

        # Thrust decrease (soft landing)
        self.currentThrust = 0
        self.lastTime = 0
        self.prevThrusts = [0]*25
        self.prevLoc = 0

        # Tracker variables
        self.trackerFrame = 0
        self.trackerPosition = Vector((0, 0, 0))
        self.trackerOrientation = (0, 0, 0, 0)
        self.velocity = Vector((0, 0, 0))
        self.targetPosition = Vector((0, 0, 0))
        self.trackerYPR = (0, 0, 0)
        self.yawOffset = 0

        self.barrelRoll = False
        self.barrelRollStage = 0
        self.lastRoll = 0

        # Logging
        self.logFile = None
        self.logCount = 0

    def calculateControl(self):
        if self.targetThrust == 0 and self.prevThrust == 0:
            if self.logFile is not None:
                self.logFile.close()
                self.logFile = None
            return {"thrust": 0, "roll": 0, "pitch": 0, "yaw": 0}

        # Logging
        if self.logFile is None:
            if not os.path.exists(os.path.join(os.path.dirname(__file__), '../Logs')):
                os.makedirs(os.path.join(os.path.dirname(__file__), '../Logs'))
            now = datetime.now()
            filename = '../Logs/Rotor_%s-%s-%s_%s-%s-%s.txt' % (now.month, now.day, now.year, now.hour, now.minute, now.second)
            absoluteName = os.path.join(os.path.dirname(__file__), filename)
            self.logFile = open(absoluteName, 'w+')

        # Soft landing
        slowDownExp = 1.0
        if self.lastTime != 0:
            # If copter is falling to fast, increase thrust by decreasing timeDiff
            # if self.velocity.y < -0.5:
            #     self.lastTime += 0.25;

            # 5 second ramp-down based on a 1 - x^3 function
            timeDiff = (time() - self.lastTime) / 5.0
            slowDownExp = 1.0 - timeDiff ** 4

        if self.lastTime == 0 and self.targetThrust != 0:
            # Thrust can increase safely
            self.updatePrevValues()
        elif self.targetThrust == 0 and self.averageThrust() > 0 and self.lastTime == 0:
            # Thrust decreasing, begin ramp-down
            self.lastTime = time()
        elif self.averageThrust() * slowDownExp - 1.0 * self.velocity.y <= 0.4:
            # We have finished ramping-down thrust, reset everything to 0
            self.resetPrevValues()
            self.lastTime = 0

        print 'Velocity y: ', self.velocity.y

        thrustScale = 1.0
        velocityScale = 1.0
        horzScale = 1.0

        # tracker space has z axis horizontal, x axis horizontal, +y vertical to ground
        # assume copter forward in +z then +x is to left of copter, +y is vertical
        ydiff = (self.targetPosition.y - self.trackerPosition.y)
        if self.targetThrust == 0:
            calcThrust = self.averageThrust() * slowDownExp - velocityScale * self.velocity.y
        else:
            calcThrust = self.targetThrust + thrustScale * ydiff - velocityScale * self.velocity.y
            # self.currentThrust = calcThrust
        calcThrust = self.constrain(calcThrust, 0.0, 1.0)
        # print "last yaw: %.3f" % self.lastyaw
        # +z is pitch forward, -z is pitch back
        # +x is pitch left, -x is pitch right, negate computed roll to compensate
        zdiff = (self.targetPosition.z - self.trackerPosition.z)
        xdiff = (self.targetPosition.x - self.trackerPosition.x)
        zvel = self.velocity.z
        xvel = self.velocity.x

        # transform z,x to copter coords to calc pitch,roll
        # positive yaw is clockwise
        # correctedyaw = self.yaw
        correctedyaw = self.trackerYPR[0]
        # print "lastyaw %.3f corrected %.3f" % (self.lastyaw,correctedyaw)
        c = math.cos(math.radians(correctedyaw))
        s = math.sin(math.radians(correctedyaw))
        ttz = c * zdiff - s * xdiff
        ttx = s * zdiff + c * xdiff
        tvz = c * zvel - s * xvel
        tvx = s * zvel + c * xvel
        calcPitch = horzScale * ttz - velocityScale * tvz
        calcRoll = horzScale * ttx - velocityScale * tvx

        calcPitch = self.constrain(calcPitch, -0.8, 0.8)
        calcRoll = - self.constrain(calcRoll, -0.8, 0.8)
        calcPitch = math.asin(calcPitch)
        calcRoll = math.asin(calcRoll)

        calcYaw = 0

        # Barrel Roll Stages
        if self.barrelRollStage != 0:
            calcPitch = 0
            # print self.roll

        if 0 < self.barrelRollStage < 10:
            calcRoll = -90
            self.barrelRollStage += 1
        else:
            self.barrelRollStage = 0

        self.prevThrust = calcThrust

        # Log data
        if self.logCount % 5 == 0:
            self.logFile.write('(Output) thrust: %.3f, roll: %.3f, pitch: %.3f, yaw: %.3f \n\t(Input) targetPos: %s, trackerPos: %s, trackerVel: %s, trackerYPR: %s, copterYPR: %s\n' %
                               (calcThrust, calcRoll, calcPitch, 0, self.roundedTuple(self.targetPosition), self.roundedTuple(self.trackerPosition), self.roundedTuple(self.velocity), self.roundedTuple(self.trackerYPR), self.roundedTuple((self.yaw, self.pitch, self.roll))))

        self.logCount = (self.logCount + 1) % 5

        # if self.barrelRollStage == 1 and self.roll > -60:
        #     calcRoll = -10
        # elif self.barrelRollStage == 1:
        #     print 'stage 2', self.roll
        #     self.barrelRollStage = 2
        #
        # if self.barrelRollStage == 2 and self.roll < -15:
        #     calcRoll = 0
        # elif self.barrelRollStage == 2:
        #     print 'stage 3', self.roll
        #     self.barrelRollStage = 3
        #
        # if self.barrelRollStage == 3 and self.roll < 60:
        #     calcRoll = 5
        # elif self.barrelRollStage == 3:
        #     print 'stage 4', self.roll
        #     self.barrelRollStage = 4
        #
        # if self.barrelRollStage == 4 and self.roll > 15 and self.lastRoll - self.roll > 0:
        #     calcRoll = -1
        # elif self.barrelRollStage == 4:
        #     print 'barrel roll ended', self.roll
        #     self.barrelRollStage = 0
        #
        # self.lastRoll = self.roll

        return {"thrust": calcThrust, "roll": calcRoll, "pitch": calcPitch, "yaw": calcYaw}

    def roundedTuple(self, values):
        return tuple(float('{0:.3f}'.format(v)) for v in values)

    def updatePrevValues(self):
        self.prevThrusts[self.prevLoc] = self.targetThrust
        self.prevLoc = (self.prevLoc + 1) % len(self.prevThrusts)

    def resetPrevValues(self):
        self.prevThrusts = [0] * len(self.prevThrusts)
        self.prevLoc = 0

    def averageThrust(self):
        return sum(self.prevThrusts)/len(self.prevThrusts)

    def GetYPR(self, q):
        # q[0] = x, q[1] = y, q[2] = z, q[3] = w
        # q = (q[0], -q[2], q[1], q[3])
        t = q[0] * q[1] + q[2] * q[3]
        if t > 0.499:
            yaw = 2 * math.atan2(q[0], q[3])
            pitch = math.pi / 2.0
            roll = 0.0
        elif t < -0.499:
            yaw = -2 * math.atan2(q[0], q[3])
            pitch = -math.pi / 2.0
            roll = 0.0
        else:
            sqx = q[0]*q[0]
            sqy = q[1]*q[1]
            sqz = q[2]*q[2]
            yaw = math.atan2(2 * q[1]*q[3] - 2 * q[0] * q[2], 1 - 2 * sqy - 2 * sqz)
            roll = math.asin(2*t)
            pitch = math.atan2(2 * q[0] * q[3] - 2 * q[1] * q[2], 1 - 2 * sqx - 2 * sqz)

        yaw = math.degrees(yaw)
        pitch = math.degrees(pitch)
        roll = math.degrees(roll)

        # roll  = Mathf.Atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
        # pitch = Mathf.Atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
        # yaw   =  Mathf.Asin(2*x*y + 2*z*w);

        yaw = - yaw       # to agree with copter orientation
        pitch = - pitch   # to agree with copter orientation

        return yaw, pitch, roll

    def performBarrelRoll(self):
        self.barrelRoll = True
        self.barrelRollStage = 1

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

        self.velocity = (Vector(position) - self.trackerPosition) * (120 / float(timediff))

        self.trackerFrame = frame
        self.trackerPosition = Vector(position)
        self.trackerOrientation = orientation
        self.trackerYPR = self.GetYPR(orientation)
