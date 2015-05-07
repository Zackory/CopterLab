import os
import math
from time import time
from Vector import Vector
from datetime import datetime
from SplineTrajectory import SplineTrajectory

__author__ = 'CopterLab'


class PIDrotor():
    def __init__(self):
        self.motors = (0, 0, 0, 0)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.targetThrust = 0
        self.prevThrust = 0

        # Used for PID controller
        self.prevTime = time()
        self.prevError = None
        self.integral = Vector((0, 0, 0))

        # Thrust decrease (soft landing)
        self.currentThrust = 0
        self.lastTime = 0
        self.prevThrusts = [0]*25
        self.thrustCount = 0
        self.decreaseThrust = 0
        self.prevLocations = [Vector((0, 0, 0))]*25
        self.locCount = 0

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

        # Rose
        self.basePosition = Vector((0, 0, 0))
        self.isRosing = False
        self.roseCount = 0

        # Spline trajectory
        self.isSplineing = False;
        self.spline = SplineTrajectory()

        # Ball Catching
        self.ballPosition = Vector((0, 0, 0))
        self.ballVelocity = Vector((0, 0, 0))
        self.ballCatch = False

        # Logging
        self.logFile = None
        self.logCount = 0

    def calculateControl(self):
        if self.targetThrust == 0 and self.averageThrust() == 0:
            # No target thrust and the copter isn't currently moving.
            # Return all zeros
            if self.logFile is not None:
                self.logFile.close()
                self.logFile = None
            return {"thrust": 0, "roll": 0, "pitch": 0, "yaw": 0}

        # Logging
        if self.logFile is None:
            if not os.path.exists(os.path.join(os.path.dirname(__file__), '../Logs')):
                os.makedirs(os.path.join(os.path.dirname(__file__), '../Logs'))
            now = datetime.now()
            filename = '../Logs/PIDrotor_%s-%s-%s_%s-%s-%s.txt' % (now.month, now.day, now.year, now.hour, now.minute, now.second)
            absoluteName = os.path.join(os.path.dirname(__file__), filename)
            self.logFile = open(absoluteName, 'w+')

        if self.targetThrust == 0 and self.decreaseThrust == 0:
            # User has requested to land.
            # We're going to be decreasing thrust now.
            # Save the current thrust for use as a baseline for the PID controller.
            self.decreaseThrust = self.averageThrust()
            # Reset all variables
            self.ballCatch = False
            self.isRosing = False

        # Update target position based on a current action or trajectory
        if self.ballCatch:
            targetPosition = Vector((self.ballPosition.x, self.ballPosition.y, self.targetPosition.z))
        elif self.isRosing:
            targetPosition = self.calcRose()
        elif self.isSplineing:
            targetPosition = self.spline.nextPoint()
        else:
            targetPosition = self.targetPosition

        # PID Controller
        kp = 1.0
        if self.ballCatch:
            ki = 0.2
        else:
            ki = 0.05
        kd = 1.0
        error = targetPosition - self.trackerPosition
        if self.prevError is None:
            self.prevError = error
        dt = time() - self.prevTime
        self.prevTime = time()
        self.integral += error*dt
        derivative = (error - self.prevError) * (1.0/dt)
        self.prevError = error

        # pid controller calculation
        # proportional: standard error that does most of the work
        # integral: adjusts for large errors (e.g. due to a nonsymmetric(heavy on one side) copter)
        # derivative: Smooths out path (e.g. removes oscillations)
        r = error*kp + self.integral*ki + derivative*kd

        # cos = math.cos(math.radians(self.yaw))
        # sin = math.sin(math.radians(self.yaw))
        cos = math.cos(math.radians(self.trackerYPR[0]))
        sin = math.sin(math.radians(self.trackerYPR[0]))
        roll = -r.x*cos - r.y*sin
        pitch = -r.x*sin + r.y*cos
        if self.targetThrust != 0:
            thrust = self.targetThrust + r.z
        else:
            thrust = self.decreaseThrust + r.z


        # Acceleration based PID controller
        # rdes = error*kp + self.integral*ki - derivative*kd
        # g = 9.81
        # mass = 0.025
        # kf = 0.001
        # hoverThrust = 0.75
        #
        # roll = (rdes.x*sin - rdes.y*cos) * (1/g)
        # pitch = (rdes.x*cos + rdes.y*sin) * (1/g)
        # thrust = rdes.z * (mass / (8*kf*hoverThrust))
        #
        # print rdes, thrust


        # Place bounds on max values
        thrust = self.constrain(thrust, 0.0, 1.0)
        # print 'Roll: %f, Pitch: %f' % (roll, pitch)
        # roll = self.constrain(roll, -0.8, 0.8)
        # pitch = self.constrain(pitch, -0.8, 0.8)

        self.prevThrust = thrust
        self.updatePrevValues()

        # Smooth landing
        if self.targetThrust == 0 and self.averageThrust() > 0.2:
            # Decrease target z location at each step until we reach the ground
            self.targetPosition = self.targetPosition - Vector((0, 0, 0.003))
        elif self.targetThrust == 0:
            # Ground reached, reset all variables
            thrust = 0
            self.prevThrust = 0
            self.decreaseThrust = 0
            self.resetPrevValues()


        # Barrel Roll
        # Don't do this! (it doesn't work yet)

        # if self.barrelRollStage == 1:
        #     roll = 45
        #     print abs(self.trackerYPR[1])
        # if self.barrelRollStage == 1 and abs(self.trackerYPR[1]) >= 150:
        #     print 'barrel roll complete'
        #     self.barrelRollStage = 0


        # Front Flip
        # Don't do this! (its not working yet)

        # cosPos = (self.trackerYPR[1]/180.0) * math.pi/2.0
        # if self.trackerYPR[1] < -5:
        #     cosPos = math.pi/2.0 + (cosPos + math.pi/2.0)
        # if self.barrelRollStage == 1:
        #     pitch = 90 * math.cos(cosPos)
        # if self.barrelRollStage == 1 and cosPos > math.pi*9/10.0:
        #     self.barrelRollStage = 0


        # Log data
        if self.logCount % 5 == 0:
            self.logFile.write('(Output) thrust: %.3f, roll: %.3f, pitch: %.3f, yaw: %.3f \n\t(Input) targetPos: %s, trackerPos: %s, trackerVel: %s, trackerYPR: %s, copterYPR: %s\n' %
                               (thrust, roll, pitch, 0, self.roundedTuple(targetPosition), self.roundedTuple(self.trackerPosition), self.roundedTuple(self.velocity), self.roundedTuple(self.trackerYPR), self.roundedTuple((self.yaw, self.pitch, self.roll))))

        self.logCount = (self.logCount + 1) % 5

        return {"thrust": thrust, "roll": roll, "pitch": pitch, "yaw": 0}

    def roundedTuple(self, values):
        return tuple(float('{0:.3f}'.format(v)) for v in values)

    def updatePrevValues(self):
        self.prevThrusts[self.thrustCount] = self.prevThrust
        self.thrustCount = (self.thrustCount + 1) % len(self.prevThrusts)

    def resetPrevValues(self):
        self.prevThrusts = [0] * len(self.prevThrusts)
        self.thrustCount = 0

    def averageThrust(self):
        return sum(self.prevThrusts)/len(self.prevThrusts)

    def updatePrevLocations(self):
        self.prevLocations[self.locCount] = self.trackerPosition
        self.locCount = (self.locCount + 1) % len(self.prevLocations)

    def resetPrevLocations(self):
        self.prevLocations = [Vector((0, 0, 0))] * len(self.prevLocations)
        self.locCount = 0

    def averageLocations(self):
        return sum(self.prevLocations)*(1.0/len(self.prevLocations))

    def splineTrajectory(self):
        self.isSplineing = not self.isSplineing
        self.basePosition = self.trackerPosition
        self.spline.buildSpline(1000)

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

    def catchBall(self):
        self.ballCatch = not self.ballCatch
        # self.barrelRoll = True
        # self.barrelRollStage = 1

    def rose(self):
        self.isRosing = not self.isRosing
        self.basePosition = self.trackerPosition

    def calcRose(self):
        period = 720.0 * 6
        self.roseCount = (self.roseCount + 1) % period

        theta = self.roseCount / float(period) * 2 * math.pi
        rad = 0.5 * math.sin(3 * theta)
        point = self.basePosition + Vector((rad * math.cos(theta), rad * math.sin(theta), 0))

        print 'rose point:', point
        return point

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

    def correctPosition(self, position):
        # position vector is currently in (x, z, y), switch to (x, y, z)
        position = (position[0], position[2], position[1])
        return Vector(position)

    def updateTracking(self, frame, bodies, unknown):
        timediff = frame - self.trackerFrame
        if timediff <= 0:
            # print "same frame number"
            return

        if len(bodies) == 0:
            print 'no bodies - not tracking'
            return

        position = bodies[0]['pos']
        orientation = bodies[0]['orient']
        correctPosition = self.correctPosition(position)

        self.velocity = (correctPosition - self.trackerPosition) * (120 / float(timediff))

        self.trackerFrame = frame
        self.trackerPosition = correctPosition
        self.trackerOrientation = orientation
        self.trackerYPR = self.GetYPR(orientation)

        if len(bodies) >= 2:
            ballPos = self.correctPosition(bodies[1]['pos'])
            self.ballVelocity = (ballPos - self.ballPosition) * (120 / float(timediff))
            self.ballPosition = ballPos
