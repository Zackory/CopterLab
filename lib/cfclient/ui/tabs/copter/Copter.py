import math
from time import time
from Vector import Vector

__author__ = 'CopterLab'


class Copter():
    def __init__(self):
        self.targetPosition = Vector((0, 0, 0))
        self.activePointer = False
        self.pointerPosition = Vector((0, 0, 0))
        self.pointA = Vector((0, 0, 0))
        self.pointB = Vector((0, 0, 0))
        self.trackerFrame = 0
        self.trackerPosition = Vector((0, 0, 0))
        self.velocity = Vector((0, 0, 0))
        self.trackerOrientation = (0, 0, 0, 0)
        self.trackerYPR = (0, 0, 0)
        self.activeTarget = False
        self.neutralThrust = 0.5
        self.thrustScale = 1
        self.horzScale = 1
        self.velocityScale = 1
        self.velocitySmooth = 0
        self.lastyaw = 0
        self.yawOffset = 0
        self.rigidCnt = 0
        self.singlesCnt = 0
        self.loopmode = False
        self.loopCount = 0
        self.loopPeriod = 6 * 120
        self.followmode = False
        self.controlCnt = 0
        self.lastTime = time()

    def constrain(self, value, minVal, maxVal):
        if value < minVal:
            value = minVal
        if value > maxVal:
            value = maxVal
        return value

    def GetYPR(self, q):
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

        yaw = - yaw       # to agree with copter orientation
        pitch = - pitch   # to agree with copter orientation

        return yaw, pitch, roll

    def updateRate(self):
        now = time()
        dt = now - self.lastTime
        rate = self.controlCnt / dt
        self.controlCnt = 0
        self.lastTime = now
        return rate

    def setPointerPosition(self, unknown):
        # see if something in unknown is acceptable for pointer
        self.activePointer = False
        for single in unknown:
            v = (Vector(single["pos"]) - self.trackerPosition)
            dist = v.magnitude()
            if dist > 0.1 and dist < 0.5:
                self.activePointer = True
                self.pointerPosition = Vector(single["pos"])
        return


    def updateTracking(self, frame, bodies, unknown):
        # if (position == (0.0,0.0,0.0)):
        #  print "not tracking"
        #  return

        timediff = frame - self.trackerFrame
        if timediff <= 0:
            # print "same frame number"
            return

        if len(bodies) == 0:
            print "no bodies - not tracking"
            return

        position = bodies[0]["pos"]
        orientation = bodies[0]["orient"]

        newvelocity = (Vector(position) - self.trackerPosition) * (120 / float(timediff))
        if self.velocitySmooth == 0:
            self.velocity = newvelocity
        else:
            frac = 1 / float(self.velocitySmooth + 1)
            self.velocity = (self.velocity * self.velocitySmooth + newvelocity) * frac
        self.trackerFrame = frame
        self.trackerPosition = Vector(position)
        self.trackerOrientation = orientation
        self.trackerYPR = self.GetYPR(orientation)

        self.setPointerPosition(unknown)

        self.rigidCnt = len(bodies)
        self.singlesCnt = len(unknown)

        return


    def updateTarget(self, position):
        self.targetPosition = Vector(position)
        self.activeTarget = True
        return


    def calcLoopTarget(self):
        if self.pointA == (0, 0, 0):
            return
        if self.pointB == (0, 0, 0):
            return
        self.loopCount += 1

        step = self.loopCount % self.loopPeriod
        theta = step / float(self.loopPeriod) * 2 * math.pi
        rad = math.sin(3 * theta)
        pt = self.pointA + Vector((rad * math.cos(theta), 0, rad * math.sin(theta)))

        # step = self.loopCount % self.loopPeriod
        #    if (step < 240):
        #      frac = step / 240.0
        #      pt = linterp(frac,self.pointA,self.pointB)
        #    else:
        #     frac = (step - 240) / 240.0
        #      pt = linterp(frac,self.pointB,self.pointA)

        self.updateTarget(pt)
        print "calcLoopTarget", self.loopCount, step, pt


    def calcFollowTarget(self):
        # call updateTarget with position calculated from pointer
        print "calcFollowTarget"
        if not self.activePointer:
            return
        followtarget = self.pointerPosition + Vector((0, -0.2, 0))
        # print "follow target: (%.3f,%.3f,%.3f)" % (followtarget[0], followtarget[1],followtarget[2])
        self.updateTarget(followtarget)

        return


    def CalcControlData(self, teleLog):
        if self.loopmode:
            self.calcLoopTarget()

        # ps3 provides thrust 0 -- 1, pitch -1 -- 1, roll -1 -- 1
        # print "CalcData target: (%.3f,%.3f,%.3f) tracker: (%.3f,%.3f,%.3f) velocity: (%.4f,%.4f,%.4f)" % (self.targetPosition[0], self.targetPosition[1], self.targetPosition[2], self.trackerPosition[0], self.trackerPosition[1], self.trackerPosition[2], self.velocity[0], self.velocity[1], self.velocity[2])

        if self.followmode:
            self.calcFollowTarget()

        if self.activeTarget:
            # tracker space has z axis horizontal, x axis horizontal, +y vertical to ground
            # assume copter forward in +z then +x is to left of copter, +y is vertical
            ydiff = (self.targetPosition.y - self.trackerPosition.y)
            thrust = self.neutralThrust + self.thrustScale * ydiff - self.velocityScale * self.velocity.y
            thrust = self.constrain(thrust, 0.0, 1.0)
            # print "last yaw: %.3f" % self.lastyaw
            # +z is pitch forward, -z is pitch back
            # +x is pitch left, -x is pitch right, negate computed roll to compensate
            zdiff = (self.targetPosition.z - self.trackerPosition.z)
            xdiff = (self.targetPosition.x - self.trackerPosition.x)
            zvel = self.velocity.z
            xvel = self.velocity.x
            # untransformed calculation of pitch,roll
            # pitch = self.horzScale * zdiff - self.velocityScale * self.velocity.z
            # roll = self.horzScale * xdiff - self.velocityScale * self.velocity.x
            # pitch = self.constrain(pitch,-1.0,1.0)
            # roll = - self.constrain(roll,-1.0,1.0)

            # transform z,x to copter coords to calc pitch,roll
            # positive yaw is clockwise
            correctedyaw = self.lastyaw + self.yawOffset
            # print "lastyaw %.3f corrected %.3f" % (self.lastyaw,correctedyaw)
            c = math.cos(math.radians(correctedyaw))
            s = math.sin(math.radians(correctedyaw))
            ttz = c * zdiff - s * xdiff
            ttx = s * zdiff + c * xdiff
            tvz = c * zvel - s * xvel
            tvx = s * zvel + c * xvel
            pitch = self.horzScale * ttz - self.velocityScale * tvz
            roll = self.horzScale * ttx - self.velocityScale * tvx

            # pitch = self.constrain(pitch,-1.0,1.0)
            # roll = - self.constrain(roll,-1.0,1.0)

            pitch = self.constrain(pitch, -0.8, 0.8)
            roll = - self.constrain(roll, -0.8, 0.8)
            pitch = math.asin(pitch)
            roll = math.asin(roll)

            yaw = 0

            if teleLog.logFile is not None:
                teleLog.logTelemetry(
                    "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f"
                    % (self.targetPosition.x, self.targetPosition.y, self.targetPosition.z,
                       self.trackerPosition.x, self.trackerPosition.y, self.trackerPosition.z,
                       thrust, pitch, roll,
                       ydiff, ttz, ttx,
                       self.velocity.y, tvz, tvx,
                    ))

            self.controlCnt += 1

        else:
            thrust = 0.0
            pitch = 0.0
            roll = 0.0
            yaw = 0.0
        # print "thrust", thrust, "pitch", pitch, "roll", roll, "yaw", yaw

        return {"thrust": thrust, "pitch": pitch, "roll": roll, "yaw": yaw}
