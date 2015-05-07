#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
The flight control tab shows telimitry data and flight settings.
"""

__author__ = 'CopterLab'
__all__ = ['copterTab']

import sys
import logging
import threading
from time import time

from PyQt4 import uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal

from cfclient.ui.widgets.ai import AttitudeIndicator
from cfclient.utils.config import Config
from cfclient.ui.tab import Tab
from cfclient.utils.logconfigreader import LogVariable, LogConfig

logger = logging.getLogger(__name__)
flight_tab_class = uic.loadUiType(sys.path[0] + '/../tabs/copterTab.ui')[0]

MAX_THRUST = 65365.0

from copter.Vector import Vector
from copter.TelemetryLogger import TelemetryLogger
from copter.Rotor import Rotor
from copter.PIDrotor import PIDrotor
from copter.NatNet import NatNet

teleLog = TelemetryLogger()
copter = PIDrotor()
natnet = NatNet()


"""
  DeviceWrapper class to allow wrapping of inputDevice object and reimplementation of readInput method
"""

class DeviceWrapper(object):

    def __init__(self, obj):
        self.obj = obj
        self.automode = False

    def readInput(self):
        """
          read from real controller device or calculate control values to fly to target point
          control values consist of roll, pitch, yaw, thurst, pitchcal, rollcal, estop and exit
        """
        if self.automode:
            global copter
            data = copter.calculateControl()
            return {"roll": data["roll"], "pitch": data["pitch"], "yaw": data["yaw"], "thrust": data["thrust"], "pitchcal": 0.0, "rollcal": 0.0, "estop": False, "exit": False}
        else:
            data = self.obj.readInput()
            print data
            return data

    def __getattr__(self, name):
        attr = getattr(self.__dict__['obj'], name)
        if callable(attr):
            print "getattr", name
            def my_wrapper(*args, **kwargs):
                ret = attr(*args, **kwargs)
                return ret
            return my_wrapper
        else:
            return attr

"""
  AutoFlightTab is FlightTab with a few changes
  would be better to subclass FlightTab but how to handle both wanting to load a UI
"""

TheTab = None

class CopterTab(Tab, flight_tab_class):

    _motor_data_signal = pyqtSignal(object)
    _imu_data_signal = pyqtSignal(object)

    connectionFinishedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    def __init__(self, tabWidget, helper, *args):
        super(CopterTab, self).__init__(*args)
        self.setupUi(self)

        global TheTab, natnet, copter
        TheTab = self

        self.tabName = "Quadrotor"
        self.menuName = "Quadrotor"

        self.tabWidget = tabWidget
        self.helper = helper

        self.disconnectedSignal.connect(self.disconnected)
        self.connectionFinishedSignal.connect(self.connected)
        # Incomming signals
        self.helper.cf.connectSetupFinished.add_callback(
            self.connectionFinishedSignal.emit)
        self.helper.cf.disconnected.add_callback(self.disconnectedSignal.emit)
        self.helper.inputDeviceReader.inputUpdateSignal.connect(
            self.updateInputControl)
        self.helper.inputDeviceReader.calUpdateSignal.connect(
            self.calUpdateFromInput)

        self._imu_data_signal.connect(self._imu_data_received)
        self._motor_data_signal.connect(self._motor_data_received)

        # Connect UI signals that are in this tab
        self.launchButton.clicked.connect(self.launchButtonClick)
        self.ballCatch.clicked.connect(self.ballCatchClick)
        self.rose.clicked.connect(self.roseClick)
        self.spline.clicked.connect(self.splineClick)
        self.thrustEntry.valueChanged.connect(self.thrustChanged)
        self.targetX.valueChanged.connect(self.targetXChanged)
        self.targetY.valueChanged.connect(self.targetYChanged)
        self.targetZ.valueChanged.connect(self.targetZChanged)
        self.resetTarget.clicked.connect(self.resetTargetClick)

        self.lastUIUpdate = time()
        self.isConnected = False

        # Replace joy stick object's input device with our wrapper class
        print "AutoFlightTab wrapping device"
        d = self.helper.inputDeviceReader.inputdevice
        w = DeviceWrapper(d)
        w.automode = True
        self.helper.inputDeviceReader.inputdevice = w

        self.helper.inputDeviceReader.updateMinMaxThrustSignal.emit(0, MAX_THRUST)
        self.helper.inputDeviceReader.updateThrustLoweringSlewrateSignal.emit(0, 0)
        self.helper.inputDeviceReader.updateMaxYawRateSignal.emit(200)
        self.helper.inputDeviceReader.updateMaxRPAngleSignal.emit(30)
        self.helper.inputDeviceReader.update_trim_pitch_signal.emit(0)
        self.helper.inputDeviceReader.update_trim_roll_signal.emit(0)

        # Create thread to run NatNet socket
        cmdthread = threading.Thread(target=natnet.manage_cmd_socket, args=[copter])
        cmdthread.setDaemon(True)
        cmdthread.start()

    # Received thrust must be in percentage (%) between 0% and 100%
    def alterThrust(self, thrust):
        if not self.isConnected:
            self.thrustEntry.setValue(0)
            return

        global copter
        copter.targetThrust = thrust / 100.0
        self.thrustEntry.setValue(thrust)
        print 'Thrust changed to:', thrust

    @pyqtSlot()
    def launchButtonClick(self):
        if not self.isConnected:
            return

        global copter
        if copter.targetThrust > 0:
            self.alterThrust(0)
        else:
            self.alterThrust(75)
            self.resetTargetClick()
        # self.helper.inputDeviceReader.inputdevice.automode = not self.helper.inputDeviceReader.inputdevice.automode

    @pyqtSlot()
    def ballCatchClick(self):
        global copter
        copter.catchBall()

    @pyqtSlot()
    def roseClick(self):
        global copter
        copter.rose()

    @pyqtSlot()
    def splineClick(self):
        global copter
        copter.splineTrajectory()

    @pyqtSlot()
    def resetTargetClick(self):
        global copter
        copter.resetTarget()
        copter.yawOffset = copter.trackerYPR[0] - copter.yaw
        self.targetX.setValue(copter.trackerPosition.x)
        self.targetY.setValue(copter.trackerPosition.y)
        # self.targetY.setValue(copter.targetPosition.y + 0.4)
        # self.targetYChanged()
        # self.targetZ.setValue(copter.targetPosition.z)
        self.targetZ.setValue(copter.trackerPosition.z + 0.1)
        self.targetZChanged()

    @pyqtSlot()
    def targetXChanged(self):
        global copter
        copter.setTargetX(self.targetX.value())

    @pyqtSlot()
    def targetYChanged(self):
        global copter
        copter.setTargetY(self.targetY.value())

    @pyqtSlot()
    def targetZChanged(self):
        global copter
        copter.setTargetZ(self.targetZ.value())

    @pyqtSlot()
    def thrustChanged(self):
        self.alterThrust(self.thrustEntry.value())

    def thrustToPercentage(self, thrust):
        return ((thrust / MAX_THRUST) * 100.0)

    def percentageToThrust(self, percentage):
        return int(MAX_THRUST * (percentage / 100.0))

    def _motor_data_received(self, data):
        global copter
        copter.motors = (data[k] for k in ('motor.m1', 'motor.m2', 'motor.m3', 'motor.m4'))

    def _imu_data_received(self, data):
        global copter
        copter.thrust = self.thrustToPercentage(data["stabilizer.thrust"])/100.0
        copter.roll = data["stabilizer.roll"]
        copter.pitch = data["stabilizer.pitch"]
        copter.yaw = data["stabilizer.yaw"]

        self.actualThrust.setText('%.2f%%' % self.thrustToPercentage(data["stabilizer.thrust"]))
        self.actualRoll.setText(('%.2f' % data["stabilizer.roll"]))
        self.actualPitch.setText(('%.2f' % data["stabilizer.pitch"]))
        self.actualYaw.setText(('%.2f' % data["stabilizer.yaw"]))

    """
      original
    """

    def loggingError(self):
        logger.warning("Callback of error in LogEntry :(")

    def connected(self, linkURI):
        self.isConnected = True
        lg = LogConfig("Stabalizer", 100)
        lg.addVariable(LogVariable("stabilizer.roll", "float"))
        lg.addVariable(LogVariable("stabilizer.pitch", "float"))
        lg.addVariable(LogVariable("stabilizer.yaw", "float"))
        lg.addVariable(LogVariable("stabilizer.thrust", "uint16_t"))

        self.log = self.helper.cf.log.create_log_packet(lg)
        if self.log is not None:
            self.log.dataReceived.add_callback(self._imu_data_signal.emit)
            self.log.error.add_callback(self.loggingError)
            self.log.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")

        lg = LogConfig("Motors", 100)
        lg.addVariable(LogVariable("motor.m1", "uint32_t"))
        lg.addVariable(LogVariable("motor.m2", "uint32_t"))
        lg.addVariable(LogVariable("motor.m3", "uint32_t"))
        lg.addVariable(LogVariable("motor.m4", "uint32_t"))

        self.log = self.helper.cf.log.create_log_packet(lg)
        if self.log is not None:
            self.log.dataReceived.add_callback(self._motor_data_signal.emit)
            self.log.error.add_callback(self.loggingError)
            self.log.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")

    def disconnected(self, linkURI):
        self.isConnected = False

    def calUpdateFromInput(self, rollCal, pitchCal):
        logger.debug("Trim changed on joystick: roll=%.2f, pitch=%.2f",
                     rollCal, pitchCal)

    def updateInputControl(self, roll, pitch, yaw, thrust):
        global copter
        now = time()
        if now - self.lastUIUpdate < 0.1:
            return
        self.lastUIUpdate = now

        self.targetRoll.setText(('%0.2f' % roll))
        self.targetPitch.setText(('%0.2f' % pitch))
        self.targetYaw.setText(('%0.2f' % yaw))
        self.targetThrust.setText(('%0.2f %%' % self.thrustToPercentage(thrust)))

        self.trackerX.setText('%.3f' % copter.trackerPosition.x)
        self.trackerY.setText('%.3f' % copter.trackerPosition.y)
        self.trackerZ.setText('%.3f' % copter.trackerPosition.z)

        self.orientation.setText("yaw: %.1f pitch: %.1f roll: %.1f" % copter.trackerYPR)

        self.targetX.setValue(copter.targetPosition.x)
        self.targetY.setValue(copter.targetPosition.y)
        self.targetZ.setValue(copter.targetPosition.z)

