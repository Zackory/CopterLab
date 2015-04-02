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
__all__ = ['AutoFlightTab']

import sys
import threading
import math

import logging
logger = logging.getLogger(__name__)

from time import time

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal, QThread, SIGNAL

from cflib.crazyflie import Crazyflie

from cfclient.ui.widgets.ai import AttitudeIndicator

from cfclient.utils.config import Config
from cflib.crazyflie.log import Log

from cfclient.ui.tab import Tab

from cfclient.utils.logconfigreader import LogVariable, LogConfig

flight_tab_class = uic.loadUiType(sys.path[0] + '/../tabs/autoflightTab.ui')[0]

MAX_THRUST = 65365.0

from copter.Vector import Vector
from copter.TelemetryLogger import TelemetryLogger
from copter.Copter import Copter
from copter.NatNet import NatNet

teleLog = TelemetryLogger()
copter = Copter()
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
        if (self.automode):
            #      print "automode"
            global copter, teleLog
            data = copter.CalcControlData(teleLog)
            return {"roll": data["roll"], "pitch": data["pitch"], "yaw": data["yaw"], "thrust": data["thrust"], "pitchcal": 0.0, "rollcal": 0.0, "estop": False, "exit": False}
        else:
            #print "ps3 mode"
            data = self.obj.readInput()
            #      print "ps3 -- thrust", data["thrust"], "pitch", data["pitch"], "roll", data["roll"]
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

class AutoFlightTab(Tab, flight_tab_class):

    uiSetupReadySignal = pyqtSignal()

    _motor_data_signal = pyqtSignal(object)
    _imu_data_signal = pyqtSignal(object)

    UI_DATA_UPDATE_FPS = 10

    connectionFinishedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    def __init__(self, tabWidget, helper, *args):
        super(AutoFlightTab, self).__init__(*args)
        self.setupUi(self)

        global TheTab, natnet, copter
        TheTab = self

        self.tabName = "Auto Flight Control"
        self.menuName = "Auto Flight Control"

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
        self.helper.inputDeviceReader.emergencyStopSignal.connect(
            self.updateEmergencyStop)

        self._imu_data_signal.connect(self._imu_data_received)
        self._motor_data_signal.connect(self._motor_data_received)

        # Connect UI signals that are in this tab
        self.flightModeCombo.currentIndexChanged.connect(self.flightmodeChange)
        self.minThrust.valueChanged.connect(self.minMaxThrustChanged)
        self.maxThrust.valueChanged.connect(self.minMaxThrustChanged)
        self.thrustLoweringSlewRateLimit.valueChanged.connect(
            self.thrustLoweringSlewRateLimitChanged)
        self.slewEnableLimit.valueChanged.connect(
            self.thrustLoweringSlewRateLimitChanged)
        self.targetCalRoll.valueChanged.connect(self._trim_roll_changed)
        self.targetCalPitch.valueChanged.connect(self._trim_pitch_changed)
        self.maxAngle.valueChanged.connect(self.maxAngleChanged)
        self.maxYawRate.valueChanged.connect(self.maxYawRateChanged)
        self.uiSetupReadySignal.connect(self.uiSetupReady)
        self.clientXModeCheckbox.toggled.connect(self.changeXmode)
        self.isInCrazyFlightmode = False
        self.uiSetupReady()

        self.clientXModeCheckbox.setChecked(Config().get("client_side_xmode"))

        self.crazyflieXModeCheckbox.clicked.connect(
            lambda enabled:
            self.helper.cf.param.set_value("flightctrl.xmode",
                                           str(enabled)))
        self.helper.cf.param.add_update_callback(
            "flightctrl.xmode",
            lambda name, checked:
            self.crazyflieXModeCheckbox.setChecked(eval(checked)))
        self.ratePidRadioButton.clicked.connect(
            lambda enabled:
            self.helper.cf.param.set_value("flightctrl.ratepid",
                                           str(enabled)))
        self.angularPidRadioButton.clicked.connect(
            lambda enabled:
            self.helper.cf.param.set_value("flightctrl.ratepid",
                                           str(not enabled)))
        self.helper.cf.param.add_update_callback("flightctrl.ratepid",
                                                 lambda name, checked:
                                                 self.ratePidRadioButton.setChecked(eval(checked)))

        self.ai = AttitudeIndicator()
        self.gridLayout.addWidget(self.ai, 0, 1)

        self.targetCalPitch.setValue(Config().get("trim_pitch"))
        self.targetCalRoll.setValue(Config().get("trim_roll"))

        """
          ADDITION: automode ui
        """
        self.automodeCheckBox.toggled.connect(self.changeAutoMode)
        self.loopmodeCheckBox.toggled.connect(self.changeLoopMode)
        self.followmodeCheckBox.toggled.connect(self.changeFollowMode)
        self.launchButton.clicked.connect(self.launchButtonClicked)
        self.upButton.clicked.connect(self.upButtonClicked)
        self.downButton.clicked.connect(self.downButtonClicked)
        self.frontButton.clicked.connect(self.frontButtonClicked)
        self.backButton.clicked.connect(self.backButtonClicked)
        self.rightButton.clicked.connect(self.rightButtonClicked)
        self.leftButton.clicked.connect(self.leftButtonClicked)
        self.setAButton.clicked.connect(self.setAButtonClicked)
        self.setBButton.clicked.connect(self.setBButtonClicked)
        self.goAButton.clicked.connect(self.goAButtonClicked)
        self.goBButton.clicked.connect(self.goBButtonClicked)
        self.neutralThrustBox.valueChanged.connect(self.neutralThrustChanged)
        self.thrustScaleBox.valueChanged.connect(self.thrustScaleChanged)
        self.horzScaleBox.valueChanged.connect(self.horzScaleChanged)
        self.velocityScaleBox.valueChanged.connect(self.velocityScaleChanged)
        self.velocitySmoothBox.valueChanged.connect(self.velocitySmoothChanged)

        self.lastUIUpdate = time()

        """
          ADDITION: replace joy stick object's input device with our wrapper class
        """
        print "AutoFlightTab wrapping device"
        d = self.helper.inputDeviceReader.inputdevice
        w = DeviceWrapper(d)
        w.automode = False
        self.helper.inputDeviceReader.inputdevice = w

        """
          ADDITION: initialize tracker fields of interface
        """
        self.position.setText("received tracker position")
        self.orientation.setText("received tracker orientation")

        """
          ADDITION: create thread to run command socket
        """
        cmdthread = threading.Thread(target=natnet.manage_cmd_socket, args=[copter])
        # cmdthread = threading.Thread(None, manage_cmd_socket, None)
        cmdthread.setDaemon(True)
        cmdthread.start()

    """
      ADDITION: automode ui methods
    """
    @pyqtSlot(bool)
    def changeAutoMode(self, checked):
        print "changeAutoMode", checked
        # self.helper.inputDeviceReader.inputdevice.automode = checked

    @pyqtSlot(bool)
    def changeLoopMode(self, checked):
        print "changeLoopMode", checked
        global copter
        copter.loopmode = checked

    @pyqtSlot(bool)
    def changeFollowMode(self, checked):
        print "changeFollowMode", checked
        global copter
        copter.followmode = checked

    @pyqtSlot()
    def launchButtonClicked(self):
        global copter
        print "launchButtonClicked tracker:", copter.trackerPosition
        copter.yawOffset = copter.trackerYPR[0] - copter.lastyaw
        copter.updateTarget(copter.trackerPosition + Vector((0,0.2,0)))
        print "yawOffset:", copter.yawOffset
        print "target:", copter.targetPosition

        self.helper.inputDeviceReader.inputdevice.automode = not self.helper.inputDeviceReader.inputdevice.automode


    @pyqtSlot()
    def upButtonClicked(self):
        global copter
        print "upButtonClicked tracker:", copter.trackerPosition
        copter.updateTarget(copter.targetPosition + Vector((0,0.1,0)))
        print "target:", copter.targetPosition

    @pyqtSlot()
    def downButtonClicked(self):
        global copter
        print "downButtonClicked tracker:", copter.trackerPosition
        copter.updateTarget(copter.targetPosition + Vector((0,-0.1,0)))
        print "target:", copter.targetPosition

    @pyqtSlot()
    def frontButtonClicked(self):
        global copter
        print "frontButtonClicked tracker:", copter.trackerPosition
        copter.updateTarget(copter.targetPosition + Vector((0,0,0.1)))
        print "target:", copter.targetPosition

    @pyqtSlot()
    def backButtonClicked(self):
        global copter
        print "backButtonClicked tracker:", copter.trackerPosition
        copter.updateTarget(copter.targetPosition + Vector((0,0,-0.1)))
        print "target:", copter.targetPosition

    @pyqtSlot()
    def rightButtonClicked(self):
        global copter
        print "rightButtonClicked tracker:", copter.trackerPosition
        copter.updateTarget(copter.targetPosition + Vector((-0.1,0,0)))
        print "target:", copter.targetPosition

    @pyqtSlot()
    def leftButtonClicked(self):
        global copter
        print "leftButtonClicked tracker:", copter.trackerPosition
        copter.updateTarget(copter.targetPosition + Vector((0.1,0,0)))
        print "target:", copter.targetPosition

    @pyqtSlot()
    def setAButtonClicked(self):
        global copter
        print "setAButtonClicked tracker:", copter.targetPosition
        copter.pointA = copter.targetPosition
        print "pointA:", copter.pointA

    @pyqtSlot()
    def setBButtonClicked(self):
        global copter
        print "setBButtonClicked tracker:", copter.targetPosition
        copter.pointB = copter.targetPosition
        print "pointB:", copter.pointB

    @pyqtSlot()
    def goAButtonClicked(self):
        global copter
        print "goAButtonClicked"
        global teleLog
        teleLog.startLog()
        copter.updateTarget(copter.pointA)

    @pyqtSlot()
    def goBButtonClicked(self):
        global copter
        print "goBButtonClicked"
        global teleLog
        teleLog.startLog()
        copter.updateTarget(copter.pointB)

    @pyqtSlot()
    def neutralThrustChanged(self):
        print "neutralThrustChanged", self.neutralThrustBox.value()
        global copter
        copter.neutralThrust = self.neutralThrustBox.value()

    @pyqtSlot()
    def thrustScaleChanged(self):
        print "thrustScaleChanged", self.thrustScaleBox.value()
        global copter
        copter.thrustScale = self.thrustScaleBox.value()

    @pyqtSlot()
    def horzScaleChanged(self):
        print "horzScaleChanged", self.horzScaleBox.value()
        global copter
        copter.horzScale = self.horzScaleBox.value()

    @pyqtSlot()
    def velocityScaleChanged(self):
        print "velocityScaleChanged", self.velocityScaleBox.value()
        global copter
        copter.velocityScale = self.velocityScaleBox.value()

    @pyqtSlot()
    def velocitySmoothChanged(self):
        print "velocitySmoothChanged", self.velocitySmoothBox.value()
        global copter
        copter.velocitySmooth = self.velocitySmoothBox.value()

    """
      original
    """

    def thrustToPercentage(self, thrust):
        return ((thrust / MAX_THRUST) * 100.0)

    def percentageToThrust(self, percentage):
        return int(MAX_THRUST * (percentage / 100.0))

    def uiSetupReady(self):
        flightComboIndex = self.flightModeCombo.findText(
            Config().get("flightmode"), Qt.MatchFixedString)
        if (flightComboIndex < 0):
            self.flightModeCombo.setCurrentIndex(0)
            self.flightModeCombo.currentIndexChanged.emit(0)
        else:
            self.flightModeCombo.setCurrentIndex(flightComboIndex)
            self.flightModeCombo.currentIndexChanged.emit(flightComboIndex)

    def loggingError(self):
        logger.warning("Callback of error in LogEntry :(")

    def _motor_data_received(self, data):
        self.actualM1.setValue(data["motor.m1"])
        self.actualM2.setValue(data["motor.m2"])
        self.actualM3.setValue(data["motor.m3"])
        self.actualM4.setValue(data["motor.m4"])

    def _imu_data_received(self, data):
        self.actualRoll.setText(("%.2f" % data["stabilizer.roll"]))
        self.actualPitch.setText(("%.2f" % data["stabilizer.pitch"]))
        self.actualYaw.setText(("%.2f" % data["stabilizer.yaw"]))
        self.actualThrust.setText("%.2f%%" %
                                  self.thrustToPercentage(
                                      data["stabilizer.thrust"]))

        # self.ai.setRollPitch(-data["stabilizer.roll"],
        #                      data["stabilizer.pitch"])

        """
        ADDITION
        save reported yaw for coord conversion - ? is there some other way to get this ?
        """
        global copter
        copter.lastyaw = data["stabilizer.yaw"]

    def connected(self, linkURI):
        lg = LogConfig("Stabalizer", 100)
        lg.addVariable(LogVariable("stabilizer.roll", "float"))
        lg.addVariable(LogVariable("stabilizer.pitch", "float"))
        lg.addVariable(LogVariable("stabilizer.yaw", "float"))
        lg.addVariable(LogVariable("stabilizer.thrust", "uint16_t"))

        self.log = self.helper.cf.log.create_log_packet(lg)
        if (self.log is not None):
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
        if (self.log is not None):
            self.log.dataReceived.add_callback(self._motor_data_signal.emit)
            self.log.error.add_callback(self.loggingError)
            self.log.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")

    def disconnected(self, linkURI):
        self.ai.setRollPitch(0, 0)
        self.actualM1.setValue(0)
        self.actualM2.setValue(0)
        self.actualM3.setValue(0)
        self.actualM4.setValue(0)
        self.actualRoll.setText("")
        self.actualPitch.setText("")
        self.actualYaw.setText("")
        self.actualThrust.setText("")

    def minMaxThrustChanged(self):
        self.helper.inputDeviceReader.updateMinMaxThrustSignal.emit(
            self.percentageToThrust(self.minThrust.value()),
            self.percentageToThrust(self.maxThrust.value()))
        if (self.isInCrazyFlightmode == True):
            Config().set("min_thrust", self.minThrust.value())
            Config().set("max_thrust", self.maxThrust.value())

    def thrustLoweringSlewRateLimitChanged(self):
        self.helper.inputDeviceReader.updateThrustLoweringSlewrateSignal.emit(
            self.percentageToThrust(self.thrustLoweringSlewRateLimit.value()),
            self.percentageToThrust(
                self.slewEnableLimit.value()))
        if (self.isInCrazyFlightmode == True):
            Config().set("slew_limit", self.slewEnableLimit.value())
            Config().set("slew_rate", self.thrustLoweringSlewRateLimit.value())

    def maxYawRateChanged(self):
        logger.debug("MaxYawrate changed to %d", self.maxYawRate.value())
        self.helper.inputDeviceReader.updateMaxYawRateSignal.emit(
            self.maxYawRate.value())
        if (self.isInCrazyFlightmode == True):
            Config().set("max_yaw", self.maxYawRate.value())

    def maxAngleChanged(self):
        logger.debug("MaxAngle changed to %d", self.maxAngle.value())
        self.helper.inputDeviceReader.updateMaxRPAngleSignal.emit(
            self.maxAngle.value())
        if (self.isInCrazyFlightmode == True):
            Config().set("max_rp", self.maxAngle.value())

    def _trim_pitch_changed(self, value):
        logger.debug("Pitch trim updated to [%f]" % value)
        self.helper.inputDeviceReader.update_trim_pitch_signal.emit(value)
        Config().set("trim_pitch", value)

    def _trim_roll_changed(self, value):
        logger.debug("Roll trim updated to [%f]" % value)
        self.helper.inputDeviceReader.update_trim_roll_signal.emit(value)
        Config().set("trim_roll", value)

    def calUpdateFromInput(self, rollCal, pitchCal):
        logger.debug("Trim changed on joystick: roll=%.2f, pitch=%.2f",
                     rollCal, pitchCal)
        self.targetCalRoll.setValue(rollCal)
        self.targetCalPitch.setValue(pitchCal)

    def updateInputControl(self, roll, pitch, yaw, thrust):
        now = time()
        if (now - self.lastUIUpdate < 0.1): return
        self.lastUIUpdate = now

        self.targetRoll.setText(("%0.2f" % roll))
        self.targetPitch.setText(("%0.2f" % pitch))
        self.targetYaw.setText(("%0.2f" % yaw))
        self.targetThrust.setText(("%0.2f %%" %
                                   self.thrustToPercentage(thrust)))
        self.thrustProgress.setValue(thrust)

        """
          ADDITION: update display with latest tracker coordinate
        """
        global copter
        self.position.setText("%.3f %.3f %.3f" % copter.trackerPosition)
        # self.orientation.setText("%.3f %.3f %.3f %.3f" % copter.trackerOrientation)
        self.orientation.setText("y: %.1f p: %.1f r: %.1f" % copter.trackerYPR)
        self.velocity.setText("%.4f %.4f %.4f" % copter.velocity)
        self.target.setText("%.3f %.3f %.3f" % copter.targetPosition)
        if (copter.activePointer):
            self.pointer.setText("%.3f %.3f %.3f" % copter.pointerPosition)
        else:
            self.pointer.setText("none")
        self.RigidCnt.setText("%d" % copter.rigidCnt)
        self.SinglesCnt.setText("%d" % copter.singlesCnt)
        self.UpdateRate.setText("%.1f / sec" % copter.updateRate())

        self.ai.setRollPitch(-copter.trackerYPR[2], copter.trackerYPR[1])

    def setMotorLabelsEnabled(self, enabled):
        self.M1label.setEnabled(enabled)
        self.M2label.setEnabled(enabled)
        self.M3label.setEnabled(enabled)
        self.M4label.setEnabled(enabled)

    def emergencyStopStringWithText(self, text):
        return ("<html><head/><body><p>"
                "<span style='font-weight:600; color:#7b0005;'>{}</span>"
                "</p></body></html>".format(text))

    def updateEmergencyStop(self, emergencyStop):
        if emergencyStop:
            self.setMotorLabelsEnabled(False)
            self.emergency_stop_label.setText(
                self.emergencyStopStringWithText("Kill switch active"))
        else:
            self.setMotorLabelsEnabled(True)
            self.emergency_stop_label.setText("")

    def flightmodeChange(self, item):
        Config().set("flightmode", self.flightModeCombo.itemText(item))
        logger.info("Changed flightmode to %s",
                    self.flightModeCombo.itemText(item))
        self.isInCrazyFlightmode = False
        if (item == 0):  # Normal
            self.maxAngle.setValue(Config().get("normal_max_rp"))
            self.maxThrust.setValue(Config().get("normal_max_thrust"))
            self.minThrust.setValue(Config().get("normal_min_thrust"))
            self.slewEnableLimit.setValue(Config().get("normal_slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                Config().get("normal_slew_rate"))
            self.maxYawRate.setValue(Config().get("normal_max_yaw"))
        if (item == 1):  # Advanced
            self.maxAngle.setValue(Config().get("max_rp"))
            self.maxThrust.setValue(Config().get("max_thrust"))
            self.minThrust.setValue(Config().get("min_thrust"))
            self.slewEnableLimit.setValue(Config().get("slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                Config().get("slew_rate"))
            self.maxYawRate.setValue(Config().get("max_yaw"))
            self.isInCrazyFlightmode = True

        if (item == 0):
            newState = False
        else:
            newState = True
        self.maxThrust.setEnabled(newState)
        self.maxAngle.setEnabled(newState)
        self.minThrust.setEnabled(newState)
        self.thrustLoweringSlewRateLimit.setEnabled(newState)
        self.slewEnableLimit.setEnabled(newState)
        self.maxYawRate.setEnabled(newState)

    @pyqtSlot(bool)
    def changeXmode(self, checked):
        self.helper.cf.commander.set_client_xmode(checked)
        Config().set("client_side_xmode", checked)
        logger.info("Clientside X-mode enabled: %s", checked)
