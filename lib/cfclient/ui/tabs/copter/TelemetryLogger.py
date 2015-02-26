from time import time

__author__ = 'CopterLab'


class TelemetryLogger():

    def __init__(self):
        self.fileCnt = 0
        self.logCnt = 0
        self.logCntMax = 120 * 3
        self.logFile = None

    def startLog(self):
        if self.logFile is not None:
            self.stopLog()
        fileName = "/home/bitcraze/Desktop/projects/" + "log-" + "%d" % self.fileCnt
        self.fileCnt += 1
        self.logFile = open(fileName, "w")
        self.startTime = time()
        self.logCnt = 0

    def stopLog(self):
        f = self.logFile
        self.logFile = None
        f.close()
        self.logCnt = 0

    def logTelemetry(self,str):
        if self.logFile is None:
            return
        dt = time() - self.startTime
        self.logFile.write("%.6f   " % dt)
        self.logFile.write(str)
        self.logFile.write("\n")
        self.logCnt += 1
        if self.logCnt > self.logCntMax:
            self.stopLog()

