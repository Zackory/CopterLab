import socket
import struct

__author__ = 'CopterLab'


class NatNet():
    """
      receive tracking data and update copter
    """

    def __init__(self):
        # NatNet message IDs
        self.NAT_PING = 0
        self.NAT_PINGRESPONSE = 1
        self.NAT_REQUEST = 2
        self.NAT_RESPONSE = 3
        self.NAT_REQUEST_MODELDEF = 4
        self.NAT_MODELDEF = 5
        self.NAT_REQUEST_FRAMEOFDATA = 6
        self.NAT_FRAMEOFDATA = 7
        self.NAT_MESSAGESTRING = 8
        self.NAT_UNRECOGNIZED_REQUEST = 100

        self.SERVER_ADDRESS = "138.49.29.17"
        # SERVER_ADDRESS = "138.49.196.130"
        # SERVER_ADDRESS = "138.49.134.244"
        self.CMD_PORT = 1510
        self.DATA_PORT = 1511

    def manage_cmd_socket(self, copter):
        print "starting NatNet receiver ..."
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        while True:
            try:
                resp, sender = s.recvfrom(1500)
                msgid, = struct.unpack("@h", resp[0:2])
                if msgid == self.NAT_MODELDEF:
                    print "Received Model Def Packet"
                elif msgid == self.NAT_FRAMEOFDATA:
                    framenum, bodies, unknown = self.unpack_data_frame(resp)
                    #        print "frame:", framenum
                    #        for i in range(len(bodies)):
                    #          print "body:", i, "pos:", bodies[i]["pos"], "orient:", bodies[i]["orient"]
                    # copter.updateTracking(framenum,bodies[0]["pos"],bodies[0]["orient"])
                    copter.updateTracking(framenum, bodies, unknown)
            except socket.timeout:
                print 'pinging'
                req = struct.pack("@hh", self.NAT_PING, 0)
                s.sendto(req, (self.SERVER_ADDRESS, self.CMD_PORT))

    def unpack_data_frame(self, data):
        # print "unpack_data_frame"
        mark = 0
        msgid, cnt = struct.unpack("@hh", data[mark:mark+4])
        mark += 4
        # print "msgid:", msgid, "cnt: ", cnt

        framenum, markersetcnt = struct.unpack("@ii", data[mark:mark+8])
        mark += 8
        # print "framenum:", framenum

        # print "Rigid Bodies:", markersetcnt
        i = markersetcnt
        while i > 0:
            namelen = data.find('\0', mark, mark+255) - mark
            name = data[mark:mark+namelen]
            mark += namelen+1
            markercnt, = struct.unpack("@i",data[mark:mark+4])
            mark += 4
            # print "\tname:", name, "markercnt:", markercnt
            j = markercnt
            while j > 0:
                x, y, z = struct.unpack("@fff", data[mark:mark+12])
                mark += 12
                # print "\t\tpos:", x, y, z
                j -= 1
            i -= 1

        umarkercnt, = struct.unpack("@i", data[mark:mark+4])
        mark += 4
        unknown = []
        # print "Unidentified Markers:", umarkercnt
        i = umarkercnt
        while i > 0:
            x, y, z = struct.unpack("@fff", data[mark:mark+12])
            mark += 12
            # print "\tpos:", x, y, z
            unknown.append(dict(pos=(x, y, z)))
            i -= 1

        rigidbodycnt, = struct.unpack("@i", data[mark:mark+4])
        mark += 4
        bodies = []
        # print "Rigid Bodies:", rigidbodycnt
        i = rigidbodycnt
        while i > 0:
            bodyid, = struct.unpack("@i", data[mark:mark+4])
            mark += 4
            x, y, z = struct.unpack("@fff", data[mark:mark+12])
            mark += 12
            a, b, c, d = struct.unpack("@ffff", data[mark:mark+16])
            mark += 16
            bodymarkercnt, = struct.unpack("@i", data[mark:mark+4])
            mark += 4
            mark += bodymarkercnt * (12 + 4 + 4)
            markererr, = struct.unpack("@f", data[mark:mark+4])
            mark += 4
            # print "\tBody Id:", bodyid, "pos:", x, y, z, "orient:", a, b, c, d, "marker err:", markererr
            bodies.append(dict(pos=(x, y, z), orient=(a, b, c, d)))
            i -= 1

        return framenum, bodies, unknown

