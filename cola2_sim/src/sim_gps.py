#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import socket
import sys
import time
import random
from threading import Thread
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D

def __degree2DegreeMinuteAux__(value):
    val = str(value).split('.')
    minute = float('0.' + val[1]) * 60.0
    if minute < 10.0:
        return float(val[0] + '0' + str(minute))
    else:
        return float(val[0] + str(minute))


def degree2DegreeMinute(lat, lon):
    lat_degree = __degree2DegreeMinuteAux__(lat)
    lon_degree = __degree2DegreeMinuteAux__(lon)
    return lat_degree, lon_degree


class SimGPS:
    def __init__(self, name):
        self.name = name
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        # get transform from gps to vessel reference point
        self.gps_tf = rospy.get_param('sim_gps/tf', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.tfBroadcaster = tf.TransformBroadcaster()
        self.gps_transf = [np.array(self.gps_tf[:3]), quaternion_from_euler(
            np.deg2rad(self.gps_tf[3]), np.deg2rad(self.gps_tf[4]), np.deg2rad(self.gps_tf[5]))]

        # create tcp server
        TCP_IP = '127.0.0.1'
        TCP_PORT = 4000
        print("Connect simulated GPS at {}:{}\n".format(TCP_IP, TCP_PORT))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)

        # subscribe to vessel position
        rospy.Subscriber("/cola2_sim/vessel_global_pose",
                         Pose2D,
                         self.update_vessel_pose,
                         queue_size=1)

        # Shutdown callback
        rospy.on_shutdown(self.shutdown)

        # Show message
        rospy.loginfo("%s: initialized", self.name)

        # and wait for connections
        self.run_server()


    def set_lat_lon_heading(self, lat_deg, lon_deg, heading):
        self.latitude = lat_deg
        self.longitude = lon_deg
        self.heading = heading

    def update_vessel_pose(self, data):
        """ Update gps position according to current vessel global pose. """

        self.set_lat_lon_heading(data.x, data.y, data.theta)

        # Publish tf from gps to vessel
        tim = rospy.Time.now()  # current ROS time
        self.tfBroadcaster.sendTransform(self.gps_transf[0], self.gps_transf[1], tim, '/gps',
                                         '/vessel')  # transform gps -> vessel

    def create_GPGGA(self, lat_deg, lon_deg):
        """
        Create GPGGA sequence.
        $GPGGA,time,lat,N/S,lon,E/W,fix,sat,hdop,alt,M,hgeo,M,,*chk
        """
        line_wo_chk = "$GPGGA,{:s},{:s},{:s},{:s},{:s},1,9,0.86,2.2,M,50.7,M,,"
        line = "$GPGGA,{:s},{:s},{:s},{:s},{:s},1,9,0.86,2.2,M,50.7,M,,*{:s}\r\n"
        tim = time.strftime("%H%M%S", time.gmtime())
        lat, lon = degree2DegreeMinute(lat_deg, lon_deg)
        north = 'N' if lat_deg > 0.0 else 'S'
        east = 'E' if lon_deg > 0.0 else 'W'

        # Generate sentence
        nmeadata = line_wo_chk.format(tim, str(lat), north, str(lon), east)

        # Compute checksum
        calc_cksum = 0
        for s in nmeadata:
            calc_cksum ^= ord(s)
        print(line.format(tim, str(lat), north, str(lon), east, str(calc_cksum)))
        return line.format(tim, str(lat), north, str(lon), east, str(calc_cksum))


    def create_GPHDT(self, yaw_deg):
        """
        Create GPHDT sequence.
        $GPHDT,XX.X,T,*chk
        """
        nmeadata = "$GPHDT,{:.1f},T".format(yaw_deg)

        # Compute checksum
        calc_cksum = 0
        for s in nmeadata:
            calc_cksum ^= ord(s)

        return "$GPHDT,{:.1f},T,*{:s}\r\n".format(yaw_deg, str(calc_cksum))

    def client_thread(self, clientsocket):
        """
        Send GGA and HDT sentences at aprox. 1Hz
        """
        print("New thread")
        while True:
            try:
                # send the fake data
                gga = self.create_GPGGA(self.latitude, self.longitude)
                #print(gga)
                clientsocket.send(gga.encode('utf-8'))
                time.sleep(0.3)
                hdt = self.create_GPHDT(self.heading)
                #print(hdt)
                clientsocket.send(hdt.encode('utf-8'))
                time.sleep(0.7)
            except:
                print("Connection closed.")
                clientsocket.close()
                break
        clientsocket.close()

    def run_server(self):
        """
        Keeps accepting connections one by one and creates thread to handle each client
        """
        while not rospy.is_shutdown():
            clientsocket, addr = self.s.accept()
            print('Connected from: {}'.format(addr))
            # Launch client thread
            th = Thread(target=self.client_thread, args=(clientsocket,))
            th.start()

    def shutdown(self):
        self.s.close()


if __name__ == '__main__':

    try:
        rospy.init_node('sim_gps')
        sim_gps = SimGPS(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



