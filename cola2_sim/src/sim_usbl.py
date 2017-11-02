#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



"""
@@>This node simulates a USBL system deployed from a vessel<@@
"""

"""
Created 11/2017
@author: tali hurtos
"""

import rospy
import threading
import math
import numpy as np
from cola2_lib import cola2_ros_lib
import socket
from cola2_lib.NED import NED
from geometry_msgs.msg import PoseStamped
from auv_msgs.msg import NavSts
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class SimUSBL:

    def __init__(self, name):
        """ Init the class """
        self.name = name
        self.ned = None
        self.vehicle_position = [0.0, 0.0, 0.0]
        self.vessel_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.initialized = False

        # Get tf of usbl wrt vessel reference point from config
        self.usbl_tf = rospy.get_param('sim_usbl/tf', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.gps_tf = rospy.get_param('sim_gps/tf', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Subscribers
        self.nav_sts_sub_ = rospy.Subscriber("/cola2_navigation/nav_sts",
                                             NavSts, self.update_vehicle_position,
                                             queue_size=1)

        self.vessel_pose_sub_ = rospy.Subscriber("/cola2_sim/vessel_ned_pose",
                                                 PoseStamped, self.update_vessel_position,
                                                 queue_size=1)

        # Create TCP server to send USBLLONG sentences
        TCP_IP = '127.0.0.1'
        TCP_PORT = 9200
        print("Connect simulated USBL at {}:{}\n".format(TCP_IP, TCP_PORT))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)

        # Compute offset from GPS to USBL positions in vessel coordinates
        self.gps2usbl = []
        self.gps2usbl.append(self.usbl_tf[0] - self.gps_tf[0])
        self.gps2usbl.append(self.usbl_tf[1] - self.gps_tf[1])
        self.gps2usbl.append(self.usbl_tf[2] - self.gps_tf[2])
        print("GPS to USBL: " + str(self.gps2usbl))

        # Tfs
        self.tfBroadcaster = tf.TransformBroadcaster()
        self.usbl_transf = [np.array(self.usbl_tf[:3]), quaternion_from_euler(
            np.deg2rad(self.usbl_tf[3]), np.deg2rad(self.usbl_tf[4]), np.deg2rad(self.usbl_tf[5]))]

        # Shutdown callback
        rospy.on_shutdown(self.shutdown)

        # Show message
        rospy.loginfo("%s: initialized", self.name)

        # Start server
        self.run_server()

    def update_vehicle_position(self, data):
        """ Save current vehicle position. """
        self.vehicle_position = [data.position.north,
                                 data.position.east,
                                 data.position.depth]

    def update_vessel_position(self, data):
        """ Save current vessel position. """
        self.vessel_position[0] = data.pose.position.x
        self.vessel_position[1] = data.pose.position.y
        self.vessel_position[2] = data.pose.position.z
        self.vessel_position[3:6] = euler_from_quaternion([data.pose.orientation.x,
                                                           data.pose.orientation.y,
                                                           data.pose.orientation.z,
                                                           data.pose.orientation.w])
        self.initialized = True

    def send_usbllong(self, clientsocket):
        """ Send USBLLONG message to the socket client at aprox. 0.5Hz"""
        while True:
            if self.initialized:
                usbllong = self.compute_usbllong()
                print(usbllong)
                clientsocket.send(usbllong.encode('utf-8'))
                # Publish tf
                tim = rospy.Time.now()  # current ROS time
                self.tfBroadcaster.sendTransform(self.usbl_transf[0], self.usbl_transf[1], tim, '/usbl',
                                                 '/vessel')  # transform usbl -> vessel
                rospy.sleep(2.0)

    def compute_usbllong(self):
        """
        USBLLONG, < current time >, < measurement time >, < remote address >,
        < X >, < Y >, < Z >, < E >, < N >, < U >, < roll >, < pitch >, < yaw >,
        < propagation time >, < rssi >, < integrity >, < accuracy >
        """

        # Compute USBL position in NED coordinates

        # Compute offset according to orientation
        yaw = self.vessel_position[5]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw), np.cos(yaw)]])
        off_ned = rot.dot(np.array([[self.gps2usbl[0], self.gps2usbl[1]]]).T)
        off_ned = off_ned.ravel()

        print("Vessel position:", self.vessel_position)
        print("GPS2USBL Off:", self.gps2usbl)
        print("Off_ned:", off_ned)
        # Substract offsets from GPS (vessel) position (in NED) to obtain USBL position (in NED).
        usbl = [self.vessel_position[0] + off_ned[0],
                self.vessel_position[1] + off_ned[1],
                self.vessel_position[2] + self.gps2usbl[2]]

        print("USBL position:", usbl)

        # Compute vehicle position with respect to the USBL position (north oriented, corrected by the IMU)
        n = self.vehicle_position[0] - usbl[0]
        e = self.vehicle_position[1] - usbl[1]
        d = self.vehicle_position[2] - usbl[2]

        curr_time = ""
        measure_time = ""
        remote_addr = 2 # 2 for sparus, 3 for girona
        x = 0.0
        y = 0.0
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        prop_time = ""
        rssi = 0
        integrity = 0
        acc = 0

        usbllong = "USBLLONG,{:s},{:s},{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}," \
                   "{:.3f},{:.3f},{:.3f},{:s},{:d},{:d},{:d}".format(
                    curr_time, measure_time, remote_addr, x, y, z, e, n, d,
                    roll, pitch, yaw, prop_time, rssi, integrity, acc)
        return usbllong

    def run_server(self):
        """
        Keeps accepting connections one by one and creates thread to handle each client
        """
        while not rospy.is_shutdown():
            clientsocket, addr = self.s.accept()
            print('Connected from: {}'.format(addr))
            # Launch client thread
            th = threading.Thread(target=self.send_usbllong, args=(clientsocket,))
            th.start()

    def shutdown(self):
        self.s.close()


if __name__ == '__main__':
    try:
        rospy.init_node('sim_usbl')
        sim_usbl = SimUSBL(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
