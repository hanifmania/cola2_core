#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



"""
@@>This node simulates a vessel<@@
"""

"""
Created 11/2017
@author: tali hurtos
"""

import rospy
import numpy as np
from cola2_lib.NED import NED
from geometry_msgs.msg import Pose2D, PoseStamped
from auv_msgs.msg import NavSts
from cola2_msgs.srv import SetVesselPose, SetVesselPoseResponse
import tf
from tf.transformations import quaternion_from_euler


class SimVessel:

    def __init__(self, name):
        """ Init the class """
        self.name = name
        self.ned = None
        self.vessel_position = [0.0, 0.0, 0.0]
        self.initialized = False

        # Publishers
        self.pub_vessel_ned_pose = rospy.Publisher(
            "/cola2_sim/vessel_ned_pose",
            PoseStamped,
            queue_size=1)

        self.pub_vessel_global_pose = rospy.Publisher(
            "/cola2_sim/vessel_global_pose",
            Pose2D,
            queue_size=1)

        # Subscribers
        self.nav_sts_sub_ = rospy.Subscriber("/cola2_navigation/nav_sts",
                                             NavSts, self.update_navigation,
                                             queue_size=1)

        # Services
        self.current_srv = rospy.Service('cola2_sim/set_vessel_pose', SetVesselPose, self.update_vessel_position)

        # Tfs
        self.tfBroadcaster = tf.TransformBroadcaster()
        self.vessel_transf = None  # We don't initialize the vessel transform until a pose is received from the service

        # Show message
        rospy.loginfo("%s: initialized", self.name)

        # Start main loop
        self.publish_poses()

    def update_navigation(self, data):
        """ Save current navigation data. """

        # If NED is not initialized yet or NED has changed create a new NED with NED origin values
        if self.ned is None or \
                        not np.isclose(np.rad2deg(self.ned.init_lat), data.origin.latitude, rtol=1e-05) or \
                        not np.isclose(np.rad2deg(self.ned.init_lon), data.origin.longitude, rtol=1e-05):

            rospy.loginfo("{}: Setting NED origin to: {},{}".format(self.name,
                                                                    data.origin.latitude,
                                                                    data.origin.longitude))
            self.ned = NED(data.origin.latitude, data.origin.longitude, 0.0)
            print("Now waiting vessel position...")

    def publish_poses(self):
        rate_vessel = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            if self.initialized:  # If Vessel position has been set
                # Publish tfs
                tim = rospy.Time.now()  # current ROS time
                self.tfBroadcaster.sendTransform(self.vessel_transf[0], self.vessel_transf[1], tim, '/vessel',
                                                 '/world')  # transform vessel -> world

                # Convert vessel ned pose to global using ned
                latlonh = self.ned.ned2geodetic((self.vessel_position[0],
                                                 self.vessel_position[1],
                                                 self.vessel_position[2]))
                # Publish vessel global pose
                vessel_global_pose = Pose2D()
                vessel_global_pose.x = latlonh[0]
                vessel_global_pose.y = latlonh[1]
                vessel_global_pose.theta = self.vessel_position[5]
                self.pub_vessel_global_pose.publish(vessel_global_pose)

                # Publish vessel ned pose
                vessel_ned_pose = PoseStamped()
                vessel_ned_pose.header.frame_id = "/vessel"
                vessel_ned_pose.header.stamp = rospy.Time.now()
                vessel_ned_pose.pose.position.x = self.vessel_position[0]
                vessel_ned_pose.pose.position.y = self.vessel_position[1]
                vessel_ned_pose.pose.position.z = self.vessel_position[2]
                vessel_quat = quaternion_from_euler(np.deg2rad(self.vessel_position[3]),
                                                    np.deg2rad(self.vessel_position[4]),
                                                    np.deg2rad(self.vessel_position[5]))
                vessel_ned_pose.pose.orientation.x = vessel_quat[0]
                vessel_ned_pose.pose.orientation.y = vessel_quat[1]
                vessel_ned_pose.pose.orientation.z = vessel_quat[2]
                vessel_ned_pose.pose.orientation.w = vessel_quat[3]
                self.pub_vessel_ned_pose.publish(vessel_ned_pose)

            rate_vessel.sleep()

    def update_vessel_position(self, request):
        """ Save current vessel position. """
        self.vessel_position = [request.pose.x,
                                request.pose.y,
                                0.0,
                                0.0,
                                0.0,
                                request.pose.theta]
        rospy.loginfo("{}: Setting Vessel Position to: {},{}".format(self.name,
                                                                     self.vessel_position[0],
                                                                     self.vessel_position[1]))

        self.vessel_transf = [np.array(self.vessel_position[:3]),
                              quaternion_from_euler(np.deg2rad(self.vessel_position[3]),
                                                    np.deg2rad(self.vessel_position[4]),
                                                    np.deg2rad(self.vessel_position[5]))]

        self.initialized = True

        return True


if __name__ == '__main__':
    try:
        rospy.init_node('sim_vessel')
        sim_vessel = SimVessel(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
