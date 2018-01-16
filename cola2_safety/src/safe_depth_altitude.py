#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



import rospy
from auv_msgs.msg import BodyVelocityReq
from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from cola2_lib import cola2_ros_lib
from dynamic_reconfigure.server import Server
from cola2_msgs.cfg import SafeDepthAltitudeConfig
from cola2_lib.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus

"""
 @@> Prevents the vehicle to go behind a maximum depth or a minimum altitude 
by sending a BodyVelocityReq asking a negative heave with maximum priority.<@@
"""
"""
Created on Fri Mar 22 2013
Modified 11/2015
@author: narcis palomeras
"""


class SafeDepthAltitude(object):
    """This node is able to check altitude and depth."""

    def __init__(self, name):
        """Init the class."""
        # Init class vars
        self.name = name

        # Get config parameters
        self.max_depth = 1.0
        self.min_altitude = 5.0
        self.get_config()

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Publisher
        self.pub_body_velocity_req = rospy.Publisher(
            "/cola2_control/body_velocity_req",
            BodyVelocityReq,
            queue_size=2)

        # Subscriber
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav_sts,
                         queue_size=1)

        # Create dynamic reconfigure servoce
        self.dynamic_reconfigure_srv = Server(SafeDepthAltitudeConfig,
                                              self.dynamic_reconfigure_callback)

        # Show message
        rospy.loginfo("%s: initialized", self.name)

    def dynamic_reconfigure_callback(self, config, level):
        """Dynamic reconfigure callback."""
        rospy.loginfo(
           "Reconfigure Request: {min_altitude}, {max_depth}".format(**config))
        self.max_depth = config.max_depth
        self.min_altitude = config.min_altitude
        return config

    def update_nav_sts(self, nav):
        """Navigation callback. It triggers the main loop."""
        self.diagnostic.add("altitude", str(nav.altitude))
        self.diagnostic.add("depth", str(nav.position.depth))

        if (nav.altitude > 0 and nav.altitude < self.min_altitude and nav.position.depth > 0.5) or (nav.position.depth > self.max_depth):
            # Show message
            self.diagnostic.setLevel(DiagnosticStatus.WARN, 'Invalid depth/altitude! Moving vehicle up.')
            if (nav.altitude > 0 and nav.altitude < self.min_altitude and nav.position.depth > 0.5):
                rospy.logwarn("%s: invalid altitude: %s",
                              self.name, nav.altitude)
            if (nav.position.depth > self.max_depth):
                rospy.logwarn("%s: invalid depth: %s",
                              self.name, nav.position.depth)

            # Go up
            bvr = BodyVelocityReq()
            bvr.twist.linear.x = 0.0
            bvr.twist.linear.y = 0.0
            bvr.twist.linear.z = -0.5
            bvr.twist.angular.x = 0.0
            bvr.twist.angular.y = 0.0
            bvr.twist.angular.z = 0.0
            bvr.disable_axis.x = True
            bvr.disable_axis.y = True
            bvr.disable_axis.z = False
            bvr.disable_axis.roll = True
            bvr.disable_axis.pitch = True
            # If yaw is True the vehicle can rotate while it goes up
            # but if it is False then it can not be teleoperated
            bvr.disable_axis.yaw = True
            bvr.goal.priority = GoalDescriptor.PRIORITY_SAFETY_HIGH
            bvr.goal.requester = self.name
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
        else:
            self.diagnostic.setLevel(DiagnosticStatus.OK)

    def get_config(self):
        """Get config from ROS param server."""
        param_dict = {'max_depth': 'safe_depth_altitude/max_depth',
                      'min_altitude': 'safe_depth_altitude/min_altitude'}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            self.bad_config_timer = rospy.Timer(rospy.Duration(0.4),
                                                self.bad_config_message)

    def bad_config_message(self, event):
        """Timer to show an error if loading parameters failed."""
        rospy.logerr('%s: bad parameters in param server!', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('safe_depth_altitude')
        safe_depth_altitude = SafeDepthAltitude(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
