#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""
@@>This node tells the robot to keep some DoF to a specific pose, provided that it is deep enough.
This functionality use to be used for keeping an specific Pitch or Roll.<@@
"""

"""
Created on Fri Mar 11 20134
Modified 11/2015
@author: narcis palomeras
"""

# ROS imports
import roslib
import rospy

from auv_msgs.msg import WorldWaypointReq
from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from cola2_lib import cola2_ros_lib


class SetPose(object):
    """ This class generates several WorldWaypointReq set at 0 enabling
        only the axis selected in the configuration file when the vehicle
        is below a configured depth. This provoques that the vehicle keeps
        its pose as specified below the desired depth. As the priority of this
        behavior is minimum and it can send commands for each DoF indepen-
        dently, it is easy to merge with other pose or velocity requests.
        WARNING: If force requests are used it is better to disable here the
        axis that the force controller is trying to achieve!"""

    def __init__(self, name):
        """ Initialize the class """
        # Init class vars
        self.name = name
        self.navigation = NavSts()
        self.set_pose_depth = 2.0
        self.set_pose_axis = [[True, True, True, True, True, True]]
        self.desired_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Get config parameters
        self.get_config()

        # Publisher
        self.pub_world_waypoint_req = rospy.Publisher(
            "/cola2_control/world_waypoint_req",
            WorldWaypointReq,
            queue_size = 2)

        # Subscriber
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav_sts,
                         queue_size = 1)

        # Timer
        rospy.Timer(rospy.Duration(0.1), self.set_pose)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def update_nav_sts(self, nav):
        """ Updates vehicle depth """
        self.navigation = nav


    def set_pose(self, event):
        """ Send zero velocity requests if the vehicle is below the
            desired depth """

        if self.navigation.position.depth > self.set_pose_depth:
            wwr = WorldWaypointReq()
            wwr.position.north = self.desired_pose[0]
            wwr.position.east = self.desired_pose[1]
            wwr.position.depth = self.desired_pose[2]
            wwr.orientation.roll = self.desired_pose[3]
            wwr.orientation.pitch = self.desired_pose[4]
            wwr.orientation.yaw = self.desired_pose[5]
            wwr.altitude_mode = False
            wwr.goal.priority =  GoalDescriptor.PRIORITY_SAFETY_LOW
            wwr.header.stamp = rospy.Time.now()

            for i in range(len(self.set_pose_axis)):
                wwr.disable_axis.x = self.set_pose_axis[i][0]
                wwr.disable_axis.y = self.set_pose_axis[i][1]
                wwr.disable_axis.z = self.set_pose_axis[i][2]
                wwr.disable_axis.roll = self.set_pose_axis[i][3]
                wwr.disable_axis.pitch = self.set_pose_axis[i][4]
                wwr.disable_axis.yaw = self.set_pose_axis[i][5]

                # Set Zero Velocity
                wwr.goal.requester = 'set_pose_' + str(i)
                self.pub_world_waypoint_req.publish(wwr)


    def get_config(self):
        """ Reads configuration from ROSPARAM SERVER """
        param_dict = {'set_pose_depth': 'safety_set_pose/set_pose_depth',
                      'set_pose_axis': 'safety_set_pose/set_pose_axis',
                      'desired_pose': 'safety_set_pose/desired_pose'}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            self.bad_config_timer = rospy.Timer(rospy.Duration(0.4),
                                                self.bad_config_message)


    def bad_config_message(self, event):
        """ Timer to show an error if loading parameters failed """
        rospy.logerr('%s: bad parameters in param server!', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('set_pose')
        __set_zero_velocity__ = SetPose(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
