#!/usr/bin/env python

"""
Created on Fri Mar 22 2013
Modified 11/2015

@author: narcis palomeras
"""

# ROS imports
import roslib
import rospy

from auv_msgs.msg import BodyVelocityReq
from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from std_srvs.srv import Empty, EmptyResponse
from cola2_lib import cola2_ros_lib
import cola2_lib
import math
import random

class SafeDepthAltitude(object):
    def __init__(self, name):

        # Init class vars
        self.name = name
        self.nav = NavSts()
        self.trap_counter = 0

        # Get config parameters
        # self.getConfig()

        # Publisher
        self.pub_body_velocity_req = rospy.Publisher(
            "/cola2_control/body_velocity_req", 
            BodyVelocityReq,
            queue_size = 2)

        # Subscriber
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.updateNavSts,
                         queue_size = 1)

        rospy.Subscriber("/cola2_control/merged_body_velocity_req",
                         BodyVelocityReq,
                         self.updateBodyVelReq,
                         queue_size = 1)

        # Services
        self.load_trajectory_srv = rospy.Service(
                '/cola2_safety/enable_trap_avoidance',
                Empty,
                self.trapAvoidance)


    def updateNavSts(self, nav):
        self.nav = nav


    def updateBodyVelReq(self, bvr):
        # Only if the vehicle is moving forward
        if bvr.twist.linear.x > 0.0:
            # If current velocity is below 1/4 of desired velocity....
            if self.nav.body_velocity.x < bvr.twist.linear.x/4.0:
                self.trap_counter = self.trap_counter + 1
            else:
                self.trap_counter = 0


            #... for more than 1 minute
            if self.trap_counter > 600:
                rospy.logwarn('%s, VEHICLE TRAPPED!!!', self.name)
                self.trapAvoidanceMethod()
                self.trap_counter = 0


    def trapAvoidance(self, event):
        self.trapAvoidanceMethod()
        return EmptyResponse()


    def trapAvoidanceMethod(self):
        trap_orientation = self.nav.orientation.yaw
        bvr = BodyVelocityReq()
        bvr.disable_axis.x = False
        bvr.disable_axis.y = False
        bvr.disable_axis.z = False
        bvr.disable_axis.roll = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw = False
        bvr.goal.priority =  GoalDescriptor.PRIORITY_MANUAL_OVERRIDE - 1
        bvr.goal.requester = self.name

        # Step 1: Move backwards
        bvr.twist.linear.x = -0.25
        r = rospy.Rate(10)
        for i in range(8 * 10):
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
            r.sleep()

        # Step 2: back and turn
        random.seed()
        if random.random() > 0.5:
            bvr.twist.angular.z = 0.3
        else:
            bvr.twist.angular.z = -0.3

        while abs(cola2_lib.normalizeAngle(self.nav.orientation.yaw - cola2_lib.normalizeAngle(trap_orientation + math.pi))) > 0.15:
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
            r.sleep()

        # Step 3: forward and turn
        bvr.twist.linear.x = 0.25
        while abs(cola2_lib.normalizeAngle(self.nav.orientation.yaw - trap_orientation)) > 0.15:
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
            r.sleep()


    def getConfig(self):
        param_dict = {'max_depth': 'safety_g500/max_depth',
                      'min_altitude': 'safety_g500/min_altitude'}
        cola2_ros_lib.getRosParams(self, param_dict)


if __name__ == '__main__':
    try:
        rospy.init_node('trap_avoidance')
        safe_depth_altitude = SafeDepthAltitude(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException: pass
