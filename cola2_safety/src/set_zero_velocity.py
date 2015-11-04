#!/usr/bin/env python
"""@@If the robot is deep enough and teleoperation is giving only disabled setpoints,
this node tells the robot to keep velocities to zero.@@"""

"""
Created on Fri Mar 22 2013
Modified 11/2015

@author: narcis palomeras
"""

# ROS imports
import roslib
import rospy

import threading

from auv_msgs.msg import WorldWaypointReq
from auv_msgs.msg import BodyVelocityReq
from auv_msgs.msg import BodyForceReq

from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from cola2_lib import cola2_ros_lib


class SetZeroVelocity(object):
    """ This class generates several BodyVelocityReq set at 0 enabling
        only the axis selected in the configuration file when the vehicle
        is below a configured depth. This provoques that the vehicle keeps
        its velocity at 0 below the desired depth. As the priority of this
        behavior is minimum and it can send commands for each DoF indepen-
        dently, it is easy to merge with other pose or velocity requests.
        WARNING: If force requests are used it is better to disable here the
        axis that the force controller is trying to achieve!"""

    def __init__(self, name):
        """ Initialize the class """
        # Init class vars
        self.name = name
        self.navigation = NavSts()
        self.set_zero_velocity_depth = 2.0
        self.set_zero_velocity_axis = [[False, False, False,
                                        False, False, False]]
        self.current_enabled_axis = [False, False, False, False, False, False]
        self.lock = threading.Lock()

        # Get config parameters
        self.get_config()

        # Publisher
        self.pub_body_velocity_req = rospy.Publisher(
            "/cola2_control/body_velocity_req", 
            BodyVelocityReq,
            queue_size = 10)

        # Subscriber
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav_sts,
                         queue_size = 1)
                         
        rospy.Subscriber("/cola2_control/world_waypoint_req",
                         WorldWaypointReq,
                         self.update_req,
                         queue_size = 1)
                         
        rospy.Subscriber("/cola2_control/body_velocity_req",
                         BodyVelocityReq,
                         self.update_req,
                         queue_size = 1)

        rospy.Subscriber("/cola2_control/body_force_req",
                         BodyForceReq,
                         self.update_req,
                         queue_size = 1)

        # Timer
        rospy.Timer(rospy.Duration(0.1), self.set_zero_velocity)

        # Show message
        rospy.loginfo("%s: initialized", self.name)



    def update_req(self, req):
        """ Update active axis req """
        self.lock.acquire()
        if not("set_zero_velocity_" in req.goal.requester):
            if req.goal.priority > 1:
                if req.header.stamp.to_sec() + 0.2 > rospy.Time.now().to_sec():
                    if not req.disable_axis.x:
                        self.current_enabled_axis[0] = True
                    if not req.disable_axis.y:
                        self.current_enabled_axis[1] = True
                    if not req.disable_axis.z:
                        self.current_enabled_axis[2] = True
                    if not req.disable_axis.roll:
                        self.current_enabled_axis[3] = True
                    if not req.disable_axis.pitch:
                        self.current_enabled_axis[4] = True
                    if not req.disable_axis.yaw:
                        self.current_enabled_axis[5] = True                        
        self.lock.release()
        
        
    def update_nav_sts(self, nav):
        """ Updates vehicle depth """
        self.navigation = nav


    def set_zero_velocity(self, event):
        """ Send zero velocity requests if the vehicle is below the
            desired depth """
            
        self.lock.acquire()
        if self.navigation.position.depth > self.set_zero_velocity_depth:
            bvr = BodyVelocityReq()
            bvr.twist.linear.x = 0.0
            bvr.twist.linear.y = 0.0
            bvr.twist.linear.z = 0.0
            bvr.twist.angular.x = 0.0
            bvr.twist.angular.y = 0.0
            bvr.twist.angular.z = 0.0

            bvr.goal.priority =  GoalDescriptor.PRIORITY_LOW + 1
            bvr.header.stamp = rospy.Time.now()

            for i in range(len(self.set_zero_velocity_axis)):
                if self.current_enabled_axis[0]:
                    bvr.disable_axis.x = True 
                else:
                    bvr.disable_axis.x = self.set_zero_velocity_axis[i][0]

                if self.current_enabled_axis[1]:
                    bvr.disable_axis.y = True 
                else:
                    bvr.disable_axis.y = self.set_zero_velocity_axis[i][1]

                if self.current_enabled_axis[2]:
                    bvr.disable_axis.z = True 
                else:
                    bvr.disable_axis.z = self.set_zero_velocity_axis[i][2]

                if self.current_enabled_axis[3]:
                    bvr.disable_axis.roll = True 
                else:
                    bvr.disable_axis.roll = self.set_zero_velocity_axis[i][3]

                if self.current_enabled_axis[4]:
                    bvr.disable_axis.pitch = True 
                else:
                    bvr.disable_axis.pitch = self.set_zero_velocity_axis[i][4]

                if self.current_enabled_axis[5]:
                    bvr.disable_axis.yaw = True 
                else:                
                    bvr.disable_axis.yaw = self.set_zero_velocity_axis[i][5]

                # Set Zero Velocity
                bvr.goal.requester = 'set_zero_velocity_' + str(i)
                self.pub_body_velocity_req.publish(bvr)

        # Disable all axis
        self.current_enabled_axis = [False, False, False, False, False, False]
        self.lock.release()


    def get_config(self):
        """ Reads configuration from ROSPARAM SERVER """
        param_dict = {'set_zero_velocity_depth': 'safety_set_zero_velocity/set_zero_velocity_depth',
                      'set_zero_velocity_axis': 'safety_set_zero_velocity/set_zero_velocity_axis'}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            self.bad_config_timer = rospy.Timer(rospy.Duration(0.4), self.bad_config_message)


    def bad_config_message(self, event):
        """ Timer to show an error if loading parameters failed """
        rospy.logerr('%s: bad parameters in param server!', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('set_zero_velocity')
        __set_zero_velocity__ = SetZeroVelocity(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
