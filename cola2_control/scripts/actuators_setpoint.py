#!/usr/bin/env python

import roslib
import rospy

from cola2_msgs.msg import Setpoints
from cola2_lib import cola2_ros_lib
from cola2_msgs.srv import Action, ActionRequest, ActionResponse

from math import *
import numpy as np


class ActuatorsSetpoint:
    """ Used just to send data to thrusters """

    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Publisher
        self.pub_thrusters = rospy.Publisher("/cola2_control/thrusters_data",
                                             Setpoints,
                                             queue_size = 2)

        # Create enable thrusters service
        self.test_srv = rospy.Service('/calibration/actuators_setpoint',
                                      Action,
                                      self.enable_thrusters)

    def enable_thrusters(self, req):
        """Thrusters service."""
        rospy.logwarn("%s: Disable Thrusters before executing this script!!!",
                      self.name)
        if len(req.param) > 1:
            setpoints = list()
            for i in range(len(req.param) - 1):
                setpoints.append(float(req.param[i]))
            timeout = float(req.param[-1])
            text = '{0}: Execute thrusters with setpoint {1} for {2} seconds.'.format(self.name, setpoints, timeout)
            rospy.loginfo(text)
            self.send_trhuster_setpoint(setpoints, timeout)
        else:
            rospy.logwarn("%s: specify setpoint per thruster and timeout",
                          self.name)

        return ActionResponse()

    def send_trhuster_setpoint(self, setpoints, timeout):
        """Send thruster setpoints."""
        # Publish to thrusters
        rate = rospy.Rate(10)
        for i in range(int(timeout * 10)):
            # rospy.loginfo("%s: sending: %s", self.name, setpoints)
            msg = Setpoints()
            msg.header.stamp = rospy.Time.now()
            msg.setpoints = setpoints
            rate.sleep()
            self.pub_thrusters.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('actuators_setpoint')
        setpoint = ActuatorsSetpoint(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
