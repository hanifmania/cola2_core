#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



"""
@@>Node to keep track of the cola2 running time, used by the safety supervisor to check it against the safety timeout.
It also checks if navigator is publishing data.<@@
"""

# ROS imports
import roslib
import rospy

from std_srvs.srv import Empty, EmptyResponse
from auv_msgs.msg import NavSts
from cola2_msgs.msg import TotalTime
from cola2_lib.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus
from cola2_lib import cola2_ros_lib


class UpTime(object):
    """ This node keeps track of the cola2 running time.
     Counts the up time since the architecture started (actually since the start of this node)
     or since the last time reset, which can be done through a provided service.
     Publishes the current up time together with the value of the safety timeout parameter. """


    def __init__(self, name):
        """ Constructor """
        self.name = name
        
        # Initial time
        self.init_time = rospy.Time.now().to_sec()

        # Initial timeout
        self.timeout = 3600

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Publisher
        self.pub_total_time = rospy.Publisher("/cola2_safety/total_time",
                                            TotalTime,
                                            queue_size = 2)

        # Create reset timeout service
        self.reset_timeout_srv = rospy.Service('/cola2_safety/reset_timeout',
                                        Empty,
                                        self.reset_timeout)
        
        # Timer to publish time since init (or last time reset)
        rospy.Timer(rospy.Duration(1.0), self.check_timeout)

        # Timer to reload timeout param from param server
        rospy.Timer(rospy.Duration(10.0), self.reload_timeout)

        # Check navigator --> This should not be in this node
        self.nav = NavSts()
        self.init_navigator_check = False
        self.last_navigator_callback = rospy.Time.now().to_sec()
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav_sts,
                         queue_size = 1)

        # Show message
        rospy.loginfo("%s: initialized", self.name)
        
        
    def check_timeout(self, event):
        """ This is the callback of the main timer """
        self.diagnostic.add("up_time",
                            str(rospy.Time.now().to_sec() - self.init_time))

        # This should not be in this node
        if self.init_navigator_check:
            self.diagnostic.add("last_nav_data",
                                str(rospy.Time.now().to_sec() - self.last_navigator_callback))

        self.diagnostic.setLevel(DiagnosticStatus.OK)

        # Publish total time
        msg = TotalTime()
        msg.total_time = int(rospy.Time.now().to_sec() - self.init_time)
        # publish also the value of the safety timeout parameter
        msg.timeout = self.timeout
        self.pub_total_time.publish(msg)


    def reset_timeout(self, req):
        """ Service used to reset timeout """
        rospy.loginfo("%s: Reset Timeout!", self.name)
        self.init_time = rospy.Time.now().to_sec()
        return EmptyResponse()

    def reload_timeout(self, event):
        """ Timer callback for periodically reloading safety timeout parameter from param server"""
        self.timeout = rospy.get_param('safety/timeout', 3600)


    def update_nav_sts(self, data):
        """ Navigator callback """
        self.nav = data
        self.last_navigator_callback = rospy.Time.now().to_sec()
        if not self.init_navigator_check:
            self.init_navigator_check = True


if __name__ == '__main__':
    try:
        rospy.init_node('timeout')
        UP_TIME = UpTime(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
