#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



"""
@@>Provides services to run and stop the launch file launch/bag.launch <@@
"""

import rospy
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
import subprocess
import time


class LogBag:
    """LogBag class."""
 
    def __init__(self):
        """Class constructor."""
        self.start_bag = rospy.Service('/enable_logs', Empty, self.enable_logs)
        self.stop_bag = rospy.Service('/disable_logs', Empty,
                                      self.disable_logs)
        self.bag_enabled = False
        rospy.loginfo("This node will execute the launch file cola2_lib/launch/bag.launch")

    def enable_logs(self, req):
        """Run launch/bag.launch file."""
        command = "roslaunch cola2_lib bag.launch"
        subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd="./")
        self.bag_enabled = True
        rospy.loginfo("Launched bag file")
        return EmptyResponse()

    def disable_logs(self, req):
        """Stop launch/bag.launch file."""
        if self.bag_enabled:
            nodes = subprocess.check_output(['rosnode', 'list']).split()
            for node in nodes:
                if node.startswith("/record_auv"):
                    subprocess.call(['rosnode', 'kill', node])
                    time.sleep(1.0)

            # killcommand = "kill -9 " + str(self.pid+1)
            # subprocess.Popen(killcommand, shell=True)
            # rospy.loginfo("Kill process: " + killcommand)
            self.bag_enabled = False
        else:
            rospy.logwarn("Bag is not enabled!")

        return EmptyResponse()

if __name__ == '__main__':
    """ Main function. """
    rospy.init_node('bag_node')
    log = LogBag()
    rospy.spin()
