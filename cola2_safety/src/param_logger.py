#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Publishes all parameters to be saved in a bag."""

import rospy
from std_msgs.msg import String


def callback(event):
    """Callback to collect all parameters."""
    strings = list()
    for name in rospy.get_param_names():
        if not name.startswith('/robot_description'):
            if not name.startswith('/roslaunch/uris'):
                value = rospy.get_param(name)
                line = '{:s}={}'.format(name, value)
                strings.append(line)
    msg = String()
    msg.data = '\n'.join(strings)
    pub.publish(msg)


if __name__ == '__main__':
    # Init node
    rospy.init_node('param_logger')
    pub = rospy.Publisher('/params_string', String, queue_size=1, latch=True)
    tim = rospy.Timer(rospy.Duration(120), callback, oneshot=True)
    rospy.spin()
