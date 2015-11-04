#!/usr/bin/env python
"""
Copyright (c) 2014, 'CIRS - Universitat de Girona'
All rights reserved.

Software is provided WITHOUT WARRANTY and software authors and license owner
'CIRS - Universitat de Girona' cannot be held liable for any damages. You may
use and modify this software for PRIVATE USE. REDISTRIBUTION in source and
binary forms, with or without modification, are FORBIDDEN. You may NOT use the
names, logos, or trademarks of contributors.
"""


"""@@Computer logger is used to record valuable data from the computer.@@"""


# ROS imports
import rospy
import subprocess
import psutil
from cola2_msgs.msg import ComputerData


class ComputerLogger(object):
    """ This node is used to log some computer data """

    def __init__(self, name):
        """ Constructor """
        # Init class vars
        self.name = name

        # Publisher
        self.pub_computer_data = rospy.Publisher(
             "/cola2_safety/computer_logger", 
             ComputerData,
             queue_size = 2)

        # Start timer
        rospy.Timer(rospy.Duration(5.0), self.iterate)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def iterate(self, event):
        """ Callback from the main timer """
        try:
            # Execute sensors command and parse output
            p = subprocess.Popen(['sensors'],
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            out, err = p.communicate()
            splitted = out.split("\n")

            cpu_temp = -99
            fan_rpm = -99
            max_core_temp = -99

            for line in splitted:
                #if "CPU Temperature:" in line:
                #    end = line.index("(") - 6
                #    start = end - 6
                #    cpu_temp = float(line[start:end])
                #if "CPU FAN Speed:" in line:
                #    end = line.index("(") - 5
                #    start = end - 6
                #    fan_rpm = float(line[start:end])
                if "Core" in line:
                    end = line.index("(") - 6
                    start = end - 6
                    temp = float(line[start:end])
                    if temp > max_core_temp:
                        max_core_temp = temp

            # Use psutil to get cpu and ram usage
            cpu_usage = psutil.cpu_percent()
            ram_usage = ram = psutil.phymem_usage().percent

            msg = ComputerData()
            msg.header.stamp = rospy.Time.now()
            msg.cpu_temp = max_core_temp
            msg.cpu_usage = cpu_usage
            msg.ram_usage = ram_usage
            self.pub_computer_data.publish(msg)

        except:
            rospy.logwarn("%s: unable to get data!", self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('computer_logger')
        computer_logger = ComputerLogger(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
