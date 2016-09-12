#!/usr/bin/env python

"""@@This node check AUV depth and altitude, and is mainly
   used to avoid collisions.@@"""

"""
Created on Mar 2015
Modified 11/2015

@author: narcis palomeras
"""

# ROS imports
import rospy
from cola2_lib import cola2_ros_lib
from cola2_lib.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan


class CheckSensors(object):
    """This node is able to check altitude and depth."""

    def __init__(self, name):
        """Init the class."""
        # Init class vars
        self.name = name
        self.check_bumblebee = False
        self.check_profiler = False
        self.check_micron = False

        self.last_bumblebee_update = 0.0
        self.last_profiler_update = 0.0
        self.last_micron_update = 0.0

        # Get config parameters
        self.get_config()

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Subscriber
        rospy.Subscriber("/stereo_camera/left/camera_info",
                         CameraInfo,
                         self.update_bumblebee,
                         queue_size=1)

        rospy.Subscriber("/tritech_seaking_profiler_slice",
                         LaserScan,
                         self.update_profiler,
                         queue_size=1)

        rospy.Subscriber("/tritech_micron_slice",
                         LaserScan,
                         self.update_micron,
                         queue_size=1)

        # Show message
        rospy.loginfo("%s: initialized", self.name)

        # Timer
        rospy.Timer(rospy.Duration(1.0), self.check_timers)

    def update_bumblebee(self, data):
        """Update bumblebee data."""
        self.last_bumblebee_update = rospy.Time.now().to_sec()

    def update_profiler(self, data):
        """Update profiler data."""
        self.last_profiler_update = rospy.Time.now().to_sec()

    def update_micron(self, data):
        """Update micron data."""
        self.last_micron_update = rospy.Time.now().to_sec()

    def check_timers(self, event):
        """Check last data received and send error if bigger than thrshold."""
        now = rospy.Time.now().to_sec()
        is_ok = True

        if self.check_bumblebee:
            self.diagnostic.add("bumbleblee_last_update",
                                str(now - self.last_bumblebee_update))
            if now - self.last_bumblebee_update > 2.0:
                self.diagnostic.setLevel(DiagnosticStatus.WARN,
                                         "No Bumblebee images!")
                is_ok = False

        if self.check_profiler:
            self.diagnostic.add("profiler_last_update",
                                str(now - self.last_profiler_update))
            if now - self.last_profiler_update > 2.0:
                self.diagnostic.setLevel(DiagnosticStatus.WARN,
                                         "No Profiler data!")
                is_ok = False

        if self.check_micron:
            self.diagnostic.add("micron_last_update",
                                str(now - self.last_micron_update))
            if now - self.last_micron_update > 2.0:
                self.diagnostic.setLevel(DiagnosticStatus.WARN,
                                         "No Micron data!")
                is_ok = False

        if is_ok:
            self.diagnostic.setLevel(DiagnosticStatus.OK)

    def get_config(self):
        """Get config data from ros param server."""
        param_dict = {'check_bumblebee': 'check_sensors/bumblebee',
                      'check_profiler': 'check_sensors/profiler',
                      'check_micron': 'check_sensors/micron'}

        cola2_ros_lib.getRosParams(self, param_dict, self.name)

if __name__ == '__main__':
    try:
        rospy.init_node('check_sensors')
        CS = CheckSensors(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
