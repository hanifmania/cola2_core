#!/usr/bin/env python
"""
Created on June 2017
@author: Eduard Vidal & Narcis Palomeras
"""

# ROS imports
import rospy
from cola2_lib import cola2_lib
from cola2_lib import cola2_ros_lib
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from auv_msgs.msg import BodyForceReq
from auv_msgs.msg import BodyVelocityReq
from auv_msgs.msg import WorldWaypointReq
from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from cola2_msgs.srv import Goto, GotoRequest
from cola2_msgs.srv import GetLandmark, GetLandmarkRequest
from cola2_msgs.srv import AddLandmark, AddLandmarkRequest
from cola2_msgs.msg import Map
from cola2_msgs.msg import WorldSectionReqAction, WorldSectionReqGoal
from cola2_msgs.msg import FastraxIt500Gps
from geometry_msgs.msg import PoseWithCovarianceStamped
from cola2_msgs.srv import Action, ActionRequest, ActionResponse

from cola2_lib import NED
import dynamic_reconfigure.client
import actionlib
import math
import tf
import threading

class Star(object):
    """Perform a start calibration maneuver."""

    def __init__(self, name):
        """Init the class."""
        # Name
        self.name = name

        # Services clients
        try:
            rospy.wait_for_service('/cola2_control/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/cola2_control/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')

        # Subscribers
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav,
                         queue_size=1)

        # Create enable start calibration service
        self.test_srv = rospy.Service('/calibration/enable_start',
                                      Action,
                                      self.enable_start)

    def enable_start(self, req):
        """Enable start calibration maneuver."""
        # Perform star maneuver
        if len(req.param) == 5:
            rospy.logwarn("%s: set gps_updates to false in the navigator!!!",
                          self.name)
            x = float(req.param[0])
            y = float(req.param[1])
            depth = float(req.param[2])
            radius = float(req.param[3])
            sides = int(req.param[4])
            text = '{0}: perform start maneuver at ({1}, {2}), depth {3} in a {4}m radius start of {5} sides.'.format(self.name, x, y, depth, radius, sides)
            rospy.loginfo(text)
            self.perform_star_maneuver(x, y, depth, radius, sides)
        else:
            rospy.logwarn('%s: Invalid number of parameters. x, y, depth, radius and num_sides required', self.name)

        return ActionResponse()

    def update_nav(self, nav):
        """Callback for navigation data."""
        self.last_nav = nav

    def transit_to(self, x, y, z, vel=0.5, tol=1.2):
        """Goto to position x, y, z, at velocity vel."""
        goto_request_srv = GotoRequest()
        goto_request_srv.position.x = x
        goto_request_srv.position.y = y
        goto_request_srv.position.z = z
        goto_request_srv.altitude_mode = False
        goto_request_srv.blocking = True
        goto_request_srv.priority = GoalDescriptor.PRIORITY_NORMAL
        goto_request_srv.disable_axis.x = False
        goto_request_srv.disable_axis.y = True
        goto_request_srv.disable_axis.z = False
        goto_request_srv.disable_axis.roll = True
        goto_request_srv.disable_axis.pitch = True
        goto_request_srv.disable_axis.yaw = False
        goto_request_srv.linear_velocity.x = vel
        goto_request_srv.position_tolerance.x = tol
        goto_request_srv.position_tolerance.y = tol
        goto_request_srv.position_tolerance.z = 0.5 * tol
        self.goto_srv(goto_request_srv)
        rospy.sleep(1.0)

    def star_segment(self, x1, y1, x2, y2, depth):
        """ Perform an start segment submerged taking GPS before and after."""
        self.transit_to(x1, y1, 0.0)
        rospy.sleep(30.0)
        self.transit_to(x1, y1, depth)
        rospy.sleep(20.0)
        self.transit_to(x2, y2, depth)
        rospy.sleep(3.0)
        self.transit_to(x2, y2, 0.0)
        rospy.sleep(60.0)

    def perform_star_maneuver(self, x, y, depth, radius, sides):
        """Perform star maneuver."""
        self.transit_to(x, y, 0.0)

        angle_error_sum = 0.0

        angle_delta = 2.0 * math.pi * math.floor(sides / 2) / sides
        angle = 0.0
        x1 = x + radius
        y1 = y
        for i in range(sides):
            angle_new = angle + angle_delta
            x2 = x + math.cos(angle_new) * radius
            y2 = y + math.sin(angle_new) * radius
            self.star_segment(x1, y1, x2, y2, depth)

            desired = math.atan2(y2 - y1, x2 - x1)
            achieved = math.atan2(self.last_nav.position.east - y1,
                                  self.last_nav.position.north - x1)
            error = math.degrees(cola2_lib.wrapAngle(achieved - desired))
            angle_error_sum = angle_error_sum + error

            rospy.loginfo('%s: transect error = %s degrees',
                          self.name, str(error))

            angle = angle_new
            x1 = x2
            y1 = y2

        rospy.loginfo('%s: final mean error = %s degrees',
                      self.name, str(angle_error_sum / float(sides)))

        self.transit_to(x, y, 0.0)
        rospy.loginfo('%s: done', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('star')
        star = Star(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
