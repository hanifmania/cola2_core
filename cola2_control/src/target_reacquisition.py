#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""Target Reacquisition."""

# ROS imports
import rospy
from cola2_lib import cola2_lib
import math
import actionlib
from cola2_lib import cola2_ros_lib
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from auv_msgs.msg import WorldWaypointReq
from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from cola2_msgs.srv import Goto, GotoRequest
from cola2_msgs.srv import Float, FloatResponse
from std_srvs.srv import Empty, EmptyResponse
from cola2_msgs.msg import WorldSectionReqAction, WorldSectionReqGoal

"""
Created on November 2016
@author: Narcis Palomeras
"""


class TargetReacquisition():
    """Create a pattern to reacquire a target from different points of view."""

    def __init__(self, name):
        """Constructor."""
        self.name = name
        self.vehicle_position = [0.0, 0.0]
        self.vehicle_angle = 0.0
        self.altitude_mode = True
        self.track_length = 5.0
        self.number_of_views = 8
        self.last_depth = 0.0
        self.track_speed = 0.2
        self.disable_trajectory = False
        self.get_config()

        # Publishers
        self.pub_marker_array = rospy.Publisher(
            "/target_reacquisition/waypoints", MarkerArray,
            queue_size=1, latch=True)
        self.pub_world_waypoint_req = rospy.Publisher(
            "/cola2_control/world_waypoint_req", WorldWaypointReq,
            queue_size=1)

        # Services clients
        try:
            rospy.wait_for_service('/cola2_control/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/cola2_control/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')

        # Create actionlib client
        self.section_action = actionlib.SimpleActionClient(
                                    'world_section_req',
                                    WorldSectionReqAction)
        rospy.loginfo('%s: waiting for pilot server', self.name)
        self.section_action.wait_for_server()
        rospy.loginfo('%s: pilot server found', self.name)

        # Subscribers
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_position,
                         queue_size=1,
                         )

        # Create service to enable/disable trajectory
        self.srv_enable_target_reacquisition_distance = rospy.Service(
            '/cola2_control/target_reacquisition_distance',
            Float,
            self.target_reacquisition_distance_service)

        self.srv_disable_target_reacquisition_distance = rospy.Service(
            '/cola2_control/disable_target_reacquisition',
            Empty,
            self.disable_target_reacquisition_service)

    def disable_target_reacquisition_service(self, req):
        """Disable target reacquisition trajectory."""
        self.disable_trajectory = True
        return EmptyResponse()

    def target_reacquisition_distance_service(self, req):
        """Enable target reacquisition trajectory giving distance to target."""
        self.disable_trajectory = False
        # Compute initial target position
        x0 = math.cos(self.vehicle_angle)*req.data + self.vehicle_position[0]
        y0 = math.sin(self.vehicle_angle)*req.data + self.vehicle_position[1]
        target_position = [x0, y0]
        waypoints = self.compute_waypoints(target_position, self.vehicle_angle,
                                           self.vehicle_position[2],
                                           self.track_length,
                                           self.number_of_views)
        self.publish_waypoints_marker(waypoints)
        self.perform_trajectory(waypoints)
        return FloatResponse()

    def update_position(self, nav):
        """Keep last vehicle position and angle."""
        if self.altitude_mode:
            self.vehicle_position = [nav.position.north,
                                     nav.position.east,
                                     nav.altitude]
        else:
            self.vehicle_position = [nav.position.north,
                                     nav.position.east,
                                     nav.position.depth]
        self.vehicle_angle = nav.orientation.yaw
        self.last_depth = nav.position.depth

    def compute_waypoints(self, target_position, initial_angle, z,
                          track_length=5.0, number_of_views=8):
        """Compute reacquisition pattern waypoints."""
        angle_inc = math.radians(360.0/number_of_views)
        waypoints = list()
        for i in range(self.number_of_views):
            angle = cola2_lib.wrapAngle(angle_inc*i + initial_angle)
            x = math.cos(angle)*track_length + target_position[0]
            y = math.sin(angle)*track_length + target_position[1]
            waypoints.append([x, y, z])

        sorted_waypoints = list()
        for i in range(0, self.number_of_views/2, 2):
            sorted_waypoints.append(waypoints[i])
            sorted_waypoints.append(waypoints[i + self.number_of_views/2])
            sorted_waypoints.append(waypoints[i + self.number_of_views/2 + 1])
            sorted_waypoints.append(waypoints[i+1])

        print waypoints
        return sorted_waypoints

    def perform_trajectory(self, waypoints):
        """Move the vehicle through the list of waypoitns."""
        for i in range(0, len(waypoints), 2):
            if self.disable_trajectory:
                break
            # Move to first waypoint
            rospy.loginfo("%s: Move to waypoint %d.", self.name, i)
            print waypoints[i]
            self.transit_to(waypoints[i][0],
                            waypoints[i][1],
                            waypoints[i][2], 0.3, 0.75)

            if self.disable_trajectory:
                break
            # Face second waypoints
            rospy.loginfo("%s: Face waypoint %d.", self.name, i+1)
            angle = math.atan2(waypoints[i+1][1] - waypoints[i][1],
                               waypoints[i+1][0] - waypoints[i][0])
            self.facing(angle, self.last_depth, 0.05)

            if self.disable_trajectory:
                break
            # Follow section between waypoint 1 and 2
            rospy.loginfo("%s: Section between wp%d and wp%d.",
                          self.name, i, i+1)
            self.follow_section(waypoints[i], waypoints[i+1],
                                self.track_speed, 0.4)

    def transit_to(self, x, y, z, vel, tol=1.0):
        """Goto to position x, y, z, at velocity vel."""
        req = GotoRequest()
        req.position.x = x
        req.position.y = y
        req.position.z = z
        req.altitude = z
        req.altitude_mode = self.altitude_mode
        req.blocking = True
        req.priority = GoalDescriptor.PRIORITY_NORMAL
        req.disable_axis.x = False
        req.disable_axis.y = True
        req.disable_axis.z = False
        req.disable_axis.roll = True
        req.disable_axis.pitch = True
        req.disable_axis.yaw = False
        req.linear_velocity.x = vel
        req.position_tolerance.x = tol
        req.position_tolerance.y = tol
        req.position_tolerance.z = 0.5 * tol
        self.goto_srv(req)
        rospy.sleep(1.0)

    def facing(self, angle, depth, max_angle_error):
        """Face specific orientation while reaching desired Z."""
        ww_req = WorldWaypointReq()
        ww_req.goal.requester = self.name + "_pose"
        ww_req.goal.priority = GoalDescriptor.PRIORITY_NORMAL
        ww_req.position.depth = depth
        ww_req.orientation.yaw = cola2_lib.wrapAngle(angle)
        ww_req.disable_axis.x = True
        ww_req.disable_axis.y = True
        ww_req.disable_axis.z = False
        ww_req.disable_axis.roll = True
        ww_req.disable_axis.pitch = True
        ww_req.disable_axis.yaw = False

        r = rospy.Rate(10)
        while abs(self.vehicle_angle - angle) > max_angle_error and not self.disable_trajectory:
            ww_req.header.stamp = rospy.Time.now()
            self.pub_world_waypoint_req.publish(ww_req)
            r.sleep()

    def follow_section(self, wp1, wp2, speed, tolerance):
        """Follow section defined by wp1 and wp2."""
        # Prepare section
        goal = WorldSectionReqGoal()
        goal.altitude_mode = self.altitude_mode
        goal.controller_type = goal.LOSCTE
        goal.priority = GoalDescriptor.PRIORITY_NORMAL
        goal.disable_z = False
        goal.timeout = 600
        goal.initial_surge = speed
        goal.final_surge = speed
        goal.tolerance.x = tolerance
        goal.tolerance.y = tolerance
        goal.tolerance.z = 0.5 * tolerance
        goal.initial_position.x = wp1[0]
        goal.initial_position.y = wp1[1]
        goal.initial_position.z = wp1[2]
        goal.final_position.x = wp2[0]
        goal.final_position.y = wp2[1]
        goal.final_position.z = wp2[2]

        # Send section
        self.section_action.send_goal(goal)
        rospy.sleep(2.0)
        self.section_action.wait_for_result()

    def publish_waypoints_marker(self, waypoints):
        """Publish the reacquisition target trajectory as a marker array."""
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = '/world'
        marker.ns = 'target_reacquisition_trajectory'
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for wp in waypoints:
            pt = Point()
            pt.x = wp[0]
            pt.y = wp[1]
            pt.z = wp[2]
            marker.points.append(pt)
        marker_array.markers.append(marker)
        self.pub_marker_array.publish(marker_array)

    def get_config(self):
        """Get configuration parameters from ROS param server."""
        params = {'altitude_mode': 'target_reacquisition/altitude_mode',
                  'track_length': 'target_reacquisition/track_length',
                  'number_of_views': 'target_reacquisition/number_of_views',
                  'track_speed': 'target_reacquisition/track_speed'}

        cola2_ros_lib.getRosParams(self, params, self.name)
        self.number_of_views = int(self.number_of_views)

if __name__ == '__main__':
    try:
        rospy.init_node('target_reacquisition')
        TR = TargetReacquisition(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
