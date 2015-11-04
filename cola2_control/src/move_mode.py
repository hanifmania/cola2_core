#!/usr/bin/env python

"""
    Created by Narcis Palomeras
    revised on Jun 12th, 2013
"""

# ROS imports
import rospy

# Msgs imports
from auv_msgs.msg import NavSts
from auv_msgs.msg import WorldWaypointReq
from auv_msgs.msg import BodyVelocityReq
from auv_msgs.msg import GoalDescriptor

# Python imports
import math
import numpy as np
from cola2_lib import cola2_lib


ERROR = -1
ABSOLUTE_X_Z_YAW = 0
ABSOLUTE_X_Y_Z_YAW = 1
RELATIVE_X_Y_Z_YAW = 2


class MoveMode:
    """ This class has methods used by the pilot """

    def __init__(self, parameters):
        """ Init the class """
        self.parameters = parameters
        self.nav = NavSts()
        self.current_waypoint = {'active': False}


    def check_tolerance(self, b, req):
        """ Check position tolerance """
        # Get current pose
        current = np.zeros(6)
        current[0] = self.nav.position.north
        current[1] = self.nav.position.east
        current[2] = self.nav.position.depth
        current[3] = self.nav.orientation.roll
        current[4] = self.nav.orientation.pitch
        current[5] = self.nav.orientation.yaw

        if req.altitude_mode:
            current[2] = self.nav.altitude

        # Get desired pose
        if req.altitude_mode:
            desired_z = req.altitude
        else:
            desired_z = req.position.depth

        desired = [req.position.north,
                   req.position.east,
                   desired_z,
                   req.orientation.roll,
                   req.orientation.pitch,
                   req.orientation.yaw]

        # Get tolerance
        tolerance = [req.position_tolerance.x,
                     req.position_tolerance.y,
                     req.position_tolerance.z,
                     req.orientation_tolerance.roll,
                     req.orientation_tolerance.pitch,
                     req.orientation_tolerance.yaw]

        # Check if request is achieved
        i = 0
        achieved = True
        while (i < 6 and achieved):
            if not(b[i]):
                if abs(current[i] - desired[i]) > tolerance[i]:
                    achieved = False
            i += 1

        if achieved:
            self.current_waypoint['achieved'] = True

        return achieved


    def legacy_req(self, req, now):
        """ A request is legacy if one of these statements is true:
            - There are no other requests
            - The request is the same than the current one
            - The requester is the same than the current one
            - The new request has more priority
            - 2 seconds has happen from the last request update
            - The last request has been achieved """
        if not(self.current_waypoint['active']):
            # If there is the first request, save request information
            self.current_waypoint['active'] = True
            self.current_waypoint['requester'] = req.goal.requester
            self.current_waypoint['id'] = req.goal.id
            self.current_waypoint['priority'] = req.goal.priority
            self.current_waypoint['stamp_sec'] = now
            self.current_waypoint['achieved'] = False
            return True
        elif (self.current_waypoint['requester'] == req.goal.requester and
              self.current_waypoint['id'] == req.goal.id):
            # Same request
            self.current_waypoint['stamp_sec'] = now
            return True
        elif (self.current_waypoint['requester'] == req.goal.requester and
              self.current_waypoint['id'] != req.goal.id):
            # Different request from the same requester
            self.current_waypoint['active'] = True
            self.current_waypoint['id'] = req.goal.id
            self.current_waypoint['priority'] = req.goal.priority
            self.current_waypoint['stamp_sec'] = now
            self.current_waypoint['achieved'] = False
            return True
        elif ((now > (self.current_waypoint['stamp_sec'] + 2.0)) or
              (req.goal.priority > self.current_waypoint['priority']) or
              self.current_waypoint['achieved']):
            # Last petition is out-dated or
            # the new request has more priority or
            # the current waypoint has been achieved
            self.current_waypoint['active'] = True
            self.current_waypoint['requester'] = req.goal.requester
            self.current_waypoint['id'] = req.goal.id
            self.current_waypoint['priority'] = req.goal.priority
            self.current_waypoint['stamp_sec'] = now
            self.current_waypoint['achieved'] = False
            return True
        else:
            print "/move_mode: ERROR! Pilot has another request to serve"
            return False


    def moveMode_X_Z_YAW(self, req):
        """ Move to a waypoint first orientating the
            vehicle then moving forward. """
        # Compute YAW error
        inc_x = req.position.north - self.nav.position.north
        inc_y = req.position.east - self.nav.position.east
        desired_yaw = math.atan2(inc_y, inc_x)
        yaw_error = cola2_lib.normalizeAngle(desired_yaw -
                                             self.nav.orientation.yaw)

        distance = (inc_x ** 2.0 + inc_y ** 2.0) ** 0.5
        
        rospy.logdebug('%s: /goto: distance: %f', rospy.get_name(), distance)
        rospy.logdebug('%s: /goto: yaw_error: %f', rospy.get_name(), yaw_error)

        # Compute SURGE error
        # If current distance to wp > 1/vel_x_k and angle_error < max_error
        # then move to max vel
        vel_x_k = 0.25
        if abs(yaw_error) < self.parameters.max_angle_error:
            x_vel = np.sqrt(inc_x ** 2 + inc_y ** 2) * vel_x_k
            x_vel = cola2_lib.saturateValueFloat(x_vel, 1.0)
        else:
            x_vel = 0.0

        # Adjust Surge response depending on Yaw error
        x_vel = x_vel * (1.0 - (abs(yaw_error) / self.parameters.max_angle_error))

        # Move from 25% to 100% of surge velocity, not less
        if x_vel > 0 and x_vel < 0.25:
            x_vel = 0.25

        # world_waypoint_req
        desired_z = req.position.depth
        if req.altitude_mode:
            desired_z = req.altitude

        wwr = __create_world_waypoint_request__(
            [0.0, 0.0, desired_z, 0.0, 0.0, desired_yaw],
            [True, True, req.disable_axis.z, True, True, req.disable_axis.yaw],
            req)

        # body velocity req
        bvr = __create_body_velocity_request__(
            [x_vel * self.parameters.max_velocity[0], 0.0, 0.0, 0.0, 0.0, 0.0],
            [req.disable_axis.x, True, True, True, True, True],
            req)

        # Compute feedback
        to_check = [req.disable_axis.x, req.disable_axis.x,
                    req.disable_axis.z, True, True, True]

        success = self.check_tolerance(to_check, req)

        return [success, bvr, wwr]


    def moveMode_LOS(self, previous_req, req):
        """ LOS mode used in girona500 """
        # Compute cross-track error
        alpha = math.atan2(req.position.east - previous_req.position.east,
                           req.position.north - previous_req.position.north)
        # print "Alpha: " + str(alpha)

        e = -(self.nav.position.north - previous_req.position.north) * np.sin(alpha) + \
            (self.nav.position.east - previous_req.position.east) * \
            np.cos(alpha)
        # print "/move_mode: e:" + str(e)
        e = cola2_lib.saturateValueFloat(e, 1.0)

        beta = math.atan2(self.nav.body_velocity.y, self.nav.body_velocity.x)
        # print "/move_mode: beta: " + str(beta)
        # TODO: Saturate beta even more or use sway velocity to compensate it
        beta = cola2_lib.saturateValueFloat(beta, 0.5)

        delta = 8.0
        # print "cross track error: " + str(math.atan(-e/delta))

        desired_yaw = cola2_lib.normalizeAngle(
            alpha + math.atan2(-e, delta) - beta)
        # print '/move_mode: desired_yaw: ', desired_yaw

        # When the vehicle reaches or leaves a way-point the forward velocity
        # follow a trapezoid from min_distance to 0.0 distance respect to the
        # way-point, the forward velocity decreases linearly
        # from 100% of max velocity to min_tant_per_one*100 % of it.
        # TODO: All these vars have to be defined in some other place
        min_tant_per_one = 0.10     # tant per one
        min_distance = 5.0          # meters, this value should be bigger
                                    # than the acceptance sphere
        min_x_v = self.parameters.min_velocity_los[0]
        x_vel = self.parameters.max_velocity[0]
        dist_angle = math.atan2(1 - min_tant_per_one, min_distance)

        # Distance to current way-point
        distance_current = np.sqrt((req.position.north - self.nav.position.north) ** 2 +
                                   (req.position.east - self.nav.position.east) ** 2)

        # Distance to previous way-point
        distace_previous = np.sqrt((previous_req.position.north - self.nav.position.north) ** 2 +
                                   (previous_req.position.east - self.nav.position.east) ** 2)

        # Take the smaller one
        distance = min(distance_current, distace_previous)

        if distance < min_distance:
            tant_per_one = min_tant_per_one + np.tan(dist_angle) * distance
            x_vel *= tant_per_one
            if x_vel < min_x_v:
                x_vel = min_x_v

        # print 'x_vel: ', x_vel

        # Current and desired z
        desired_z = req.position.depth
        current_z = self.nav.position.depth
        if req.altitude_mode:
            desired_z = req.altitude
            current_z = self.nav.altitude

        # Compute feedback
        to_check = [False, False, req.disable_axis.z, True, True, True]
        success = self.check_tolerance(to_check, req)

        # Check if the limit line has been crossed. Where the limit line is
        # the line perpendicular to the desired path that pass through the
        # current way-point
        inc_y = req.position.east - previous_req.position.east
        if inc_y != 0.0:
            m_l = -(req.position.north - previous_req.position.north) / inc_y
        else:
            m_l = 999999.9
        c_l = -m_l * req.position.north + req.position.east
        current_d = (m_l * self.nav.position.north -
                     self.nav.position.east + c_l) / np.sqrt(m_l ** 2 + 1)
        signe = (m_l * previous_req.position.north -
                 previous_req.position.east + c_l) / np.sqrt(m_l ** 2 + 1)
        if signe * current_d < 0.0:
            print '/move_mode: beyond the waypoint!'
            # The vehicle has crossed the line
            if req.disable_axis.z:
            # If z is uncontrolled
                success = True
            elif abs(current_z - desired_z) < req.position_tolerance.z:
            # If Z is ok, success = True.
                success = True
            else:
            # Otherwise X and Yaw = 0.0 and success = false
            # wait for z to be True
                desired_yaw = self.nav.orientation.yaw
                x_vel = 0.0
                success = False

        # There was a problem when the 2 consecutive points in the LOS
        # are the same this particular case can make the vehicle's drift
        # TODO: COMPTE! Quan esperem nomes la Z el robot no s'hauria de moure en
        # yaw no?
        if (req.position.east == previous_req.position.east and
                req.position.north == previous_req.position.north):
            print '/move_mode: same waypoint!'
            x_vel = 0.0
            desired_yaw = self.nav.orientation.yaw
            if abs(current_z - desired_z) < req.position_tolerance.z:
                success = True

        # Compute world_waypoint_req
        wwr = __create_world_waypoint_request__(
            [0.0, 0.0, desired_z, 0.0, 0.0, desired_yaw],
            [True, True, req.disable_axis.z, True, True, False],
            req)

        # Compute body_velocity_req
        bvr = __create_body_velocity_request__(
            [x_vel, 0.0, 0.0, 0.0, 0.0, 0.0],
            [False, True, True, True, True, True],
            req)

        return [success, bvr, wwr]


    def moveMode_keep_pose_sparus2(self, req):
        """ Keep pose mode """
        #print("/move_mode: (sparus2 keep_pose)")

        # Embedded configuration
        kp = 0.1
        radius = 1.0
        min_surge = -0.1
        max_surge = 0.4
        safety_distance = 50.0

        # Compute distance to the waypoint
        robot_distance_2D = ((req.position.north - self.nav.position.north) ** 2.0 + (req.position.east - self.nav.position.east) ** 2.0) ** 0.5
        #print("/move_mode: (sparus2 keep_pose) robot distance 2D: " + str(robot_distance_2D))

        # Heave
        if req.altitude_mode:
            desired_heave = 0.0
        else:
            desired_heave = req.position.depth

        # Yaw
        desired_yaw = math.atan2(req.position.east - self.nav.position.east, req.position.north - self.nav.position.north)

        # Surge
        if abs(cola2_lib.wrapAngle(desired_yaw - self.nav.orientation.yaw)) > 0.5:
            desired_surge = 0.0
        else:
            error = radius - robot_distance_2D
            desired_surge = -kp * error
            if desired_surge > max_surge:
                desired_surge = max_surge
            if desired_surge < min_surge:
                desired_surge = min_surge

        # Safety
        allow_altitude = True
        if robot_distance_2D > safety_distance:
            #print("/move_mode: (sparus2 keep_pose) too far from the waypoint! This node is not for navigating!")
            desired_surge = 0.0
            desired_yaw = 0.0
            desired_heave = 0.0
            allow_altitude = False

        # Compute world_waypoint_req
        if desired_heave < 0.20 and self.nav.position.depth < 0.5:  # No heave in surface
            wwr = __create_world_waypoint_request__(
                [0.0, 0.0, 0.0, 0.0, 0.0, desired_yaw],
                [True, True, True, True, True, False],
                req)
        else:
            wwr = __create_world_waypoint_request__(
                [0.0, 0.0, desired_heave, 0.0, 0.0, desired_yaw],
                [True, True, False, True, True, False],
                req)

        # Check if altutude_mode is activated and if it is safe to use altitude
        if req.altitude_mode and allow_altitude:
            wwr.altitude_mode = True
            wwr.altitude = req.altitude
            wwr.disable_axis.z = False
        else:
            wwr.altitude_mode = False
            wwr.altitude = 1000.0

        # Compute body_velocity_req
        bvr = __create_body_velocity_request__(
            [desired_surge, 0.0, 0.0, 0.0, 0.0, 0.0],
            [False, True, True, True, True, True],
            req)

        # Print some info about desired velocities and positions
        #print("/move_mode: (sparus2 keep_pose) desired surge: " + str(desired_surge))
        #if wwr.altitude_mode:
            #print("/move_mode: (sparus2 keep_pose) desired altitude: " + str(req.altitude))
        #else:
            #print("/move_mode: (sparus2 keep_pose) desired heave: " + str(desired_heave))
        #print("/move_mode: (sparus2 keep_pose) desired yaw: " + str(desired_yaw))

        # Check if success
        success = False
        if robot_distance_2D < radius:
            success = True

        return [success, bvr, wwr]


    def moveMode_LOS_sparus2(self, previous_req, req):
        """ LOS mode used in sparus2 """
        rospy.logdebug('%s: (sparus2 LOS)', rospy.get_name())
        rospy.logdebug('%s: (sparus2 LOS) requested pose. North: %s, East: %s', rospy.get_name(), str(req.position.north), str(req.position.east))
        rospy.logdebug('%s: (sparus2 LOS) current pose. North: %s, East: %s', rospy.get_name(), str(self.nav.position.north), str(self.nav.position.east))
        rospy.logdebug('%s: (sparus2 LOS) desired depth: %f', rospy.get_name(), req.position.depth)

        # Check acceptance radius from config file. Smaller than 0.5 meters is not allowed
        acceptance_radius = 0.5
        if self.parameters.sparus_los.acceptance_radius > 0.5:
            acceptance_radius = self.parameters.sparus_los.acceptance_radius

        # Compute some distances
        robot_distance_2D = ((req.position.north - self.nav.position.north) ** 2.0 + (
            req.position.east - self.nav.position.east) ** 2.0) ** 0.5
        robot_distance_depth = abs(req.position.depth - self.nav.position.depth)
        robot_distance_altitude = abs(req.altitude - self.nav.altitude)
        robot_previous_distance_2D = ((previous_req.position.north - self.nav.position.north) ** 2.0 + (
            previous_req.position.east - self.nav.position.east) ** 2.0) ** 0.5
        waypoints_distance_2D = ((previous_req.position.north - req.position.north) ** 2.0 + (
            previous_req.position.east - req.position.east) ** 2.0) ** 0.5

        # Print some info about distances
        rospy.logdebug('%s: (sparus2 LOS) robot distance 2D: %f', rospy.get_name(), robot_distance_2D)
        if req.altitude_mode:
            rospy.logdebug('%s: (sparus2 LOS) robot distance altitude: %f', rospy.get_name(), robot_distance_altitude)
        else:
            rospy.logdebug('%s: (sparus2 LOS) robot distance depth: %f', rospy.get_name(), robot_distance_depth)

        # Select movement
        if waypoints_distance_2D > 0.1 and robot_distance_2D > acceptance_radius - 0.35:
            # Angle of path
            alpha_k = math.atan2(req.position.east - previous_req.position.east, req.position.north - previous_req.position.north)

            # Along-track distance (s) and cross-track error (e) (rotation)
            los_s = (self.nav.position.north - previous_req.position.north) * np.cos(
                alpha_k) + (self.nav.position.east - previous_req.position.east) * np.sin(alpha_k)
            los_e = - (self.nav.position.north - previous_req.position.north) * np.sin(
                alpha_k) + (self.nav.position.east - previous_req.position.east) * np.cos(alpha_k)

            # Orthogonal projection
            Xproj = previous_req.position.north + los_s * np.cos(alpha_k)
            Yproj = previous_req.position.east + los_s * np.sin(alpha_k)

            # Compute lookahead distance (los_delta). It is always positive
            los_delta = 0.0
            if self.parameters.sparus_los.los_radius > abs(los_e):
                los_delta = (self.parameters.sparus_los.los_radius ** 2.0 - los_e ** 2.0) ** 0.5

            if self.parameters.sparus_los.los_radius > robot_distance_2D:
                los_delta = ((req.position.north - Xproj) ** 2.0 + (req.position.east - Yproj) ** 2.0) ** 0.5

            # Compute LOS vector
            if los_s < waypoints_distance_2D:
                LOSX = Xproj + los_delta * np.cos(alpha_k) - self.nav.position.north
                LOSY = Yproj + los_delta * np.sin(alpha_k) - self.nav.position.east

            else:
                LOSX = Xproj - los_delta * np.cos(alpha_k) - self.nav.position.north
                LOSY = Yproj - los_delta * np.sin(alpha_k) - self.nav.position.east

            # Compute surge
            if self.parameters.sparus_los.los_radius > robot_distance_2D or self.parameters.sparus_los.los_radius > robot_previous_distance_2D:
                desired_surge = self.parameters.sparus_los.low_surge
            else:
                desired_surge = self.parameters.sparus_los.high_surge

            # Compute yaw
            beta_low_speed = 0.25
            beta_high_speed = 0.5
            if self.parameters.sparus_los.sway_correction:
                if self.nav.body_velocity.x > beta_high_speed:
                    beta_factor = 1.0
                elif self.nav.body_velocity.x < beta_low_speed:
                    beta_factor = 0.0
                else:
                    beta_factor = (self.nav.body_velocity.x - beta_low_speed) / (beta_high_speed - beta_low_speed)
            else:
                beta_factor = 0.0
            beta = math.atan2(self.nav.body_velocity.y, self.nav.body_velocity.x)
            desired_yaw = cola2_lib.wrapAngle(math.atan2(LOSY, LOSX) - beta_factor * beta)

            # Compute heave
            if req.altitude_mode:
                desired_heave = 0.0
            else:
                if self.parameters.sparus_los.heave_mode_in_3D and not req.altitude_mode and not previous_req.altitude_mode:
                    if los_s < 0.0:
                        desired_heave = previous_req.position.depth
                    elif los_s > waypoints_distance_2D:
                        desired_heave = req.position.depth
                    else:
                        desired_heave = previous_req.position.depth + (req.position.depth - previous_req.position.depth) * (los_s / waypoints_distance_2D)
                else:
                    desired_heave = req.position.depth

        else:
            # Compute surge
            if robot_distance_2D < acceptance_radius:
                desired_surge = 0.0
            else:
                desired_surge = self.parameters.sparus_los.low_surge

            # Compute yaw
            desired_yaw = math.atan2(req.position.east - self.nav.position.east, req.position.north - self.nav.position.north)

            # Compute heave
            if req.altitude_mode:
                desired_heave = 0.0
            else:
                desired_heave = req.position.depth

        # Some safety
        allow_altitude = True
        path_center_north = previous_req.position.north + 0.5 * (req.position.north - previous_req.position.north)
        path_center_east = previous_req.position.east + 0.5 * (req.position.east - previous_req.position.east)
        path_center_to_robot_distance_2D = ((path_center_north - self.nav.position.north) ** 2.0 + (path_center_east - self.nav.position.east) ** 2.0) ** 0.5
        if path_center_to_robot_distance_2D > waypoints_distance_2D + 100.0:
            print("/move_mode: (sparus2 LOS) too far from the center of the path!")
            desired_surge = 0.0
            desired_yaw = 0.0
            desired_heave = 0.0
            allow_altitude = False
        if waypoints_distance_2D > self.parameters.sparus_los.safety_max_distance_btw_waypoints:
            print("/move_mode: (sparus2 LOS) distance between waypoints too large!")
            desired_surge = 0.0
            desired_yaw = 0.0
            desired_heave = 0.0
            allow_altitude = False

        # Compute world_waypoint_req
        if desired_heave < 0.20 and self.nav.position.depth < 0.3:  # No heave in surface
            wwr = __create_world_waypoint_request__(
                [0.0, 0.0, 0.0, 0.0, 0.0, desired_yaw],
                [True, True, True, True, True, False],
                req)
        else:
            wwr = __create_world_waypoint_request__(
                [0.0, 0.0, desired_heave, 0.0, 0.0, desired_yaw],
                [True, True, False, True, True, False],
                req)

        # Check if altutude_mode is activated and if it is safe to use altitude
        if req.altitude_mode and allow_altitude:
            wwr.altitude_mode = True
            wwr.altitude = req.altitude
            wwr.disable_axis.z = False
        else:
            wwr.altitude_mode = False
            wwr.altitude = 1000.0

        # Compute body_velocity_req
        bvr = __create_body_velocity_request__(
            [desired_surge, 0.0, 0.0, 0.0, 0.0, 0.0],
            [False, True, True, True, True, True],
            req)

        # Print some info about desired velocities and positions
        rospy.logdebug('%s: (sparus2 LOS) desired surge: %f', rospy.get_name(), desired_surge)
        if wwr.altitude_mode:
            rospy.logdebug('%s: (sparus2 LOS) desired altitude: %f', rospy.get_name(), req.altitude)
        else:
            rospy.logdebug('%s: (sparus2 LOS) desired heave: %f', rospy.get_name(), desired_heave)
        rospy.logdebug('%s: (sparus2 LOS) desired yaw: %f', rospy.get_name(), desired_yaw)

        # Check if success
        success = False
        if robot_distance_2D < acceptance_radius:
            if req.altitude_mode:
                if robot_distance_altitude < 0.5 * acceptance_radius:
                    success = True
            else:
                if robot_distance_depth < 0.2 * acceptance_radius:
                    success = True

        return [success, bvr, wwr]


    def moveMode_LOS_sparus2_old(self, previous_req, req):  # I will remove this soon! Its just as a backup
        """ LOS mode used in sparus2 """
        print "/move_mode: (sparus2 LOS) "
        print "/move_mode: (sparus2 LOS) requested pose: North", req.position.north, " East:", req.position.east
        print "/move_mode: (sparus2 LOS) current pose: North", self.nav.position.north, " East:", self.nav.position.east

        # Compute distance from robot to waypoint in 2D and 3D
        robot_distance_2D = ((req.position.north - self.nav.position.north) ** 2.0 + (
            req.position.east - self.nav.position.east) ** 2.0) ** 0.5
        robot_previous_distance_2D = ((previous_req.position.north - self.nav.position.north) ** 2.0 + (
            previous_req.position.east - self.nav.position.east) ** 2.0) ** 0.5
        robot_distance_3D = ((req.position.north - self.nav.position.north) ** 2.0 + (
            req.position.east - self.nav.position.east) ** 2.0 + (req.position.depth - self.nav.position.depth) ** 2.0) ** 0.5
        print "/move_mode: (sparus2 LOS) robot distance 2D:", robot_distance_2D
        print "/move_mode: (sparus2 LOS) robot distance 3D:", robot_distance_3D

        # Compute distance from previous waypoint to waypoint in 2D
        waypoints_distance_2D = ((previous_req.position.north - req.position.north) ** 2.0 + (
            previous_req.position.east - req.position.east) ** 2.0) ** 0.5

        # Select movement case:   1 -> Go near the waypoint
        #                         2 -> Go up or down
        #                         3 -> Waypoint location: one above the other
        #                         4 -> Keep pose

        success = False
        if (robot_distance_2D < self.parameters.sparus_los.acceptance_radius):
            if (robot_distance_3D < self.parameters.sparus_los.acceptance_radius):
                success = True
                print "/move_mode: (sparus2 LOS) success on (" + str(req.position.north) + ", " + str(req.position.east) + ")"
                movement_case = 4  # TODO: this is not good. The vehicle wants to stop for one iteration
            else:
                movement_case = 2
        else:
            if (waypoints_distance_2D < self.parameters.sparus_los.movement_3_limit):
                movement_case = 3
            else:
                movement_case = 1

        # Compute movement
        if (movement_case == 1):  # Go near the waypoint (using LOS)
            # Angle of path
            alpha_K = math.atan2(
                req.position.east - previous_req.position.east, req.position.north - previous_req.position.north)

            # Along-track distance (s) and cross-track error (e) (rotation)
            los_s = (self.nav.position.north - previous_req.position.north) * np.cos(
                alpha_K) + (self.nav.position.east - previous_req.position.east) * np.sin(alpha_K)
            los_e = - (self.nav.position.north - previous_req.position.north) * np.sin(
                alpha_K) + (self.nav.position.east - previous_req.position.east) * np.cos(alpha_K)

            # Orthogonal projection
            Xproj = previous_req.position.north + los_s * np.cos(alpha_K)
            Yproj = previous_req.position.east + los_s * np.sin(alpha_K)

            # Compute lookahead distance (los_delta). It is always positive
            los_delta = 0.0
            if (self.parameters.sparus_los.los_radius > abs(los_e)):
                los_delta = (self.parameters.sparus_los.los_radius ** 2.0 - los_e ** 2.0) ** 0.5

            if (self.parameters.sparus_los.los_radius > robot_distance_2D):
                los_delta = ((req.position.north - Xproj) ** 2.0 + (
                    req.position.east - Yproj) ** 2.0) ** 0.5

            # Compute LOS vector
            if (los_s < waypoints_distance_2D):
                LOSX = Xproj + los_delta * np.cos(alpha_K) - self.nav.position.north
                LOSY = Yproj + los_delta * np.sin(alpha_K) - self.nav.position.east

            else:
                LOSX = Xproj - los_delta * np.cos(alpha_K) - self.nav.position.north
                LOSY = Yproj - los_delta * np.sin(alpha_K) - self.nav.position.east

            # Compute surge
            if ((self.parameters.sparus_los.los_radius > robot_distance_2D) or (self.parameters.sparus_los.los_radius > robot_previous_distance_2D)):
                desired_surge = self.parameters.sparus_los.low_surge
            else:
                desired_surge = self.parameters.sparus_los.high_surge

            # Compute yaw
            beta_low_speed = 0.25
            beta_high_speed = 0.5
            if self.parameters.sparus_los.sway_correction:
                if (self.nav.body_velocity.x > beta_high_speed):
                    beta_factor = 1.0
                elif (self.nav.body_velocity.x < beta_low_speed):
                    beta_factor = 0.0
                else:
                    beta_factor = (self.nav.body_velocity.x - beta_low_speed) / (beta_high_speed - beta_low_speed)
            else:
                beta_factor = 0.0
            beta = math.atan2(self.nav.body_velocity.y, self.nav.body_velocity.x)
            desired_yaw = cola2_lib.wrapAngle(math.atan2(LOSY, LOSX) - beta_factor * beta)

            # Compute heave
            if req.altitude_mode:
                desired_heave = self.nav.position.depth + self.nav.altitude - req.altitude
            else:
                if not self.parameters.sparus_los.heave_mode_in_3D:
                    desired_heave = req.position.depth
                else:
                    if (los_s < 0.0):
                        desired_heave = previous_req.position.depth
                    elif (los_s > waypoints_distance_2D):
                        desired_heave = req.position.depth  # Do not combine altitude and 3D
                    else:
                        desired_heave = previous_req.position.depth + \
                            (req.position.depth - previous_req.position.depth) * (
                                los_s / waypoints_distance_2D)

        elif (movement_case == 2):  # Go up or down
            desired_surge = 0.0

            desired_yaw = math.atan2(
                req.position.east - self.nav.position.east, req.position.north - self.nav.position.north)

            if req.altitude_mode:
                desired_heave = self.nav.position.depth + self.nav.altitude - req.altitude
            else:
                desired_heave = req.position.depth

        elif (movement_case == 3):  # Waypoint locaton: one above the other
            if (robot_distance_2D < self.parameters.sparus_los.acceptance_radius):
                desired_surge = 0.0
            else:
                desired_surge = self.parameters.sparus_los.low_surge

            desired_yaw = math.atan2(
                req.position.east - self.nav.position.east, req.position.north - self.nav.position.north)

            if req.altitude_mode:
                desired_heave = self.nav.position.depth + self.nav.altitude - req.altitude
            else:
                desired_heave = req.position.depth

        elif (movement_case == 4):  # Keep pose
            desired_surge = 0.0
            desired_yaw = self.nav.orientation.yaw  # This can't keep pose for a long time
            if req.altitude_mode:
                desired_heave = self.nav.position.depth + self.nav.altitude - req.altitude
            else:
                desired_heave = req.position.depth

        else:  # Surface
            desired_surge = 0.0
            desired_yaw = 0.0
            desired_heave = 0.0

        # Safety
        if (self.nav.altitude < self.parameters.sparus_los.safety_min_altitude):
            print "/move_mode: (sparus2 LOS) too close to the bottom! Going up!"
            desired_heave = 0.0

        pathCenterNorth = previous_req.position.north + 0.5 * \
            (req.position.north - previous_req.position.north)
        pathCenterEast = previous_req.position.east + 0.5 * \
            (req.position.east - previous_req.position.east)
        pathCenterTorobot_distance_2D = (
            (pathCenterNorth - self.nav.position.north) ** 2 + (pathCenterEast - self.nav.position.east) ** 2) ** 0.5
        if (pathCenterTorobot_distance_2D > waypoints_distance_2D + 50.0):
            print "/move_mode: (sparus2 LOS) too far from the center of the path! Going up!"
            desired_surge = 0.0
            desired_yaw = 0.0
            desired_heave = 0.0

        if (waypoints_distance_2D > self.parameters.sparus_los.safety_max_distance_btw_waypoints):
            print "/move_mode: (sparus2 LOS) distance between waypoints too large!"
            desired_surge = 0.0
            desired_yaw = 0.0
            desired_heave = 0.0

        print "/move_mode: (sparus2 LOS) desired surge:", desired_surge
        print "/move_mode: (sparus2 LOS) desired heave:", desired_heave
        print "/move_mode: (sparus2 LOS) desired yaw:", desired_yaw
        # Compute world_waypoint_req
        if ((desired_heave < 0.01) and (self.nav.position.depth < 0.5)):  # No heave in surface
            wwr = __create_world_waypoint_request__(
                [0.0, 0.0, 0.0, 0.0, 0.0, desired_yaw],
                [True, True, True, True, True, False],
                req)
        else:
            wwr = __create_world_waypoint_request__(
                [0.0, 0.0, desired_heave, 0.0, 0.0, desired_yaw],
                [True, True, False, True, True, False],
                req)

        wwr.altitude_mode = False  # Important! Output is always depth. TODO: this has to be changed

        # Compute body_velocity_req
        bvr = __create_body_velocity_request__(
            [desired_surge, 0.0, 0.0, 0.0, 0.0, 0.0],
            [False, True, True, True, True, True],
            req)

        return [success, bvr, wwr]


    def moveMode_X_Y_Z_YAW(self, req):
        """ Mode with surge, sway, heave and yaw """
        # world pose request
        desired_z = req.position.depth
        if req.altitude_mode:
            desired_z = req.altitude

        wwr = __create_world_waypoint_request__(
            [req.position.north, req.position.east, desired_z,
             req.orientation.roll, req.orientation.pitch, req.orientation.yaw],
            [req.disable_axis.x, req.disable_axis.y, req.disable_axis.z,
             req.disable_axis.roll, req.disable_axis.pitch, req.disable_axis.yaw],
            req)

        bvr = __create_body_velocity_request__(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [True, True, True, True, True, True],
            req)

        to_check = [False, False, req.disable_axis.z,
                    True, True, req.disable_axis.yaw]
        success = self.check_tolerance(to_check, req)

        return [success, bvr, wwr]

    def update_nav(self, nav_data):
        self.nav = nav_data


def __create_world_waypoint_request__(values, disable_axis, req):
    """ Helper function to create a world_waypoint_req message """
    wwr = WorldWaypointReq()
    wwr.goal = GoalDescriptor(req.goal.requester,
                              req.goal.id,
                              req.goal.priority)
    wwr.altitude_mode = req.altitude_mode
    wwr.position.north = values[0]
    wwr.position.east = values[1]
    wwr.position.depth = values[2]
    wwr.altitude = values[2]
    wwr.orientation.roll = values[3]
    wwr.orientation.pitch = values[4]
    wwr.orientation.yaw = values[5]
    wwr.disable_axis.x = disable_axis[0]
    wwr.disable_axis.y = disable_axis[1]
    wwr.disable_axis.z = disable_axis[2]
    wwr.disable_axis.roll = disable_axis[3]
    wwr.disable_axis.pitch = disable_axis[4]
    wwr.disable_axis.yaw = disable_axis[5]

    return wwr


def __create_body_velocity_request__(values, disable_axis, req):
    """ Helper function to create a body_velocity_req message """
    bvr = BodyVelocityReq()
    bvr.goal = GoalDescriptor(req.goal.requester,
                              req.goal.id,
                              req.goal.priority)
    bvr.twist.linear.x = values[0]
    bvr.twist.linear.y = values[1]
    bvr.twist.linear.z = values[2]
    bvr.twist.angular.x = values[3]
    bvr.twist.angular.y = values[4]
    bvr.twist.angular.z = values[5]
    bvr.disable_axis.x = disable_axis[0]
    bvr.disable_axis.y = disable_axis[1]
    bvr.disable_axis.z = disable_axis[2]
    bvr.disable_axis.roll = disable_axis[3]
    bvr.disable_axis.pitch = disable_axis[4]
    bvr.disable_axis.yaw = disable_axis[5]

    return bvr


def __check_absolute_navigation_mode__(b):
    """ Function to check absolute navitaion mode """
    if(not(b.x) and b.y and b.roll and b.pitch and not(b.yaw)):
        return ABSOLUTE_X_Z_YAW
    elif (b.x and b.y and not(b.z) and b.yaw):
        # Only controls Z
        return ABSOLUTE_X_Z_YAW
    elif(not(b.x) and not(b.y) and b.roll and b.pitch):
        return ABSOLUTE_X_Y_Z_YAW
    else:
        return ERROR
