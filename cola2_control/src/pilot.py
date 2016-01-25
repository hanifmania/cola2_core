#!/usr/bin/env python
"""@@The pilot node, directed by the captain, publishes position and velocity
setpoints to the position and velocity controllers. Pilot uses move_mode.py to
compute requests.@@"""

# ROS imports
import rospy
import actionlib

# Msgs imports
from auv_msgs.msg import NavSts, BodyVelocityReq, WorldWaypointReq
from cola2_msgs.msg import WorldWaypointReqGoal, WorldWaypointReqAction
from cola2_msgs.msg import WorldWaypointReqFeedback, WorldWaypointReqResult
from visualization_msgs.msg import Marker

# services import
from cola2_msgs.srv import SetVelocitiesLOS

# Python imports
from cola2_lib import cola2_ros_lib
import move_mode
import copy


class Pilot:
    """ This node handles requests (actions) from captain """

    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Rate
        self.rate = 10  # TODO: this should be in the config file

        # Define nav_sts message
        self.nav_sts = NavSts()

        # Define current goal message
        self.current_goal = WorldWaypointReqGoal()

        # Get config
        self.get_config()

        # Define a move_mode class
        self.move_mode = move_mode.MoveMode(self.parameters)

        # Initialize previous req (necessary for LOS)
        self.previous_goal = WorldWaypointReqGoal()
        self.previous_goal.goal.requester = self.name

        # Create publisher
        self.pub_body_velocity_req = rospy.Publisher(
                                        "/cola2_control/body_velocity_req",
                                        BodyVelocityReq,
                                        queue_size=2)

        self.pub_world_waypoint_req = rospy.Publisher(
                                        "/cola2_control/world_waypoint_req",
                                        WorldWaypointReq,
                                        queue_size=2)

        self.pub_marker = rospy.Publisher('/cola2_control/waypoint_marker',
                                          Marker,
                                          queue_size=2)

        # Create subscriber
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav_sts,
                         queue_size = 1)
        
        #Create services
        self.set_vel_los_srv = rospy.Service('cola2_control/set_vel_los',
                                             SetVelocitiesLOS,
                                             self.set_vel_los)

        # Create actionlib server (for services that take a long time to exec.)
        # This is the link with the captain
        self.absolute_action = actionlib.SimpleActionServer(
                                            'absolute_movement',
                                            WorldWaypointReqAction,
                                            self.action_absolute, False)
        self.absolute_action.start()

        # Show message
        rospy.loginfo("%s: initialized", self.name)

    def set_vel_los(self, req):
        """ set velocities to LOS """
        self.move_mode.parameters.sparus_los.low_surge = req.min_vel
        self.move_mode.parameters.sparus_los.high_surge = req.max_vel
        return True

    def action_absolute(self, goal):
        """ Absolute action. Input of the actionlib server """
        # Init flags and time
        success = False
        preempted = False
        timeout = False
        init_time = rospy.Time().now()

        # Set the current goal as the recieved goal
        self.current_goal = goal

        # Show message with the recieved goal
        #rospy.loginfo('%s: RECEIVED ABSOLUTE ACTION: \n%s', self.name, goal)

        # CREATE & PUBLISH MARKER
        self.create_marker(goal)

        # If is the first waypoint to achieve, initialize the previous
        # one with current position
        if goal.goal.id == 0:
            self.previous_goal.position.north = copy.copy(
                                                self.nav_sts.position.north)
            self.previous_goal.position.east = copy.copy(
                                                self.nav_sts.position.east)
            # print '\n\n\n\n init waypoint 0:\n', self.previous_goal.position, '\n\n\n\n'

        # If the goal can be processed
        if self.move_mode.legacy_req(goal, rospy.Time.now().to_sec()):
            # Check, from disabled axis, which mode has to be applied
            mode = move_mode.__check_absolute_navigation_mode__(goal.disable_axis)

            if mode == move_mode.ABSOLUTE_X_Z_YAW:
                # Show message with selected mode
                rospy.loginfo('%s: MovementMode.ABSOLUTE_X_Z_YAW', self.name)

                # Define a rate
                r = rospy.Rate(self.rate)

                # Main while of this mode. This really processes the goal
                while not success and not preempted and not timeout:
                    # Call Move X Z YAW Method. There are two available modes:
                    # moveMode_X_Z_YAW (default) or moveMode_LOS
                    if goal.mode == 'los':
                        [success, bvr, wwr] = self.move_mode.moveMode_LOS(
                            self.previous_goal, goal)
                    elif goal.mode == 'sparus_los':
                        [success, bvr, wwr] = self.move_mode.moveMode_LOS_sparus2(
                            self.previous_goal, goal)
                    elif goal.mode == 'sparus_keep_pose':
                        [success, bvr, wwr] = self.move_mode.moveMode_keep_pose_sparus2(goal)
                        success = False
                    else:
                        [success, bvr, wwr] = self.move_mode.moveMode_X_Z_YAW(goal)

                    # Publish setpoints
                    self.pub(bvr, wwr)

                    # Print some info
                    #print bvr.twist
                    #print wwr.position
                    #print wwr.altitude
                    #print wwr.orientation

                    # If preempted
                    if self.absolute_action.is_preempt_requested():
                        rospy.loginfo('%s: preempted ABSOLUTE_X_Z_YAW', self.name)
                        self.absolute_action.set_preempted()
                        preempted = True
                    else:
                        # Create Feedback response
                        self.pub_absolute_feedback(goal)
                        # Sleep
                        r.sleep()

                    # Compute Timeout
                    if goal.timeout > 0:
                        if (rospy.Time().now() - init_time).to_sec() > goal.timeout:
                            rospy.logerr('%s: waypoint timeout!', self.name)
                            timeout = True

                # TODO: Differentitate success and timeout
                if success or timeout:
                    # Save the last waypoint
                    self.previous_goal = copy.copy(goal)
                    self.delete_marker(goal)
                    self.pub_absolute_result(goal)

            elif mode == move_mode.ABSOLUTE_X_Y_Z_YAW:
                # Show message with selected mode
                rospy.loginfo('%s: MovementMode.ABSOLUTE_X_Y_Z_YAW', self.name)

                # Define a rate
                r = rospy.Rate(self.rate)

                # Main while of this mode. This really processes the goal
                while not success and not preempted:
                    # Call Move X Y Z YAW Method
                    [success, bvr, wwr] = self.move_mode.moveMode_X_Y_Z_YAW(goal)
                    if goal.mode == 'neverending':
                        success = False

                    # Publish body_velocity_req
                    self.pub(bvr, wwr)

                    # If preempted
                    if self.absolute_action.is_preempt_requested():
                        rospy.loginfo('%s: preempted ABSOLUTE_X_Y_Z_YAW',
                                      self.name)
                        self.absolute_action.set_preempted()
                        preempted = True
                    else:
                        # Create Feedback response
                        self.pub_absolute_feedback(goal)
                        # Sleep
                        r.sleep()

                if success:
                    # Save the last waypoint
                    self.previous_goal = goal
                    self.delete_marker(goal)
                    self.pub_absolute_result(goal)
            else:
                rospy.logwarn("%s: invalid movement mode", self.name)
        else:
            rospy.logwarn("%s: ilegal world waypoint request", self.name)


    def create_marker(self, goal):
        """ Create marker and publish it """
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time().now()
        marker.ns = "waypoint"
        marker.id = goal.goal.id
        marker.type = 2  # SPHERE
        marker.action = 0  # Add/Modify an object
        marker.pose.position.x = goal.position.north
        marker.pose.position.y = goal.position.east
        marker.pose.position.z = goal.position.depth
        marker.scale.x = goal.position_tolerance.x * 2
        marker.scale.y = goal.position_tolerance.y * 2
        marker.scale.z = goal.position_tolerance.z
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime = rospy.Duration(600.0)
        marker.frame_locked = False
        self.pub_marker.publish(marker)


    def delete_marker(self, goal):
        """ Delete marker method """
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time().now()
        marker.ns = "waypoint"
        marker.id = goal.goal.id
        marker.type = 2  # SPHERE
        marker.action = 2  # Deletes an object
        self.pub_marker.publish(marker)


    def pub(self, bvr, wwr):
        """ Publish pose and vel requests """
        # Helper function used to publish both messages
        bvr.header.stamp = rospy.Time().now()
        wwr.header.stamp = rospy.Time().now()
        wwr.goal.requester = self.name + '_position'
        bvr.goal.requester = self.name + '_velocity'

        #rospy.loginfo("%s: send body_velocity_req & world_waypoint_req:\n%s\n%s", self.name, wwr, bvr)

        self.pub_body_velocity_req.publish(bvr)
        self.pub_world_waypoint_req.publish(wwr)


    def update_nav_sts(self, nav_sts):
        """ Update nav sts """
        self.move_mode.update_nav(nav_sts)
        self.nav_sts = nav_sts


    def pub_absolute_feedback(self, goal):
        """ Publish absolute feedback """
        # Publish feedback back to captain
        feedback = WorldWaypointReqFeedback()
        feedback.altitude_mode = goal.altitude_mode
        feedback.position.north = self.nav_sts.position.north
        feedback.position.east = self.nav_sts.position.east
        feedback.position.depth = self.nav_sts.position.depth
        feedback.altitude = self.nav_sts.altitude
        feedback.orientation.roll = self.nav_sts.orientation.roll
        feedback.orientation.pitch = self.nav_sts.orientation.pitch
        feedback.orientation.yaw = self.nav_sts.orientation.yaw
        self.absolute_action.publish_feedback(feedback)


    def pub_absolute_result(self, goal):
        """ Publish absolute result """
        # Publish result back to captain
        result = WorldWaypointReqResult()
        result.altitude_mode = goal.altitude_mode
        result.position.north = self.nav_sts.position.north
        result.position.east = self.nav_sts.position.east
        result.position.depth = self.nav_sts.position.depth
        result.altitude = self.nav_sts.altitude
        result.orientation.roll = self.nav_sts.orientation.roll
        result.orientation.pitch = self.nav_sts.orientation.pitch
        result.orientation.yaw = self.nav_sts.orientation.yaw
        rospy.loginfo('%s: succeeded', self.name)
        self.absolute_action.set_succeeded(result)


    def get_config(self):
        """ Load parameters from the rosparam server """
        param_dict = {'max_velocity': 'move_mode/max_velocity',
                      'min_velocity_los': 'move_mode/min_velocity_los',
                      'max_angle_error': 'move_mode/max_angle_error'}

        self.parameters = cola2_ros_lib.Config()

        valid_config = True
        if not cola2_ros_lib.getRosParams(self.parameters, param_dict, self.name):
            valid_config = False

        # Check if sparus_los params are defined. If they are then load these params
        if rospy.has_param('move_mode/sparus_los/acceptance_radius'):
            sparus_los_param_dict = {'acceptance_radius': 'move_mode/sparus_los/acceptance_radius',
                                     'los_radius': 'move_mode/sparus_los/los_radius',
                                     'low_surge': 'move_mode/sparus_los/low_surge',
                                     'high_surge': 'move_mode/sparus_los/high_surge',
                                     'sway_correction': 'move_mode/sparus_los/sway_correction',
                                     'heave_mode_in_3D': 'move_mode/sparus_los/heave_mode_in_3D',
                                     'safety_max_distance_btw_waypoints': 'move_mode/sparus_los/safety_max_distance_btw_waypoints'}

            self.parameters.sparus_los = cola2_ros_lib.Config()
        
            if not cola2_ros_lib.getRosParams(self.parameters.sparus_los, sparus_los_param_dict, self.name):
                valid_config = False

        if not valid_config:
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way


if __name__ == '__main__':
    try:
        rospy.init_node('pilot', log_level=rospy.INFO) #log_level=rospy.DEBUG)
        pilot = Pilot(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
