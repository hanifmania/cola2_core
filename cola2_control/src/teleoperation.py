#!/usr/bin/env python
"""@@This node is subscribed to the map_ack output message. It is used to
   compute position and velocity setpoints out of the input joy message.@@"""

# ROS imports
import rospy

# Import messages
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import DiagnosticStatus
from auv_msgs.msg import BodyVelocityReq
from auv_msgs.msg import WorldWaypointReq
from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from cola2_msgs.srv import MaxJoyVelocity, MaxJoyVelocityResponse
from cola2_lib.diagnostic_helper import DiagnosticHelper
from cola2_lib import cola2_lib, cola2_ros_lib


class Teleoperation(object):
    """ This class recieves a joy message and generates a world_waypoint_req
        or a body_velocity_req.

        The joy message always have the same structure. The axis contain
        the value for pose and twist:
        --> axis: [x][y][z][roll][pitch][yaw][u][v][w][p][q][r]
        While the buttons decide if an axis is controlled in pose or in twist:
        --> buttons: [x][y][z][roll][pitch][yaw][u][v][w][p][q][r]
    """

    def __init__(self, name):
        """ Constructor """
        self.name = name
        self.last_map_ack = 0.0
        self.robot_name = ''

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Some vars
        self.map_ack_init = False
        self.map_ack_alive = True
        self.manual_pitch = False
        self.seq = 0
        self.nav_init = False
        self.base_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.nav_init = False

        # Get config
        self.actualize_base_pose = True  # Default
        self.get_config()

        # Create publishers
        self.pub_body_velocity_req = rospy.Publisher(
            '/cola2_control/body_velocity_req',
            BodyVelocityReq,
            queue_size=2)

        self.pub_world_waypoint_req = rospy.Publisher(
            '/cola2_control/world_waypoint_req',
            WorldWaypointReq,
            queue_size=2)

        self.pub_check_joystick = rospy.Publisher(
            '/cola2_control/map_ack_ok',
            String,
            queue_size=2)

        # Create subscribers
        rospy.Subscriber("cola2_control/map_ack_ack",
                         String,
                         self.map_ack_ack_callback,
                         queue_size=1)

        rospy.Subscriber("/cola2_control/map_ack_data",
                         Joy,
                         self.map_ack_data_callback,
                         queue_size=1)

        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.nav_sts_update,
                         queue_size=1)

        # Create services
        self.load_trajectory_srv = rospy.Service(
            '/cola2_control/set_max_joy_velocity',
            MaxJoyVelocity,
            self.set_max_joy_vel)

        # Init periodic check timer
        rospy.Timer(rospy.Duration(1.0), self.check_map_ack)

    def nav_sts_update(self, data):
        """ This is the callback for the navigation message """
        self.nav_init = True
        self.last_pose[0] = data.position.north
        self.last_pose[1] = data.position.east
        self.last_pose[2] = data.position.depth
        self.last_pose[3] = data.orientation.roll
        self.last_pose[4] = data.orientation.pitch
        self.last_pose[5] = data.orientation.yaw

    def map_ack_ack_callback(self, ack_msg):
        """ This is the callback for the ack safety message """
        data = ack_msg.data.split(' ')
        if data[1] == 'ack' and data[0] == str(self.seq + 1):
            self.map_ack_alive = True
            self.map_ack_init = True
            self.seq = self.seq + 1
            self.last_map_ack = rospy.Time.now().to_sec()

    def check_map_ack(self, event):
        """ This is a callback for a timer. It publishes ack safety message
            and pose and velocity safety messages if map_ack is lost """
        if self.map_ack_init:
            self.diagnostic.add(
                "last_ack",
                str(rospy.Time.now().to_sec() - self.last_map_ack))
            if self.map_ack_alive:
                self.map_ack_alive = False
                self.diagnostic.setLevel(DiagnosticStatus.OK)
            else:
                rospy.loginfo("%s: we have lost map_ack!", self.name)
                self.diagnostic.setLevel(
                    DiagnosticStatus.WARN,
                    'Communication with map_ack lost!')
                body_velocity_req = BodyVelocityReq()
                body_velocity_req.goal.priority = GoalDescriptor.PRIORITY_LOW
                body_velocity_req.goal.requester = self.name + '_vel'
                body_velocity_req.twist.linear.x = 0.0
                body_velocity_req.twist.linear.y = 0.0
                body_velocity_req.twist.linear.z = 0.0
                body_velocity_req.twist.angular.x = 0.0
                body_velocity_req.twist.angular.y = 0.0
                body_velocity_req.twist.angular.z = 0.0
                body_velocity_req.disable_axis.x = True
                body_velocity_req.disable_axis.y = True
                body_velocity_req.disable_axis.z = True
                body_velocity_req.disable_axis.roll = True
                body_velocity_req.disable_axis.pitch = True
                body_velocity_req.disable_axis.yaw = True
                body_velocity_req.header.stamp = rospy.Time().now()
                self.pub_body_velocity_req.publish(body_velocity_req)

                world_waypoint_req = WorldWaypointReq()
                world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_LOW
                world_waypoint_req.goal.requester = self.name + '_pose'
                world_waypoint_req.disable_axis.x = True
                world_waypoint_req.disable_axis.y = True
                world_waypoint_req.disable_axis.z = True
                world_waypoint_req.disable_axis.roll = True
                world_waypoint_req.disable_axis.pitch = True
                world_waypoint_req.disable_axis.yaw = True
                world_waypoint_req.header.stamp = rospy.Time().now()
                self.pub_world_waypoint_req.publish(world_waypoint_req)
        else:
            rospy.loginfo("%s: waiting for map ack...", self.name)

        # Send ack message
        msg = String()
        msg.data = str(self.seq) + ' ok'
        self.pub_check_joystick.publish(msg)

    def map_ack_data_callback(self, data):
        """ This is the main callback. Data is recieved, processed and sent
            to pose and velocity controllers """

        # Compute desired positions and velocities
        desired = [0 for x in range(12)]
        for i in range(6):
            if (data.axes[i] < 0):
                desired[i] = abs(data.axes[i]) * self.min_pos[i] + self.base_pose[i]
            else:
                desired[i] = data.axes[i] * self.max_pos[i] + self.base_pose[i]
            if i > 2:
                # Normalize angles
                desired[i] = cola2_lib.normalizeAngle(desired[i])

        for i in range(6, 12):
            if (data.axes[i] < 0):
                desired[i] = abs(data.axes[i]) * self.min_vel[i - 6]
            else:
                desired[i] = data.axes[i] * self.max_vel[i - 6]

        # Check if pose controller is enabled
        for b in range(6):
            if data.buttons[b] == 1:
                self.pose_controlled_axis[b] = True
                if self.actualize_base_pose:
                    self.base_pose[b] = self.last_pose[b]
                rospy.loginfo("%s: axis %s now is pose", self.name, str(b))

        # Check if velocity controller is enabled
        for b in range(6, 12):
            if data.buttons[b] == 1:
                self.pose_controlled_axis[b - 6] = False
                rospy.loginfo("%s: axis %s now is velocity", self.name, str(b-6))

        if self.nav_init:
            # Positions
            world_waypoint_req = WorldWaypointReq()
            world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_MANUAL_OVERRIDE
            world_waypoint_req.goal.requester = self.name + '_pose'
            world_waypoint_req.position.north = desired[0]
            world_waypoint_req.position.east = desired[1]
            world_waypoint_req.position.depth = desired[2]
            world_waypoint_req.orientation.roll = desired[3]
            world_waypoint_req.orientation.pitch = desired[4]
            world_waypoint_req.orientation.yaw = desired[5]
            world_waypoint_req.disable_axis.x = not self.pose_controlled_axis[0]
            world_waypoint_req.disable_axis.y = not self.pose_controlled_axis[1]
            world_waypoint_req.disable_axis.z = not self.pose_controlled_axis[2]
            world_waypoint_req.disable_axis.roll = not self.pose_controlled_axis[3]
            world_waypoint_req.disable_axis.pitch = not self.pose_controlled_axis[4]
            world_waypoint_req.disable_axis.yaw = not self.pose_controlled_axis[5]
            world_waypoint_req.header.stamp = rospy.Time().now()
            # if not world_waypoint_req.disable_axis.pitch:
            #    rospy.logfatal("%s: PITCH IS NOT DISABLED!", self.name)
            #    world_waypoint_req.disable_axis.pitch = True
            self.pub_world_waypoint_req.publish(world_waypoint_req)

            # Velocities
            body_velocity_req = BodyVelocityReq()
            body_velocity_req.goal.priority = GoalDescriptor.PRIORITY_MANUAL_OVERRIDE
            body_velocity_req.goal.requester = self.name + '_vel'
            body_velocity_req.twist.linear.x = desired[6]
            body_velocity_req.twist.linear.y = desired[7]
            body_velocity_req.twist.linear.z = desired[8]
            body_velocity_req.twist.angular.x = desired[9]
            body_velocity_req.twist.angular.y = desired[10]
            body_velocity_req.twist.angular.z = desired[11]
            body_velocity_req.disable_axis.x = self.pose_controlled_axis[0]
            body_velocity_req.disable_axis.y = self.pose_controlled_axis[1]
            body_velocity_req.disable_axis.z = self.pose_controlled_axis[2]
            body_velocity_req.disable_axis.roll = self.pose_controlled_axis[3]
            body_velocity_req.disable_axis.pitch = self.pose_controlled_axis[4]
            body_velocity_req.disable_axis.yaw = self.pose_controlled_axis[5]

            # Check if DoF is disable
            if abs(body_velocity_req.twist.linear.x) < 0.025:
                body_velocity_req.disable_axis.x = True

            if abs(body_velocity_req.twist.linear.y) < 0.025:
                body_velocity_req.disable_axis.y = True

            if abs(body_velocity_req.twist.linear.z) < 0.025:
                body_velocity_req.disable_axis.z = True

            if abs(body_velocity_req.twist.angular.x) < 0.01:
                body_velocity_req.disable_axis.roll = True

            if abs(body_velocity_req.twist.angular.y) < 0.01:
                body_velocity_req.disable_axis.pitch = True

            if abs(body_velocity_req.twist.angular.z) < 0.01:
                body_velocity_req.disable_axis.yaw = True

            # If all DoF are disabled set priority to LOW
            if (body_velocity_req.disable_axis.x and
                    body_velocity_req.disable_axis.y and
                    body_velocity_req.disable_axis.z and
                    body_velocity_req.disable_axis.roll and
                    body_velocity_req.disable_axis.pitch and
                    body_velocity_req.disable_axis.yaw):
                body_velocity_req.goal.priority = GoalDescriptor.PRIORITY_LOW

            # Publish message
            body_velocity_req.header.stamp = rospy.Time().now()
            self.pub_body_velocity_req.publish(body_velocity_req)

    def get_config(self):
        """ Get config from param server """
        param_dict = {'max_pos': 'teleoperation/max_pos',
                      'min_pos': 'teleoperation/min_pos',
                      'max_vel': 'teleoperation/max_vel',
                      'min_vel': 'teleoperation/min_vel',
                      'pose_controlled_axis': 'teleoperation/pose_controlled_axis',
                      'base_pose': 'teleoperation/base_pose',
                      'actualize_base_pose': 'teleoperation/actualize_base_pose',
                      'robot_name': '/navigator/robot_frame_id'}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way

    def set_max_joy_vel(self, req):
        rospy.loginfo("%s: change max/min joy velocity", self.name)
        for i in range(6):
            self.max_vel[i] = req.max_joy_velocity[i]
            self.min_vel[i] = -req.max_joy_velocity[i]

        return(MaxJoyVelocityResponse(True))

if __name__ == '__main__':
    try:
        rospy.init_node('teleoperation')
        teleoperation = Teleoperation(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
