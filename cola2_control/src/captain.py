#!/usr/bin/env python
"""@@This node is used to load and execute missions or tasks. Using services,
user is able to tell the captain to do different things. This node mainly
interacts with the pilot.@@"""

"""
Modified 11/2015
@author: narcis palomeras
"""

# ROS imports
import rospy
import rosparam
import actionlib
import tf
import dynamic_reconfigure.client

# import ipdb
# Msgs imports

from cola2_msgs.msg import WorldWaypointReqAction, WorldWaypointReqGoal
from cola2_msgs.msg import MissionStatus
from cola2_msgs.srv import Goto, GotoResponse, GotoRequest
from cola2_msgs.srv import GotoWithYaw, GotoWithYawResponse
from cola2_msgs.srv import String, StringResponse
from cola2_msgs.srv import StringList
from cola2_msgs.srv import Submerge, SubmergeResponse, SubmergeRequest
from cola2_msgs.srv import MissionTimeout, MissionTimeoutRequest
from auv_msgs.msg import WorldWaypointReq
from auv_msgs.msg import NavSts, GoalDescriptor

from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Path  # An array of poses that represents a path
from geometry_msgs.msg import PoseStamped

from cola2_lib.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus

# Python imports
from cola2_lib import cola2_lib, cola2_ros_lib, NED
from threading import Thread
import numpy as np
import os.path


class Captain:
    """ This node is used to enable and disable behaviors of the AUV """

    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Flags used to know which action is running
        self.init_trajectory = False
        self.init_keep_pose = False
        self.init_goto = False
        self.init_keep_z = False


        # Flag used to know if some trajectory has been loaded
        self.trajectory_loaded = False
        self.trajectory_time_out = 0.0
        # Path used to publish trajectory for RViz
        self.path = Path()

        # Variables used to keep track of the NED origin
        self.last_origin_lat = -1234  # Random impossible number
        self.last_origin_lon = -1234

        # Object that stores a trajectory
        self.trajectory = cola2_lib.Trajectory()

        # Create navigation message
        self.nav = NavSts()

        # Create mission status message (same content as a diagnostic)
        self.mission_status = MissionStatus()
        self.diagnostic = DiagnosticHelper(self.name, "soft")
        self.diagnostic.add("trajectory_enabled", "False")

        # Create publisher
        self.pub_mission_status = rospy.Publisher(
                                    "/cola2_control/mission_status",
                                    MissionStatus,
                                    queue_size = 2)

        self.pub_trajectory = rospy.Publisher(
                                        "/cola2_control/trajectory_path",
                                        Path, latch=True,
                                        queue_size = 2)
        self.pub_wwr = rospy.Publisher(
            "/cola2_control/world_waypoint_req",
            WorldWaypointReq,
            queue_size = 2)

        # Create subscriber
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav_sts,
                         queue_size = 1)

        rospy.Subscriber("/move_base_simple/goal",
                         PoseStamped,
                         self.nav_goal,
                         queue_size = 1)


        # Create actionlib client (for services that take a long time to exec.)
        # This is the link with the pilot
        self.client_absolute = actionlib.SimpleActionClient(
                                            'absolute_movement',
                                            WorldWaypointReqAction)
        rospy.loginfo('%s: waiting for pilot server', self.name)
        self.client_absolute.wait_for_server()  # Wait until found
        rospy.loginfo('%s: pilot server found', self.name)

        # Create services
        self.load_trajectory_srv = rospy.Service(
                                        '/cola2_control/load_trajectory',
                                        String,
                                        self.load_trajectory)

        self.load_trajectory_str_srv = rospy.Service(
                                        '/cola2_control/load_trajectory_str',
                                        String,
                                        self.load_trajectory_str)

        self.load_trajectory_config_srv = rospy.Service(
                                        '/cola2_control/load_trajectory_config',
                                        Empty,
                                        self.load_trajectory_config)

        self.enable_trajectory_srv = rospy.Service(
                                        '/cola2_control/enable_trajectory',
                                        Empty,
                                        self.enable_trajectory)

        self.disable_trajectory_srv = rospy.Service(
                                        '/cola2_control/disable_trajectory',
                                        Empty,
                                        self.disable_trajectory)


        self.enable_keep_depth_srv = rospy.Service(
                                    'cola2_control/enable_keep_depth',
                                    Empty,
                                    self.enable_keep_depth)


        self.enable_keep_position_s2_srv = rospy.Service(
                                    'cola2_control/enable_keep_position_s2',
                                    Empty,
                                    self.enable_keep_position_s2)

        self.enable_global_keep_given_position_s2_srv = rospy.Service(
                                    'cola2_control/enable_global_keep_given_position_s2',
                                    Goto,
                                    self.enable_global_keep_given_position_s2)

        self.enable_keep_position_g500_srv = rospy.Service(
                                    'cola2_control/enable_keep_position_g500',
                                    Empty,
                                    self.enable_keep_position_g500)

        self.disable_keep_position_srv = rospy.Service(
                                    '/cola2_control/disable_keep_position',
                                    Empty,
                                    self.disable_keep_position)

        self.goto_srv_local = rospy.Service('/cola2_control/goto_local',
                                             Goto,
                                             self.goto_local)

        self.goto_loc_block_srv = rospy.Service('/cola2_control/goto_local_block',
                                             Goto,
                                             self.goto_local_block)

        self.goto_rel_srv = rospy.Service('/cola2_control/goto_relative',
                                             Goto,
                                             self.goto_relative)

        self.goto_xy_srv = rospy.Service('/cola2_control/goto_xy',
                                             Goto,
                                             self.goto_xy)

        self.goto_srv_global = rospy.Service('/cola2_control/goto_global',
                                             Goto,
                                             self.goto_global)

        self.goto_srv_global_block = rospy.Service('/cola2_control/goto_global_block',
                                             Goto,
                                             self.goto_global_block)

        self.disable_goto_srv = rospy.Service('/cola2_control/disable_goto',
                                             Empty,
                                             self.disable_goto)

        self.submerge_srv = rospy.Service('/cola2_control/submerge',
                                    Submerge,
                                    self.submerge)

        self.keep_z_srv = rospy.Service('/cola2_control/keep_z',
                                    Submerge,
                                    self.keep_z)

        self.disable_keep_z_srv = rospy.Service('/cola2_control/disable_keep_z',
                                    Empty,
                                    self.disable_keep_z)

        self.goto_holonomic_srv = rospy.Service('/cola2_control/goto_holonomic',
                                                GotoWithYaw,
                                                self.goto_holonomic)

        self.goto_holonomic_block_srv = rospy.Service('/cola2_control/goto_holonomic_block',
                                                      GotoWithYaw,
                                                      self.goto_holonomic_block)

        # Create timer to publish mission status
        rospy.Timer(rospy.Duration(1.0), self.pub_mission_status_timer)

        # Map shutdown function
        rospy.on_shutdown(self.stop)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def goto_holonomic_block(self, req):
        if (not self.init_keep_pose and
            not self.init_trajectory and
            not self.init_goto):

            self.init_goto = True
            rospy.loginfo('%s Enable goto holonomic', self.name)
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = req.altitude_mode
            goal.position.north = req.north_lat
            goal.position.east = req.east_lon
            goal.position.depth = req.z
            goal.altitude = req.z
            goal.orientation.roll = 0.0
            goal.orientation.pitch = 0.0
            goal.orientation.yaw = req.yaw

            # Here we choose the movement type --> XYZYaw, neverending
            goal.mode = 'achieve_wp'
            goal.disable_axis.x = False
            goal.disable_axis.y = False
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False

            # Set tolerance
            goal.position_tolerance.x = req.tolerance
            goal.position_tolerance.y = req.tolerance
            goal.position_tolerance.z = req.tolerance
            goal.orientation_tolerance.roll = 4.00
            goal.orientation_tolerance.pitch = 4.00
            goal.orientation_tolerance.yaw = req.tolerance
            self.client_absolute.send_goal(goal)
            t = Thread(target=self.wait_goto, args=())
            t.daemon = True
            t.start()
            t.join()
            return GotoWithYawResponse(True)

        else:
            rospy.logerr('%s: unable to execute keepPosition, another service is running.',
                         self.name)
            rospy.logerr('%s: keepPose: %s',
                         self.name, self.init_keep_pose)
            rospy.logerr('%s: trajectory: %s',
                         self.name, self.init_trajectory)
            rospy.logerr('%s: goto: %s',
                         self.name, self.init_goto)

            return GotoWithYawResponse(False)


    def goto_holonomic(self, req):
        if (not self.init_keep_pose and
            not self.init_trajectory and
            not self.init_goto):

            self.init_goto = True
            rospy.loginfo('%s Enable goto holonomic', self.name)
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = req.altitude_mode
            goal.position.north = req.north_lat
            goal.position.east = req.east_lon
            goal.position.depth = req.z
            goal.altitude = req.z
            goal.orientation.roll = 0.0
            goal.orientation.pitch = 0.0
            goal.orientation.yaw = req.yaw

            # Here we choose the movement type --> XYZYaw, neverending
            goal.mode = 'achieve_wp'
            goal.disable_axis.x = False
            goal.disable_axis.y = False
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False

            # Set tolerance
            goal.position_tolerance.x = req.tolerance
            goal.position_tolerance.y = req.tolerance
            goal.position_tolerance.z = req.tolerance
            goal.orientation_tolerance.roll = 4.00
            goal.orientation_tolerance.pitch = 4.00
            goal.orientation_tolerance.yaw = req.tolerance
            self.client_absolute.send_goal(goal)
            t = Thread(target=self.wait_goto, args=())
            t.daemon = True
            t.start()
            return GotoWithYawResponse(True)

        else:
            rospy.logerr('%s: unable to execute keepPosition, another service is running.',
                         self.name)
            rospy.logerr('%s: keepPose: %s',
                         self.name, self.init_keep_pose)
            rospy.logerr('%s: trajectory: %s',
                         self.name, self.init_trajectory)
            rospy.logerr('%s: goto: %s',
                         self.name, self.init_goto)

            return GotoWithYawResponse(False)


    def keep_z(self, req):
        if (not self.init_trajectory and
            not self.init_keep_pose and
            not self.init_goto and
            not self.init_keep_z):

            self.init_keep_z = True
            t = Thread(target=self.send_keep_z_msg, args=(req.z, req.altitude_mode))
            t.daemon = True  # Close this thread when captain exits
            t.start()
            return SubmergeResponse(True)
        else:
            return SubmergeResponse(False)

    def send_keep_z_msg(self, z, altitude_mode):
        req = WorldWaypointReq()
        req.header.frame_id = 'girona500'
        r = rospy.Rate(10)
        i = 0
        while self.init_keep_z:
            req.header.stamp = rospy.Time().now()
            req.position.north = 0.0
            req.position.east = 0.0
            req.position.depth = z
            req.altitude = z
            req.orientation.roll = 0.0
            req.orientation.pitch = 0.0
            req.orientation.yaw = 0.0
            req.altitude_mode = altitude_mode
            req.disable_axis.x = True
            req.disable_axis.y = True
            req.disable_axis.z = False
            req.disable_axis.roll = True
            req.disable_axis.pitch = True
            req.disable_axis.yaw = True
            req.goal.requester = self.name + '_keep_z'
            req.goal.id = i
            req.goal.priority = GoalDescriptor.PRIORITY_NORMAL + 2
            self.pub_wwr.publish(req)
            r.sleep()
            i = i + 1

    def disable_keep_z(self, req):
        self.init_keep_z = False
        return EmptyResponse()

    def load_trajectory(self, req):
        """ This method parses a trajectory from ros param server. Input is
            the absolute path of the yaml file of the trajectory """

        # Delete all prevoius params
        __delete_param__('/trajectory/north')
        __delete_param__('/trajectory/east')
        __delete_param__('/trajectory/latitude')
        __delete_param__('/trajectory/longitude')
        __delete_param__('/trajectory/z')
        __delete_param__('/trajectory/altitude_mode')
        __delete_param__('/trajectory/mode')
        __delete_param__('/trajectory/time_out')
        __delete_param__('/trajectory/wait')
        __delete_param__('/trajectory/actions')

        # Load file if given
        if req.mystring != '':
            # Check if file exists
            if os.path.isfile(req.mystring):
                # Check if empty file
                with open(req.mystring, "r") as myfile:
                    data_content = myfile.read().replace('\n', '').strip()
                if len(data_content) == 0:
                    rospy.logerr("%s: empty trajectory file!", self.name)
                    return StringResponse('Empty trajectory file')

                # Try load file
                try:
                    paramlist = rosparam.load_file(req.mystring)
                    for params,ns in paramlist:
                        rosparam.upload_params(ns, params)
                except:
                    rospy.logerr("%s: unable to load trajectory file!", self.name)
                    return StringResponse('Unable to load trajectory file')
            else:
                rospy.logerr("%s: unable to find trajectory file! Provide absoulte path!", self.name)
                return StringResponse('Unable to find trajectory file')
        else:
            rospy.loginfo("%s: loading previous or default trajectory already in rosparam server", self.name)


        # Check for timeout
        if rospy.has_param("trajectory/time_out"):
            self.trajectory_time_out = rospy.get_param("trajectory/time_out")
        else:
            rospy.logerr("%s: unsafe to load. Missing trajectory/time_out", self.name)
            return StringResponse('Missing trajectory/time_out')

        # Config container
        config = cola2_ros_lib.Config()

        # Check for trajectory type (absolute or relative)
        if rospy.has_param("trajectory/north") and rospy.has_param("trajectory/east"):
            config.trajectory_type = 'relative'
        elif rospy.has_param("trajectory/latitude") and rospy.has_param("trajectory/longitude"):
            config.trajectory_type = 'absolute'
        else:
            rospy.logerr("%s: invalid trajectory!", self.name)
            return StringResponse('Invalid trajectory')

        # Define correct parameters
        param_dict = {'z': 'trajectory/z',
                      'altitude_mode': 'trajectory/altitude_mode',
                      'mode': 'trajectory/mode'}

        if (config.trajectory_type == 'absolute'):
            param_dict['lat'] = 'trajectory/latitude'
            param_dict['lon'] = 'trajectory/longitude'
        elif (config.trajectory_type == 'relative'):
            param_dict['north'] = 'trajectory/north'
            param_dict['east'] = 'trajectory/east'
        else:
            rospy.logerr("%s: invalid trajectory type!", self.name)
            return StringResponse()

        if rospy.has_param("trajectory/wait"):
            param_dict['wait'] = 'trajectory/wait'
        else:
            config.wait=[]

        if rospy.has_param("trajectory/actions"):
            param_dict['actions'] = 'trajectory/actions'
        else:
            config.actions=[]

        # print "---> Trajectory params to load: \n", param_dict

        # Get trajectory
        if not cola2_ros_lib.getRosParams(config, param_dict, 'Trajectory'):
            rospy.logerr("%s: some trajectory parameters not found!", self.name)
            return StringResponse('Some trajectory parameters not found')

        if (config.trajectory_type == 'absolute'):
            self.trajectory.load(config.trajectory_type,
                                 config.lat,
                                 config.lon,
                                 config.z,
                                 config.altitude_mode,
                                 config.mode,
                                 config.wait,
                                 config.actions)
        else:
            self.trajectory.load(config.trajectory_type,
                                 config.north,
                                 config.east,
                                 config.z,
                                 config.altitude_mode,
                                 config.mode,
                                 config.wait,
                                 config.actions)

        # Generate path and publish
        self.path = self.generate_trajectory_path()
        self.pub_trajectory.publish(self.path)

        # Check services
        if not self.check_trajectory_services():
            rospy.logerr("%s: some trajectory services are not ready!", self.name)
            return StringResponse('Some trajectory services are not ready')

        # Get distance between first waypoint and current position
        wp_0 = self.trajectory.getWaypointNed(0)  # Get first waypoint
        distance = ((wp_0[NED.NORTH] - self.mission_status.current_north) ** 2 +
                    (wp_0[NED.EAST] - self.mission_status.current_east) ** 2) ** 0.5

        # Show distance
        rospy.loginfo("%s: the distance to the first waypoint is: %s m",
                      self.name, distance)  # Show distance
        # Check distance
        if distance > 300:  # Check distance to avoid loading wrong trajectories
            rospy.logerr("%s: distance to the first waypoint too large! Distance: %s m", self.name, distance)
            return StringResponse('Distance to firts waypoint too large')

        # Check first last waypoint at surface
        if (config.z[0] == 0.0 and config.altitude_mode[0] == False and
             config.z[-1] == 0.0 and config.altitude_mode[-1] == False):
            self.trajectory_loaded = True
            return StringResponse('Trajectory loaded')
        else:
            rospy.logerr("%s: Inital and final waypoints must be at surface! %s(%s), %s(%s)",
                         self.name, config.z[0], config.altitude_mode[0], config.z[-1], config.altitude_mode[-1])
            return StringResponse('Inital and final waypoints must be at surface')


    def load_trajectory_str(self, req):
        """ This method parses a trajectory from ros param server. Input is
            an string with all the values in a yaml format"""

        #WORK AROUND Since in python the \n in a string is not consider as a
        # newline it thinks is the same value. For this reason the yaml library
        # doesn't load the correctly the yaml string.
        # To aboid this problem We can create a temp file with the string and\
        # load like the original service.
        # Since is a service it needs a special
        # message to call it so its easier coyp it again.
        #
        #ifile = open("/tmp/mission.yaml","w")
        lines = req.mystring.split('\\n')
        for line in lines:
            if (line != '' ) :
                arguments = line.split(':')
                rosparam.upload_params(arguments[0], eval(arguments[1]))
            else:
                rospy.loginfo('Skiped empty line')
        #file.close()

        # Check for timeout
        if rospy.has_param("trajectory/time_out"):
            self.trajectory_time_out = rospy.get_param("trajectory/time_out")
        else:
            rospy.logerr("%s: unsafe to load. Missing trajectory/time_out", self.name)
            return StringResponse('Missing trajectory/time_out')

        # Config container
        config = cola2_ros_lib.Config()

        # Check for trajectory type (absolute or relative)
        if rospy.has_param("trajectory/north") and rospy.has_param("trajectory/east"):
            config.trajectory_type = 'relative'
        elif rospy.has_param("trajectory/latitude") and rospy.has_param("trajectory/longitude"):
            config.trajectory_type = 'absolute'
        else:
            rospy.logerr("%s: invalid trajectory!", self.name)
            return StringResponse('Invalid trajectory')

        # Define correct parameters
        param_dict = {'z': 'trajectory/z',
                      'altitude_mode': 'trajectory/altitude_mode',
                      'mode': 'trajectory/mode'}

        if (config.trajectory_type == 'absolute'):
            param_dict['lat'] = 'trajectory/latitude'
            param_dict['lon'] = 'trajectory/longitude'
        elif (config.trajectory_type == 'relative'):
            param_dict['north'] = 'trajectory/north'
            param_dict['east'] = 'trajectory/east'
        else:
            rospy.logerr("%s: invalid trajectory type!", self.name)
            return StringResponse()

        if rospy.has_param("trajectory/wait"):
            param_dict['wait'] = 'trajectory/wait'
        else:
            config.wait=[]

        if rospy.has_param("trajectory/actions"):
            param_dict['actions'] = 'trajectory/actions'
        else:
            config.actions=[]

        # Get trajectory
        if not cola2_ros_lib.getRosParams(config, param_dict, 'Trajectory'):
            rospy.logerr("%s: some trajectory parameters not found!", self.name)
            return StringResponse('Some trajectory parameters not found')

        # Set a trajectory class from config through cola2_lib method
        if (config.trajectory_type == 'absolute'):
            self.trajectory.load(config.trajectory_type,
                                 config.lat,
                                 config.lon,
                                 config.z,
                                 config.altitude_mode,
                                 config.mode,
                                 config.wait,
                                 config.actions)
        else:
            self.trajectory.load(config.trajectory_type,
                                 config.north,
                                 config.east,
                                 config.z,
                                 config.altitude_mode,
                                 config.mode,
                                 config.wait,
                                 config.actions)

        # Generate path and publish
        self.path = self.generate_trajectory_path()
        self.pub_trajectory.publish(self.path)

        # Check services
        if not self.check_trajectory_services():
            rospy.logerr("%s: some trajectory services are not ready!", self.name)
            return StringResponse('Some trajectory services are not ready')

        # Get distance between first waypoint and current position
        wp_0 = self.trajectory.getWaypointNed(0)  # Get first waypoint
        distance = ((wp_0[NED.NORTH] - self.mission_status.current_north) ** 2 +
                    (wp_0[NED.EAST] - self.mission_status.current_east) ** 2) ** 0.5

        # Show distance
        rospy.loginfo("%s: the distance to the first waypoint is: %s m",
                      self.name, distance)  # Show distance
        # Check distance
        if distance > 300:  # Check distance to avoid loading wrong trajectories
            rospy.logerr("%s: distance to the first waypoint too large! Distance: %s m", self.name, distance)
            return StringResponse('Distance to firts waypoint too large')

        # Check first last waypoint at surface
        if (config.z[0] == 0.0 and config.altitude_mode[0] == False and
             config.z[-1] == 0.0 and config.altitude_mode[-1] == False):
            self.trajectory_loaded = True
            return StringResponse('Trajectory loaded')
        else:
            rospy.logerr("%s: Inital and final waypoints must be at surface! %s(%s), %s(%s)",
                         self.name, config.z[0], config.altitude_mode[0], config.z[-1], config.altitude_mode[-1])
            return StringResponse('Inital and final waypoints must be at surface')

    def load_trajectory_config(self, req):
        """
        This method suppose that some other node has already loaded the mission
        in the configuration server so it only need to load
        """
        # print "!!!!!!!!!!!!!!!!!!!! Load Trajectory Config !!!!!!!!!!!"
        # Check for timeout
        if rospy.has_param("trajectory/time_out"):
            self.trajectory_time_out = rospy.get_param("trajectory/time_out")
        else:
            rospy.logerr("%s: unsafe to load. Missing trajectory/time_out", self.name)
            #return StringResponse('Missing trajectory/time_out')
            return EmptyResponse()

        # Config container
        config = cola2_ros_lib.Config()

        # Check for trajectory type (absolute or relative)
        if rospy.has_param("trajectory/north") and rospy.has_param("trajectory/east"):
            config.trajectory_type = 'relative'
        elif rospy.has_param("trajectory/latitude") and rospy.has_param("trajectory/longitude"):
            config.trajectory_type = 'absolute'
        else:
            rospy.logerr("%s: invalid trajectory!", self.name)
            #return StringResponse('Invalid trajectory')
            return EmptyResponse()

        # Define correct parameters
        param_dict = {'z': 'trajectory/z',
                      'altitude_mode': 'trajectory/altitude_mode',
                      'mode': 'trajectory/mode'}

        if (config.trajectory_type == 'absolute'):
            param_dict['lat'] = 'trajectory/latitude'
            param_dict['lon'] = 'trajectory/longitude'
        elif (config.trajectory_type == 'relative'):
            param_dict['north'] = 'trajectory/north'
            param_dict['east'] = 'trajectory/east'
        else:
            rospy.logerr("%s: invalid trajectory type!", self.name)
            return EmptyResponse()
            #return StringResponse()

        if rospy.has_param("trajectory/wait"):
            param_dict['wait'] = 'trajectory/wait'
        else:
            config.wait=[]
        if rospy.has_param("trajectory/actions"):
            param_dict['actions'] = 'trajectory/actions'
        else:
            config.actions=[]

        # Get trajectory
        if not cola2_ros_lib.getRosParams(config, param_dict, 'Trajectory'):
            rospy.logerr("%s: some trajectory parameters not found!", self.name)
            return StringResponse('Some trajectory parameters not found')

        # Set a trajectory class from config through cola2_lib method
        if (config.trajectory_type == 'absolute'):
            self.trajectory.load(config.trajectory_type,
                                 config.lat,
                                 config.lon,
                                 config.z,
                                 config.altitude_mode,
                                 config.mode,
                                 config.wait,
                                 config.actions)
        else:
            self.trajectory.load(config.trajectory_type,
                                 config.north,
                                 config.east,
                                 config.z,
                                 config.altitude_mode,
                                 config.mode,
                                 config.wait,
                                 config.actions)

        # Generate path and publish
        self.path = self.generate_trajectory_path()
        self.pub_trajectory.publish(self.path)

        # Check services
        if not self.check_trajectory_services():
            rospy.logerr("%s: some trajectory services are not ready!", self.name)
            #return StringResponse('Some trajectory services are not ready')
            return EmptyResponse()

        # Get distance between first waypoint and current position
        wp_0 = self.trajectory.getWaypointNed(0)  # Get first waypoint
        distance = ((wp_0[NED.NORTH] - self.mission_status.current_north) ** 2 +
                    (wp_0[NED.EAST] - self.mission_status.current_east) ** 2) ** 0.5

        # Show distance
        rospy.loginfo("%s: the distance to the first waypoint is: %s m",
                      self.name, distance)  # Show distance
        # Check distance
        if distance > 300:  # Check distance to avoid loading wrong trajectories
            rospy.logerr("%s: distance to the first waypoint too large! Distance: %s m", self.name, distance)
            #return StringResponse('Distance to firts waypoint too large')
            return EmptyResponse()

        # Check first last waypoint at surface
        if (config.z[0] == 0.0 and config.altitude_mode[0] == False and
             config.z[-1] == 0.0 and config.altitude_mode[-1] == False):
            self.trajectory_loaded = True
            #return StringResponse('Trajectory loaded')
            return EmptyResponse()
        else:
            rospy.logerr("%s: Inital and final waypoints must be at surface! %s(%s), %s(%s)",
                         self.name, config.z[0], config.altitude_mode[0], config.z[-1], config.altitude_mode[-1])
            #return StringResponse('Inital and final waypoints must be at surface')
            return EmptyResponse()

    def enable_trajectory(self, req):
        """ Follows a list of way-points using the LOS algorithm.
            Once each way-point is reached, if some waiting time is defined,
            performs a keep position movement to this way-point """
        if not self.trajectory_loaded:
            rospy.logerr("%s: trajectory not loaded!", self.name)
        else:
            # Start trajectory
            if (not self.init_trajectory and
                 not self.init_keep_pose and
                 not self.init_goto and
                 not self.init_keep_z):  # If there is no other action running

                # Set captain flag to true
                self.init_trajectory = True
                self.diagnostic.add("trajectory_enabled", "True")

                ############### Old way to set mission timeout ##############
                """ try:  # Try the following
                    # Set mission timeout, a service of the safety package
                    rospy.wait_for_service('/mission_timeout', 5)
                    self.mission_timeout_srv = rospy.ServiceProxy(
                                                    '/mission_timeout',
                                                    MissionTimeout)
                    # Declare a MissionTimeoutRequest service
                    mt = MissionTimeoutRequest()
                    # Set the flag to start the mission
                    mt.start_mission = True
                    # Set the time_out field
                    mt.time_out = self.trajectory_time_out
                    # Display a message
                    rospy.loginfo('%s: init trajectory with mission_timeout: %s',
                                  self.name, mt)
                    response = self.mission_timeout_srv(mt)
                    if not response.success:
                        rospy.logerr('%s: error initializing mission timeout',
                                     self.name)
                except rospy.exceptions.ROSException:  # If not
                    # Display warning message, mission will start without timeout
                    rospy.logerr('%s: error initializing mission timeout. Are you in simulation mode?',
                                 self.name) """
                #############################################################

                ############### New way to set mission timeout ##############
                try:
                    client = dynamic_reconfigure.client.Client( "diagnostics_supervisor",
                                                                timeout=10 )
                    client.update_configuration({"timeout": self.trajectory_time_out*60})

                except rospy.exceptions.ROSException:
                    rospy.logerr('%s, Error setting Mission Timeout.', self.name)
                #############################################################

                # Start trajectory thread
                t = Thread(target=self.follow_trajectory, args=())
                t.daemon = True  # Close this thread when captain exits
                t.start()

            else:
                # Error message
                rospy.logerr('%s: impossible to enable trajectory, another service is running',
                             self.name)

                # Info to track which service is already running
                rospy.logwarn('%s: keepPose: %s',
                             self.name, self.init_keep_pose)
                rospy.logwarn('%s: trajectory: %s',
                             self.name, self.init_trajectory)
                rospy.logwarn('%s: goto: %s',
                             self.name, self.init_goto)
                rospy.logwarn('%s: keep_z: %s',
                             self.name, self.init_keep_z)

        return EmptyResponse()


    def disable_trajectory(self, req):
        """ This method disables trajectory mode """
        if self.init_trajectory:  # If there is a trajectory action running
            # Print message
            rospy.loginfo("%s: disable trajectory", self.name)

            # Set captain flag to false
            self.init_trajectory = False
            self.diagnostic.add("trajectory_enabled", "False")

            # Cancel the goal through actionlib server
            self.client_absolute.cancel_goal()
        else:
            rospy.logerr('%s: no trajectory service active', self.name)

        return EmptyResponse()


    def generate_trajectory_path(self):
        """ Generate trajectory path for RViz """
        # Fill path struct with all waypoints
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = '/world'

        if self.trajectory.init_ned:
            for i in range(len(self.trajectory.z)):
                pose = PoseStamped()
                pose.header.frame_id = path.header.frame_id
                wp = self.trajectory.getWaypointNed(i)
                pose.pose.position.x = wp[NED.NORTH]
                pose.pose.position.y = wp[NED.EAST]
                pose.pose.position.z = wp[NED.DEPTH]
                path.poses.append(pose)

        return path


    def follow_trajectory(self):
        """ Follow trajectory behavior. This is a thread started when enable
            trajectory service is called """
        # For each waypoint...
        for i in range(len(self.trajectory.z)):
            # Execute service related to the waypoint
            if self.trajectory.actions[i] != '':  # If there is a service to be run
                if rospy.has_param("trajectory/" + self.trajectory.actions[i]):
                    # Get service string from trajectory
                    service_string = rospy.get_param("trajectory/" + self.trajectory.actions[i])

                    # Check if it is an empty type service or a string type service
                    if len(service_string) == 2:
                        # Call empty service
                        try:
                            rospy.wait_for_service(service_string[0], 1)  # Wait 1s
                            self.service = rospy.ServiceProxy(service_string[0], Empty)
                            self.service()
                            rospy.loginfo('%s: successful call to service',
                                          self.name)
                        except:
                            rospy.logerr('%s: unable to call empty service on waypoint %s',
                                         self.name, i)
                            if service_string[1] != 'false':
                                # Abort trajectory
                                self.disable_trajectory(Empty)
                    elif len(service_string) > 2:
                        # Call service with string arguments
                        try:
                            rospy.wait_for_service(service_string[0], 1)  # Wait 1s
                            self.service = rospy.ServiceProxy(service_string[0], StringList)
                            response = self.service(service_string[1:len(service_string)-1])
                            succeeded = response.flag
                            rospy.loginfo('%s: call to service done. Response: %s',
                                          self.name, succeeded)
                        except:
                            rospy.logerr('%s: unable to call string service on waypoint %s',
                                         self.name, i)
                            succeeded = False

                        # Check what to do if not succeeded
                        if service_string[len(service_string)-1] != 'false' and not succeeded:
                            # Abort trajectory
                            rospy.loginfo('%s: aborting trajectory due to service failure',
                                          self.name)
                            self.init_trajectory = False
                            self.diagnostic.add("trajectory_enabled", "False")
                            return EmptyResponse()
                    else:
                        rospy.logerr('%s: service with not enough arguments on waypoint %s',
                                 self.name, i)
                else:
                    rospy.logerr('%s: impossible to parse service of waypoint %s',
                             self.name, i)

            # Publish trajectory to follow each waypoint
            self.pub_trajectory.publish(self.path)

            # Creates a goal to send to the action server
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name

            # It is important that the first goal.id in a trajectory
            # is 0 (pilot requirement)
            goal.goal.id = i

            goal.goal.priority = self.trajectory.priority
            goal.altitude_mode = self.trajectory.altitude_mode[i]

            # Take timeout
            goal.timeout = self.trajectory.computeWaypointTimeout(
                i, self.nav.position.north, self.nav.position.east,
                self.nav.position.depth, self.nav.altitude)

            rospy.loginfo('%s: computed timeout for waypoint %s: %s s',
                          self.name, i, goal.timeout)

            # The trajectory is defined in Lat, Lon coordinates. Conv. needed
            wp_i = self.trajectory.getWaypointNed(i)
            goal.position.north = wp_i[NED.NORTH]
            goal.position.east = wp_i[NED.EAST]

            goal.position.depth = self.trajectory.z[i]
            goal.altitude = self.trajectory.z[i]
            goal.orientation.roll = 0.0
            goal.orientation.pitch = 0.0
            goal.orientation.yaw = 0.0

            # Set mode and disable_axis
            goal.mode = self.trajectory.mode
            goal.disable_axis.x = False
            goal.disable_axis.y = True
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False

            # Set tolerance
            goal.position_tolerance.x = self.trajectory.tolerance
            goal.position_tolerance.y = self.trajectory.tolerance
            goal.position_tolerance.z = self.trajectory.tolerance
            goal.orientation_tolerance.roll = 3.1416
            goal.orientation_tolerance.pitch = 3.1416
            goal.orientation_tolerance.yaw = 3.1416

            # Fill mission_status topic
            self.mission_status.current_wp = i + 1
            self.mission_status.total_wp = len(self.trajectory.z)
            self.mission_status.wp_north = wp_i[NED.NORTH]
            self.mission_status.wp_east = wp_i[NED.EAST]
            self.mission_status.altitude_mode = self.trajectory.altitude_mode[i]
            self.mission_status.wp_depth_altitude = self.trajectory.z[i]
            self.mission_status.wp_remaining_time = self.trajectory.wait[i]

            self.diagnostic.add("current_wp", str(self.mission_status.current_wp))
            self.diagnostic.add("total_wp", str(self.mission_status.total_wp))
            self.diagnostic.add("wp_north", str(self.mission_status.wp_north))
            self.diagnostic.add("wp_east", str(self.mission_status.wp_east))
            self.diagnostic.add("altitude_mode", str(self.mission_status.altitude_mode))
            self.diagnostic.add("wp_depth_altitude", str(self.mission_status.wp_depth_altitude))
            self.diagnostic.add("wp_remaining_time", str(self.mission_status.wp_remaining_time))



            # Send the waypoint through actionlib server
            rospy.loginfo("%s: request GOAL:\n%s", self.name, goal)
            self.client_absolute.send_goal(goal)

            # Here the for is delayed until result
            self.client_absolute.wait_for_result()
            result = self.client_absolute.get_result()
            rospy.loginfo("%s: obtained RESULT:\n%s", self.name, result)

            # Check if the trajectory has been disabled
            if not self.init_trajectory:
                rospy.loginfo("%s: trajectory disabled", self.name)
                return EmptyResponse()

            # Check if the vehicle has to keep this way-point for a while
            if self.trajectory.wait[i] > 0.0:
                # Here we choose the movement type --> neverending Keep Pose
                goal.mode = 'neverending'
                goal.disable_axis.x = False
                goal.disable_axis.y = False
                goal.disable_axis.z = False
                goal.disable_axis.roll = True
                goal.disable_axis.pitch = True
                goal.disable_axis.yaw = True

                # Set tolerance to 0.01
                goal.position_tolerance.x = 0.01
                goal.position_tolerance.y = 0.01
                goal.position_tolerance.z = 0.01
                goal.orientation_tolerance.roll = 0.01
                goal.orientation_tolerance.pitch = 0.01
                goal.orientation_tolerance.yaw = 0.01

                # Wait for n seconds
                rospy.loginfo("%s: wait for %s seconds",
                              self.name, self.trajectory.wait[i])

                self.client_absolute.send_goal(goal)

                for w in range(int(self.trajectory.wait[i])):
                    rospy.sleep(1.0)
                    # Update MissionStatus topic
                    self.mission_status.wp_remaining_time = self.trajectory.wait[i] - w
                    self.diagnostic.add("wp_remaining_time", str(self.mission_status.wp_remaining_time))

                    # Check if the trajectory is aborted while waiting
                    if not self.init_trajectory:
                        rospy.loginfo("%s: trajectory disabled", self.name)
                        return EmptyResponse()

                rospy.loginfo("%s: waiting done, %s s",
                              self.name, self.trajectory.wait[i])

                # Abort the action of waiting when the time has expired
                self.client_absolute.cancel_goal()
                rospy.loginfo("%s: finalize keep pose action", self.name)

        # When the trajectory finalizes mark it and return the service
        self.init_trajectory = False
        self.diagnostic.add("trajectory_enabled", "False")


    def check_trajectory_services(self):
        """ This method checks availability of all trajectory services. This is
            called when loading a trajectory """
        try:
            valid_services = True
            for i in range(len(self.trajectory.actions)):
                if self.trajectory.actions[i] != '':  # If there is a service to be run on this waypoint
                    if rospy.has_param("trajectory/" + self.trajectory.actions[i]):
                        # Get service string from trajectory
                        service_string = rospy.get_param("trajectory/" + self.trajectory.actions[i])

                        # Check service
                        if len(service_string) == 0:
                            rospy.logerr('%s: empty service on waypoint %s!',
                                         self.name, i)
                            valid_services = False
                        elif len(service_string) == 1:
                            rospy.logerr('%s: invalid number of arguments in service %s of waypoint %s!',
                                         self.name, service_string[0], i)
                            valid_services = False
                        else:
                            try:
                                rospy.wait_for_service(service_string[0], 1)  # Wait 1s
                                if len(service_string) == 2:
                                    service = rospy.ServiceProxy(service_string[0], Empty)
                                else:
                                    service = rospy.ServiceProxy(service_string[0], StringList)
                            except:
                                rospy.logerr('%s: unable to locate service %s of waypoint %s!',
                                             self.name, service_string[0], i)
                                valid_services = False
                    else:
                        rospy.logerr('%s: impossible to parse service of waypoint %s!',
                                 self.name, i)
                        valid_services = False
            return valid_services
        except:
            rospy.logerr('%s: unexpected error checking services!', self.name)
            return False



    def enable_keep_depth(self, req):
        """ This mode keeps the depth for the AUV but no the other DoFs. """
        req = SubmergeRequest()
        req.z = self.nav.position.depth
        req.altitude_mode = False
        self.keep_z(req)
        return EmptyResponse()


    def enable_keep_position_s2(self, req):
        """ This mode keeps the position of the AUV. The goal is set at the
            position of the AUV. Suitable for s2 type vehicles """
        if (not self.init_keep_pose and
             not self.init_trajectory and
             not self.init_goto and
             not self.init_keep_z):  # If there is no other action running

            # Set captain flag to true
            self.init_keep_pose = True

            # Show message of activation
            rospy.loginfo('%s: enable keep position', self.name)

            # Define a goal with current position
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = False
            goal.position.north = self.nav.position.north
            goal.position.east = self.nav.position.east
            goal.position.depth = self.nav.position.depth
            goal.altitude = self.nav.altitude
            goal.orientation.roll = self.nav.orientation.roll
            goal.orientation.pitch = self.nav.orientation.pitch
            goal.orientation.yaw = self.nav.orientation.yaw

            # Here we choose the movement type --> XYZYaw, neverending
            goal.mode = 'sparus_keep_pose'
            goal.disable_axis.x = False
            goal.disable_axis.y = True
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False

            # Set tolerance
            goal.position_tolerance.x = 0.4
            goal.position_tolerance.y = 0.4
            goal.position_tolerance.z = 0.4
            goal.orientation_tolerance.roll = 0.4
            goal.orientation_tolerance.pitch = 0.4
            goal.orientation_tolerance.yaw = 0.4
            self.client_absolute.send_goal(goal)
        else:
            # Error message
            rospy.logerr('%s: unable to execute service to keep pose, another service is running',
                         self.name)

            # Info to track which service is already running
            rospy.logwarn('%s: keepPose: %s',
                         self.name, self.init_keep_pose)
            rospy.logwarn('%s: trajectory: %s',
                         self.name, self.init_trajectory)
            rospy.logwarn('%s: goto: %s',
                         self.name, self.init_goto)
            rospy.logwarn('%s: keep_z: %s',
                         self.name, self.init_keep_z)

        return EmptyResponse()


    def enable_global_keep_given_position_s2(self, req):
        """ This mode keeps the position of the AUV. The goal is set at the
            position request. Suitable for s2 type vehicles """
        if (not self.init_keep_pose and
             not self.init_trajectory and
             not self.init_goto and
             not self.init_keep_z):  # If there is no other action running

            # Set captain flag to true
            self.init_keep_pose = True

            # Show message of activation
            rospy.loginfo('%s: enable keep given position', self.name)

            # TO NED
            pose = self.trajectory.ned.geodetic2ned([req.north_lat,
                                                    req.east_lon,
                                                    0.0])

            # Define a goal with current position
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = req.altitude_mode
            goal.position.north = pose[0]
            goal.position.east = pose[1]
            goal.position.depth = req.z
            goal.altitude = req.z
            goal.orientation.roll = 0.0
            goal.orientation.pitch = 0.0
            goal.orientation.yaw = 0.0

            # Here we choose the movement type --> XYZYaw, neverending
            goal.mode = 'sparus_keep_pose'
            goal.disable_axis.x = False
            goal.disable_axis.y = True
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False

            # Set tolerance
            goal.position_tolerance.x = 0.4
            goal.position_tolerance.y = 0.4
            goal.position_tolerance.z = 0.4
            goal.orientation_tolerance.roll = 0.4
            goal.orientation_tolerance.pitch = 0.4
            goal.orientation_tolerance.yaw = 0.4
            self.client_absolute.send_goal(goal)
        else:
            # Error message
            rospy.logerr('%s: unable to execute service to keep given pose, another service is running',
                         self.name)

            # Info to track which service is already running
            rospy.logwarn('%s: keep_pose: %s',
                         self.name, self.init_keep_pose)
            rospy.logwarn('%s: trajectory: %s',
                         self.name, self.init_trajectory)
            rospy.logwarn('%s: goto: %s',
                         self.name, self.init_goto)
            rospy.logwarn('%s: keep_z: %s',
                         self.name, self.init_keep_z)

        return GotoResponse(True)


    def enable_keep_position_g500(self, req):
        """ This mode keeps the position of the AUV. The goal is set at the
            position of the AUV. Suitable for g500 type vehicles """
        if (not self.init_keep_pose and
             not self.init_trajectory and
             not self.init_goto and
             not self.init_keep_z):  # If there is no other action running

            # Set captain flag to true
            self.init_keep_pose = True

            # Show message of activation
            rospy.loginfo('%s: enable keep position', self.name)

            # Define a goal with current position
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = False
            goal.position.north = self.nav.position.north
            goal.position.east = self.nav.position.east
            goal.position.depth = self.nav.position.depth
            goal.altitude = self.nav.altitude
            goal.orientation.roll = self.nav.orientation.roll
            goal.orientation.pitch = self.nav.orientation.pitch
            goal.orientation.yaw = self.nav.orientation.yaw

            # Here we choose the movement type --> XYZYaw, neverending
            goal.mode = 'neverending'
            goal.disable_axis.x = False
            goal.disable_axis.y = False
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False

            # Set tolerance
            goal.position_tolerance.x = 0.01
            goal.position_tolerance.y = 0.01
            goal.position_tolerance.z = 0.01
            goal.orientation_tolerance.roll = 0.01
            goal.orientation_tolerance.pitch = 0.01
            goal.orientation_tolerance.yaw = 0.01
            self.client_absolute.send_goal(goal)
        else:
            # Error message
            rospy.logerr('%s: unable to execute service to keep pose, another service is running',
                         self.name)

            # Info to track which service is already running
            rospy.logwarn('%s: keepPose: %s',
                         self.name, self.init_keep_pose)
            rospy.logwarn('%s: trajectory: %s',
                         self.name, self.init_trajectory)
            rospy.logwarn('%s: goto: %s',
                         self.name, self.init_goto)
            rospy.logwarn('%s: keep_z: %s',
                         self.name, self.init_keep_z)

        return EmptyResponse()


    def disable_keep_position(self, req):
        """ This method disables keep position mode """
        if self.init_keep_pose:  # If there is a keep pose action running
            # Print message
            rospy.loginfo('%s: disable keep position', self.name)

            # Set captain flag to false
            self.init_keep_pose = False

            # Cancel the goal through actionlib server
            self.client_absolute.cancel_goal()
        else:
            rospy.logerr('%s: no keep position service active', self.name)

        return EmptyResponse()


    def goto_local(self, req):
        """ This mode is used to guide the AUV to a local position. Input is
            specified as meters north, meters east... """
        # Display requested position
        rospy.loginfo('%s: north: %s', self.name, req.north_lat)
        rospy.loginfo('%s: east: %s', self.name, req.east_lon)
        rospy.loginfo('%s: z: %s', self.name, req.z)
        rospy.loginfo('%s: altitude_mode: %s', self.name, req.altitude_mode)
        rospy.loginfo('%s: enable goto', self.name)

        # Process it through self.goto method
        return self.goto(req)


    def goto_local_block(self, req):
        """ This mode is used to guide the AUV to a local position. Input is
            specified as meters north, meters east... The captain blocks until
            reached """
        # Display requested position
        rospy.loginfo('%s: north: %s', self.name, req.north_lat)
        rospy.loginfo('%s: east: %s', self.name, req.east_lon)
        rospy.loginfo('%s: z: %s', self.name, req.z)
        rospy.loginfo('%s: altitude_mode: %s', self.name, req.altitude_mode)
        rospy.loginfo('%s: enable goto', self.name)

        # Process it through self.goto method
        return self.goto(req, True)


    def goto_relative(self, req):
        """ This mode is used to guide the AUV to a relative position. Input is
            specified as meters north, meters east... """
        # Display requested position
        rospy.loginfo('%s: north: %s', self.name, req.north_lat)
        rospy.loginfo('%s: east: %s', self.name, req.east_lon)
        rospy.loginfo('%s: z: %s', self.name, req.z)
        rospy.loginfo('%s: altitude_mode: %s', self.name, req.altitude_mode)
        rospy.loginfo('%s: enable goto', self.name)

        # Compute new position
        req.north_lat += self.nav.position.north
        req.east_lon += self.nav.position.east

        # Process it through self.goto method
        return self.goto(req)


    def goto_xy(self, req):
        """ This mode is used to guide the AUV to a relative position with
            respect to the body position. Input is specified as meters north,
            meters east... """
        # Compute new relative position from body position
        ori = self.nav.orientation.yaw
        north = (np.cos(ori) * req.north_lat) - (np.sin(ori) * req.east_lon)
        east = (np.sin(ori) * req.north_lat) + (np.cos(ori) * req.east_lon)

        # Display requested position
        rospy.loginfo('%s: north: %s', self.name, req.north_lat)
        rospy.loginfo('%s: east: %s', self.name, req.east_lon)
        rospy.loginfo('%s: z: %s', self.name, req.z)
        rospy.loginfo('%s: altitude_mode: %s', self.name, req.altitude_mode)
        rospy.loginfo('%s: enable goto', self.name)

        # Compute new position
        req.north_lat = self.nav.position.north + north
        req.east_lon = self.nav.position.east + east

        # Process it through self.goto method
        return self.goto(req)


    def goto_global(self, req):
        """ This mode is used to guide the AUV to a global position. Input is
            specified as latitude, longitude... """
        if self.trajectory.init_ned:  # If ned pose is initialized
            # Translate request to ned
            pose = self.trajectory.ned.geodetic2ned([req.north_lat,
                                                    req.east_lon,
                                                    0.0])

            # Show requested position
            rospy.loginfo('%s: lat: %s', self.name, req.north_lat)
            rospy.loginfo('%s: lon: %s', self.name, req.east_lon)
            rospy.loginfo('%s: north: %s', self.name, pose[NED.NORTH])
            rospy.loginfo('%s: east: %s', self.name, pose[NED.EAST])
            rospy.loginfo('%s: z: %s', self.name, req.z)
            rospy.loginfo('%s: altitude_mode: %s', self.name, req.altitude_mode)
            rospy.loginfo('%s: enable goto', self.name)

            # Redefine position in ned structure
            req.north_lat = pose[NED.NORTH]
            req.east_lon = pose[NED.EAST]

            # Process it through self.goto method
            return self.goto(req)

        else:
            rospy.logerr('%s: initialize NED first', self.name)

            return GotoResponse(False)


    def goto_global_block(self, req):
        """ This mode is used to guide the AUV to a global position. Input is
            specified as latitude, longitude... """
        if self.trajectory.init_ned:  # If ned pose is initialized
            # Translate request to ned
            pose = self.trajectory.ned.geodetic2ned([req.north_lat,
                                                    req.east_lon,
                                                    0.0])

            # Show requested position
            rospy.loginfo('%s: lat: %s', self.name, req.north_lat)
            rospy.loginfo('%s: lon: %s', self.name, req.east_lon)
            rospy.loginfo('%s: north: %s', self.name, pose[NED.NORTH])
            rospy.loginfo('%s: east: %s', self.name, pose[NED.EAST])
            rospy.loginfo('%s: z: %s', self.name, req.z)
            rospy.loginfo('%s: altitude_mode: %s', self.name, req.altitude_mode)
            rospy.loginfo('%s: enable goto', self.name)

            # Redefine position in ned structure
            req.north_lat = pose[NED.NORTH]
            req.east_lon = pose[NED.EAST]

            # Process it through self.goto method
            return self.goto(req, block = True)

        else:
            rospy.logerr('%s: initialize NED first', self.name)

            return GotoResponse(False)


    def goto(self, req, block = False):
        """ This mode is used to process a local or a global position req. Input
            is in meters """
        if (not self.init_trajectory and
             not self.init_keep_pose and
             not self.init_goto and
             not self.init_keep_z):  # If there is no other action running

            # Set captain flag to true
            self.init_goto = True

            # Fill a WorldWaypointReqGoal message
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = req.altitude_mode
            goal.position.north = req.north_lat
            goal.position.east = req.east_lon
            goal.position.depth = req.z
            goal.altitude = req.z
            goal.orientation.roll = 0.0
            goal.orientation.pitch = 0.0
            goal.orientation.yaw = 0.0

            goal.mode = 'waypoint'
            goal.disable_axis.x = False
            goal.disable_axis.y = True
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False

            # Set tolerance
            goal.position_tolerance.x = req.tolerance
            goal.position_tolerance.y = req.tolerance
            goal.position_tolerance.z = req.tolerance
            goal.orientation_tolerance.roll = req.tolerance
            goal.orientation_tolerance.pitch = req.tolerance
            goal.orientation_tolerance.yaw = req.tolerance

            # Process the goal through actionlib server
            self.client_absolute.send_goal(goal)

            # Start a thread
            t = Thread(target=self.wait_goto, args=())
            t.daemon = True  # Close this thread when captain exits
            t.start()
            if block:
                t.join()

            return GotoResponse(True)
        else:
            # Error message
            rospy.logerr('%s: impossible to run goto, another service is running',
                         self.name)

            # Info to track which service is already running
            rospy.logwarn('%s: keepPose: %s',
                         self.name, self.init_keep_pose)
            rospy.logwarn('%s: trajectory: %s',
                         self.name, self.init_trajectory)
            rospy.logwarn('%s: goto: %s',
                         self.name, self.init_goto)
            rospy.logwarn('%s: keep_z: %s',
                         self.name, self.init_keep_z)

            return GotoResponse(False)


    def wait_goto(self):
        """ Wait for goto result method. This is a thread started in goto
            method or in submerge method """
        # Wait actionlib server to answer and set the captain flag back to false
        self.client_absolute.wait_for_result()
        self.init_goto = False


    def disable_goto(self, req):
        """ This method disables goto mode """
        if self.init_goto:  # If there is a goto action running
            # Print message
            rospy.loginfo('%s: disable goto/surface', self.name)

            # Set captain flag to false
            self.init_goto = False

            # Cancel the goal through actionlib server
            self.client_absolute.cancel_goal()
        else:
            rospy.logerr('%s: no goto/surface service active', self.name)

        return EmptyResponse()


    def submerge(self, req):
        """ Submerge behavior. This is a service used to submerge the AUV
            to a given depth or altitude. The east/north position of the
            waypoint is the robot position when submerge service is called.
            This uses the move_mode waypoint """
        if ( not self.init_trajectory and
             not self.init_keep_pose and
             not self.init_goto and
             not self.init_keep_z):
            # If there is no other action running

            # Set captain flag of GOTO to true
            self.init_goto = True

            # Show message
            rospy.loginfo('%s: z: %s', self.name, req.z)
            rospy.loginfo('%s: altitude_mode: %s', self.name, req.altitude_mode)
            rospy.loginfo('%s: enable submerge', self.name)

            # Fill a goal message
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_MANUAL_OVERRIDE + 10 # This service is used mostly for emergencies!
            goal.altitude_mode = req.altitude_mode
            goal.position.north = self.nav.position.north
            goal.position.east = self.nav.position.east
            goal.position.depth = req.z
            goal.altitude = req.z
            goal.orientation.roll = 0.0
            goal.orientation.pitch = 0.0
            goal.orientation.yaw = 0.0

            goal.mode = 'waypoint'
            goal.disable_axis.x = True
            goal.disable_axis.y = True
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = True

            # Set tolerance
            tolerance = 4.0
            goal.position_tolerance.x = tolerance
            goal.position_tolerance.y = tolerance
            goal.position_tolerance.z = 1.0
            goal.orientation_tolerance.roll = tolerance
            goal.orientation_tolerance.pitch = tolerance
            goal.orientation_tolerance.yaw = tolerance

            # Process the goal through actionlib server
            self.client_absolute.send_goal(goal)

            # Start a wait_goto thread
            t = Thread(target=self.wait_goto, args=())
            t.daemon = True  # Close this thread when captain exits
            t.start()

            return SubmergeResponse(True)
        else:
            # Error message
            rospy.logerr('%s: impossible to submerge, another service is running',
                         self.name)

            # Info to track which service is already running
            rospy.logwarn('%s: keepPose: %s',
                         self.name, self.init_keep_pose)
            rospy.logwarn('%s: trajectory: %s',
                         self.name, self.init_trajectory)
            rospy.logwarn('%s: goto: %s',
                         self.name, self.init_goto)
            rospy.logwarn('%s: keep_z: %s',
                         self.name, self.init_keep_z)

            return SubmergeResponse(False)


    def pub_mission_status_timer(self, event):
        """ Publish mission status. This is a timer """
        # Publish mission status periodically
        self.pub_mission_status.publish(self.mission_status)
        self.diagnostic.setLevel(DiagnosticStatus.OK)


    def nav_goal(self, goal):
        req = GotoRequest()
        req.altitude_mode = False
        req.north_lat = goal.pose.position.x

        # TODO: Rotate PoseStamped goal to world frame
        listener = tf.TransformListener()
        try:
            listener.waitForTransform('/world', goal.header.frame_id, rospy.Time(), rospy.Duration(2.0))
            (trans, rot) = listener.lookupTransform('/world', goal.header.frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("%s: No Tf from world to %s found", self.name, goal.header.frame_id)
            return

        frameToWorld = np.array(tf.transformations.quaternion_matrix(rot))
        frameToWorld[0,3] = trans[0]
        frameToWorld[1,3] = trans[1]
        frameToWorld[2,3] = trans[2]

        point = np.array([[goal.pose.position.x],[goal.pose.position.y], [0], [1.0]])
        point_in_world = np.dot(frameToWorld, point)

        """if goal.header.frame_id == 'rviz':
            req.east_lon = -1.0 * goal.pose.position.y
        elif goal.header.frame_id == 'world':
            req.east_lon = goal.pose.position.y
        else :
            rospy.logerr("%s: Invalid frame_id. Set Fixed frame ans rviz or world", self.name)
            return -1
        """
        req.north_lat = point_in_world[0]
        req.east_lon = point_in_world[1]
        req.z = self.nav.position.depth
        req.tolerance = 1.5
        print 'Request:\n', req
        self.goto(req)


    def update_nav_sts(self, nav_sts):
        """ Navigation message callback """
        # Update self.nav
        self.nav = nav_sts

        # Check if NED origin has changed
        if self.last_origin_lat != self.nav.origin.latitude or \
           self.last_origin_lon != self.nav.origin.longitude:

            # If NED origin has changed, update it on the trajectory
            self.last_origin_lat = self.nav.origin.latitude
            self.last_origin_lon = self.nav.origin.longitude
            self.trajectory.initNed(nav_sts.origin.latitude,
                                    nav_sts.origin.longitude)

        # Update mission status current pose
        self.mission_status.current_north = nav_sts.position.north
        self.mission_status.current_east = nav_sts.position.east
        self.mission_status.current_altitude = nav_sts.altitude
        self.mission_status.current_depth = nav_sts.position.depth

        self.diagnostic.add("current_north", str(self.mission_status.current_north))
        self.diagnostic.add("current_east", str(self.mission_status.current_east))
        self.diagnostic.add("current_altitude", str(self.mission_status.current_altitude))
        self.diagnostic.add("current_depth", str(self.mission_status.current_depth))



    def stop(self, message=''):
        """ Cancel the current goal, called when shutdown """
        # Display a message
        rospy.loginfo("%s: cancel the current goal", self.name)

        # Set all captain flags to false
        self.init_trajectory = False
        self.init_keep_pose = False
        self.init_goto = False
        self.init_keep_z = False
        self.diagnostic.add("trajectory_enabled", "False")
        # Cancel goal through actionlib server
        self.client_absolute.cancel_goal()

def __delete_param__( param_name ):
        try:
            rospy.delete_param(param_name)
        except KeyError:
            pass #print param_name, " value not set"

if __name__ == '__main__':
    try:
        rospy.init_node('captain', log_level=rospy.INFO) #log_level=rospy.DEBUG)
        follow_trajectory = Captain(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
