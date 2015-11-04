#!/usr/bin/env python
"""@@Recovery actions node is used to handle requests for recovery actions coming
from any node@@"""

"""
Created on Mar 25 2013
Modified 11/2015
@author: narcis palomeras
"""

# ROS imports
import roslib
import rospy

from std_srvs.srv import Empty, EmptyRequest
from cola2_msgs.srv import Submerge, SubmergeRequest
from cola2_msgs.srv import RecoveryAction, RecoveryActionRequest, RecoveryActionResponse
from cola2_msgs.msg import ThrustersData
from cola2_lib import cola2_ros_lib


class RecoveryActions(object):
    """ This class is able to handle recovery requests coming from all the
        nodes """

    def __init__(self, name):
        """ Init the class """
        # Save node name
        self.name = name

        # Get config
        self.get_config()

        # Create publisher
        self.pub_thrusters = rospy.Publisher("/cola2_control/thrusters_data",
                                             ThrustersData,
                                             queue_size = 2)

        # Init service clients
        rospy.loginfo("%s: waiting for services", self.name)
        self.captain_clients = True
        try:
            rospy.wait_for_service('/cola2_control/disable_trajectory', 20)
            self.abort_mission_srv = rospy.ServiceProxy(
                                '/cola2_control/disable_trajectory', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: disable trajectory service not available!", self.name)

        try:
            rospy.wait_for_service('/cola2_control/disable_keep_position', 2)
            self.abort_keep_pose_srv = rospy.ServiceProxy(
                                '/cola2_control/disable_keep_position', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: disable keep position service not available!", self.name)

        try:
            rospy.wait_for_service('/cola2_control/disable_goto', 2)
            self.abort_goto_srv = rospy.ServiceProxy(
                                '/cola2_control/disable_goto', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: disable goto service not available!", self.name)

        try:
            rospy.wait_for_service('/cola2_control/submerge', 2)
            self.surface_srv = rospy.ServiceProxy(
                                '/cola2_control/submerge', Submerge)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: submerge service not available!", self.name)

        if not self.captain_clients:
            self.no_captain_clients_timer = rospy.Timer(rospy.Duration(0.4), self.no_captain_clients_message)

        try:
            rospy.wait_for_service('/cola2_control/disable_thrusters', 20)
            self.abort_thrusters_srv = rospy.ServiceProxy(
                                '/cola2_control/disable_thrusters', Empty)
        except rospy.exceptions.ROSException:
            self.no_disable_thrusters_service_timer = rospy.Timer(rospy.Duration(0.4), self.no_disable_thrusters_message)

        # Create service
        self.recovery_srv = rospy.Service('/cola2_safety/recovery_action',
                                        RecoveryAction,
                                        self.recovery_action_srv)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def recovery_action_srv(self, req):
        """ Callback of recovery action service """
        rospy.loginfo('%s: received recovery action', self.name)
        self.recovery_action(req.error_level)
        ret = RecoveryActionResponse()
        ret.attempted = True
        return ret


    def recovery_action(self, error):
        """ This method calls the appropiate method to handle the input code """
        if error == RecoveryActionRequest.INFORMATIVE:
            rospy.loginfo("%s: recovery action %s: INFORMATIVE",
                          self.name, error)
            # TODO: send message through modem?
        elif error == RecoveryActionRequest.ABORT_MISSION:
            rospy.loginfo("%s: recovery action %s: ABORT_MISSION",
                          self.name, error)
            self.abort_mission()
        elif error == RecoveryActionRequest.ABORT_AND_SURFACE:
            rospy.loginfo("%s: recovery action %s: ABORT_AND_SURFACE",
                          self.name, error)
            self.abort_mission()
            self.surface()
        elif error == RecoveryActionRequest.EMERGENCY_SURFACE:
            rospy.loginfo("%s: recovery action %s: EMERGENCY_SURFACE",
                          self.name, error)
            self.abort_mission()
            self.emergency_surface()
        elif error == RecoveryActionRequest.STOP_THRUSTERS:
            rospy.loginfo("%s: recovery action %s: STOP_THRUSTERS",
                          self.name, error)
            # Disable thrusters
            try:
                self.abort_thrusters_srv(EmptyRequest())
            except rospy.exceptions.ROSException:
                rospy.logerr('%s: error disabling thrusters', self.name)
        else:
            rospy.loginfo("%s: recovery action %s: INVALID ERROR CODE",
                          self.name, error)


    def abort_mission(self):
        """ This method handles abort mission """
        rospy.loginfo("%s: abort mission", self.name)
        try:
            self.abort_mission_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error aborting the mission', self.name)
            
        try:
            self.abort_goto_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error aborting the goto', self.name)

        try:
            self.abort_keep_pose_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error aborting the keep pose', self.name)



    def surface(self):
        """ This method handles surface recovery action """
        rospy.loginfo("%s: surface", self.name)
        try:
            surface = SubmergeRequest()
            surface.z = self.controlled_surface_depth
            surface.altitude_mode = False
            self.surface_srv(surface)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error surfacing the vehicle', self.name)


    def emergency_surface(self):
        """ This method handles an emergency surface """
        rospy.loginfo("%s: emergency surface", self.name)
        try:
            self.abort_thrusters_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error disabling thrusters', self.name)

        r = rospy.Rate(10)
        thrusters = ThrustersData()
        thrusters.header.frame_id = self.frame_id
        while True:
            thrusters.header.stamp = rospy.Time.now()
            thrusters.setpoints = self.emergency_surface_setpoints
            self.pub_thrusters.publish(thrusters)
            r.sleep()


    def get_config(self):
        """ Get config from param server """
        param_dict = {'frame_id': 'recovery_actions/frame_id',
                      'emergency_surface_setpoints': 'recovery_actions/emergency_surface_setpoints',
                      'controlled_surface_depth': 'recovery_actions/controlled_surface_depth'}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            self.bad_config_timer = rospy.Timer(rospy.Duration(0.4), self.bad_config_message)


    def bad_config_message(self, event):
        """ Timer to show an error if loading parameters failed """
        rospy.logfatal('%s: bad parameters in param server!', self.name)


    def no_disable_thrusters_message(self, event):
        """ Timer to show an error in disable thrusters service """
        rospy.logfatal('%s: error creating client to disable thrusters', self.name)


    def no_captain_clients_message(self, event):
        """ Timer to show an error if unavailable captain service """
        rospy.logfatal('%s: error creating some captain clients', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('recovery_actions')
        recovery_actions = RecoveryActions(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
