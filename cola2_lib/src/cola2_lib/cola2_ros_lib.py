#!/usr/bin/env python
import rospy
from auv_msgs.msg import BodyVelocityReq, GoalDescriptor
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest


# Simple service for enabling/disabling a node. See a use case in
# enrics_robot_code (stack)/cirs_cpp (package)/cliff_following_bh.py
class Enabler:
    def __init__(self):
        self.enabled = False

    def enable(self, req):
        self.enabled = True
        return EmptyResponse()

    def disable(self, req):
        self.enabled = False
        return EmptyResponse()

    def setEnabled(self, e):
        self.enabled = e

    def isEnabled(self):
        return self.enabled


# Empty config object, comes handy sometimes
class Config(object): pass


# getRosParams:
#   obj: object for which to set its attributes
#   params: dictionary of {'key': 'value'} pairs defining the attribute
#      name that'll be set and the name of the ROS param that'll be
#      assigned to it
#   [node_name]: optional ROS node name of the node for which the params
#      are retrieved
def getRosParams(obj, params, node_name=None):
    valid_config = True
    for key in params:
        if rospy.has_param(params[key]):
            param_value = rospy.get_param(params[key])
            setattr(obj, key, param_value)
        else:
            valid_config = False
            if node_name == None:
                rospy.logfatal(params[key] + " parameter not found")
            else:
                rospy.logfatal(node_name + ": " + params[key] + " parameter not found")
    return valid_config


# getRosParamsWithDefaults:
#   obj: object for which to set its attributes
#   params: dictionary of
#      {'attr_name': {'name': 'actual_name', default: default_value}
#      pairs defining the attribute name that'll be set, the name of the
#      ROS param that'll be assigned to it and its default value if
#      the ROS param is not found on the param server
#   [node_name]: optional ROS node name of the node for which the params
#      are retrieved
def getRosParamsWithDefaults(obj, params, node_name=None):
    valid_config = True
    for key in params:
        if rospy.has_param(params[key]['name']):
            param_value = rospy.get_param(params[key]['name'])
            setattr(obj, key, param_value)
        else:
            valid_config = False
            setattr(obj, key, params[key]['default'])
            if node_name == None:
                rospy.logfatal(params[key]['name'] + " param not found")
            else:
                rospy.logfatal(node_name + ": " + params[key]['name'] + " param not found")
    return valid_config


def requestBodyVelocity(vx, vy, vz, vyaw, pub):
    # Function for steering the robot at the low level

    #print ("(vx,vy,vz,vyaw) = ", vx,vy,vz,vyaw)

    # Build BodyVelocityReq message according to params
    message = BodyVelocityReq()

    message.header.stamp = rospy.Time.now()

    message.goal.priority = GoalDescriptor.PRIORITY_AVOID_OBSTACLE
    message.goal.requester = rospy.get_name()

    message.twist.linear.x = vx
    message.twist.linear.y = vy
    message.twist.linear.z = vz
    message.twist.angular.z = vyaw

    message.disable_axis.x = False
    message.disable_axis.y = False
    message.disable_axis.z = False
    message.disable_axis.roll = True
    message.disable_axis.pitch = True
    message.disable_axis.yaw = False

    # Publish built message
    rospy.loginfo(rospy.get_name()+": publishing BodyVelocityReq message")
    pub.publish(message)
