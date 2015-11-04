#! /usr/bin/env python

#  map_ack.py
#  Created on: 19/04/2013
#  Modified 11/2015
#  Author: narcis & eduard

"""@@This node is used to join all input devices, such as keyboards and joystics.
Once joined, the map_ack node publishes a message. This node mainly interacts with
the teleoperation node. When teleoperating the without cable, map_ack should be
run outside the robot computer.@@"""

# ROS imports
import rospy

# Import messages
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# More imports
import numpy as np
from cola2_lib import cola2_lib, cola2_ros_lib


class LogitechFX10:
    """ This class is required to transform the joy messages that comes from the
        Logitech F710/F510 to the message required by the teleoperation node """

    def __init__(self, name):
        """ Constructor """
        self.name = name
        self.joy_msg = Joy()
        self.joy_msg.axes = [0]*12
        self.joy_msg.buttons = [0]*15 # Why 15 instead of 12 ???
        
        self.up_down = 0.0
        self.left_right = 0.0
        
        # Create publisher
        self.pub_map_ack_data = rospy.Publisher("/cola2_control/map_ack_data", 
                                                Joy,
                                                queue_size = 2)
                                                
        self.pub_map_ack_ack_teleop = rospy.Publisher("/cola2_control/map_ack_ack", 
                                                      String,
                                                      queue_size = 2)

        # Create subscriber
        rospy.Subscriber("/cola2_control/map_ack_ok",
                         String,
                         self.update_ack,
                         queue_size = 4)
                         
        rospy.Subscriber("/joy",
                         Joy,
                         self.update_joy,
                         queue_size = 4)
                         
        # Timer for the publish method
        rospy.Timer(rospy.Duration(0.1), self.iterate)
        
                         
    def update_ack(self, ack):
        """ Ack safety method """
        ack_list = ack.data.split()
        if len(ack_list) == 2 and ack_list[1] == 'ok':
            seq = int(ack_list[0]) + 1
            self.pub_map_ack_ack_teleop.publish(str(seq) + " ack")
        else:
            rospy.logerr("%s: received invalid teleoperation heart beat!",
                                                                     self.name)


    def update_joy(self, joy):
        """ Transform FX10 joy data into 12 axis data (pose + twist) """
        # print "Received:\n", joy
        # left joy Up -> +1
        # left joy Down -> -1
        # left joy right -> -0
        # left joy left -> +0
        # right joy Up -> +4
        # right joy Down -> -4
        # right joy right -> -3
        # right joy left -> +3
        # cross up_down --> 7
        # cross left_right --> 6
        
        self.joy_msg.header = joy.header
        
        # Transform discrete axis (creueta) to two 'analog' axis to control
        # depth and yaw in position.
        
        # up-down (depth control pose)
        if(joy.axes[7]== -1.0):
            self.up_down = self.up_down + 0.1
            if self.up_down > 1.0:
                self.up_down = 1.0
        elif(joy.axes[7]==1.0):
            self.up_down = self.up_down - 0.1
            if self.up_down < -1.0:
                self.up_down = -1.0
                
        # left-right (yaw control pose)
        if(joy.axes[6]== -1.0):
            self.left_right = self.left_right + 0.1
            if self.left_right > 1.0:
                self.left_right = 1.0
        elif(joy.axes[6]==1.0):
            self.left_right = self.left_right - 0.1
            if self.left_right < -1.0:
                self.left_right = -1.0
                
        # z
        self.joy_msg.axes[2] = self.up_down
        # yaw
        self.joy_msg.axes[5] = self.left_right
        
        
        # u
        self.joy_msg.axes[6] = joy.axes[4]
        # v
        self.joy_msg.axes[7] = -joy.axes[3]
        # w
        self.joy_msg.axes[8] = -joy.axes[1]
        # r
        self.joy_msg.axes[11] = -joy.axes[0]


        # We always publish the desired pose and the desired twist. 
        # However, using the buttons we decide which ones we use
         
        # enable/disable z control position
        self.joy_msg.buttons[3] = joy.buttons[0]
        self.joy_msg.buttons[9] = joy.buttons[3]
        if joy.buttons[0] == 1.0: 
            self.up_down = 0.0

        # enable/disable yaw control position
        self.joy_msg.buttons[6] = joy.buttons[1]
        self.joy_msg.buttons[12] = joy.buttons[2]
        if joy.buttons[1] == 1.0: 
            self.left_right = 0.0
            
        # rospy.loginfo("%s: Joy msg:\n %s", self.name, self.joy_msg)


    def iterate(self, event):
        """ This method is a callback of a timer. This is used to publish the
            output joy message """
            
        # Publish message
        self.pub_map_ack_data.publish(self.joy_msg)

        # Reset buttons


if __name__ == '__main__':
    try:
        rospy.init_node('map_ack')
        map_ack = LogitechFX10(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
