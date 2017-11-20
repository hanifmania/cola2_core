#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


import rospy
from cola2_lib.JoystickBase import JoystickBase

class LogitechFX10(JoystickBase):
    """ This class inherits from JoystickBase. It has to overload the 
    method update_joy(self, joy) that receives a sensor_msgs/Joy 
    message and fill the var self.joy_msg as described in the class
    JoystickBase. 
    From this class it is also possible to call services or anything 
    else reading the buttons in the update_joy method."""
    
    
    def __init__(self, name):
        """ Constructor """
        JoystickBase.__init__(self, name)
        # rospy.loginfo("%s: LogitechFX10 constructor", name)
        
        # To transform button into axis
        self.up_down = 0.0
        self.left_right = 0.0
        
        
    def update_joy(self, joy):
        """ Transform FX10 joy data into 12 axis data (pose + twist) 
        and sets the buttons that especify if position or velocity 
        commands are used in the teleoperation."""

        # JOYSTICK  DEFINITION:
        LEFT_JOY_HORIZONTAL = 0     # LEFT+, RIGHT-
        LEFT_JOY_VERTICAL = 1       # UP+, DOWN-
        LEFT_TRIGGER = 2            # NOT PRESS 1, PRESS -1
        RIGHT_JOY_HORIZONTAL = 3    # LEFT+, RIGHT-
        RIGHT_JOY_VERTICAL = 4      # UP+, DOWN-
        RIGHT_TRIGGER = 5           # NOT PRESS 1, PRESS -1
        CROSS_HORIZONTAL = 6        # LEFT+, RIGHT-
        CROSS_VERTICAL = 7          # UP+, DOWN-
        BUTTON_A = 0
        BUTTON_B = 1
        BUTTON_X = 2
        BUTTON_Y = 3
        BUTTON_LEFT = 4
        BUTTON_RIGHT = 5
        BUTTON_BACK = 6
        BUTTON_START = 7
        BUTTON_LOGITECH = 8 
        BUTTON_LEFT_JOY = 9
        BUTTON_RIGHT_JOY = 10
        MOVE_UP = 1
        MOVE_DOWN = -1
        MOVE_LEFT = 1
        MOVE_RIGHT = -1
        
        self.joy_msg.header = joy.header
        
        # Transform discrete axis (cross) to two 'analog' axis to control
        # depth and yaw in position.
        
        # up-down (depth control pose)
        if(joy.axes[CROSS_VERTICAL] == MOVE_DOWN):
            self.up_down = self.up_down + 0.1
            if self.up_down > 1.0:
                self.up_down = 1.0
        elif(joy.axes[CROSS_VERTICAL] == MOVE_UP):
            self.up_down = self.up_down - 0.1
            if self.up_down < -1.0:
                self.up_down = -1.0
                
        # left-right (yaw control pose)
        if(joy.axes[CROSS_HORIZONTAL] == MOVE_RIGHT):
            self.left_right = self.left_right + 0.1
            if self.left_right > 1.0:
                self.left_right = 1.0
        elif(joy.axes[CROSS_HORIZONTAL] == MOVE_LEFT):
            self.left_right = self.left_right - 0.1
            if self.left_right < -1.0:
                self.left_right = -1.0
                
        self.joy_msg.axes[JoystickBase.AXIS_POSE_Z] = self.up_down
        self.joy_msg.axes[JoystickBase.AXIS_POSE_YAW] = self.left_right
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_U] = joy.axes[RIGHT_JOY_VERTICAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_V] = -joy.axes[RIGHT_JOY_HORIZONTAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_W] = -joy.axes[LEFT_JOY_VERTICAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_R] = -joy.axes[LEFT_JOY_HORIZONTAL]

        # We always publish the desired pose and the desired twist. 
        # However, using the buttons we decide which ones we use
         
        # enable/disable z control position
        self.joy_msg.buttons[JoystickBase.BUTTON_POSE_Z] = joy.buttons[BUTTON_A]
        self.joy_msg.buttons[JoystickBase.BUTTON_TWIST_W] = joy.buttons[BUTTON_Y]
        if joy.buttons[BUTTON_A] == 1.0: 
            self.up_down = 0.0
            rospy.loginfo("%s: Reset up_down counter", self.name)

        # enable/disable yaw control position
        self.joy_msg.buttons[JoystickBase.BUTTON_POSE_YAW] = joy.buttons[BUTTON_B]
        self.joy_msg.buttons[JoystickBase.BUTTON_TWIST_R] = joy.buttons[BUTTON_X]
        if joy.buttons[BUTTON_B] == 1.0: 
            self.left_right = 0.0
            rospy.loginfo("%s: Reset left_right counter", self.name)

        print 'JoystickBase.BUTTON_POSE_Z: ', JoystickBase.BUTTON_POSE_Z
if __name__ == '__main__':
    """ Initialize the logitech_fx10 node. """
    try:
        rospy.init_node('logitech_fx10')
        map_ack = LogitechFX10(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
