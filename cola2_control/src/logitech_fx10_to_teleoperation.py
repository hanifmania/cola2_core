#! /usr/bin/env python
import rospy
from cola2_lib import JoystickBase
from JoystickBase import JoystickBase

class LogitechFX10(JoystickBase):
    """ This class inherent from JoystickBase. It has to overload the 
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

        # rospy.loginfo("%s: Received:\n %s", self.name, joy)
        
        # JOYSTICK BUTTONS DEFINITION:
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
        
        # Transform discrete axis (cross) to two 'analog' axis to control
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
                
        self.joy_msg.axes[JoystickBase.AXIS_POSE_Z] = self.up_down
        self.joy_msg.axes[JoystickBase.AXIS_POSE_YAW] = self.left_right
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_U] = joy.axes[4]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_V] = -joy.axes[3]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_W] = -joy.axes[1]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_R] = -joy.axes[0]

        # We always publish the desired pose and the desired twist. 
        # However, using the buttons we decide which ones we use
         
        # enable/disable z control position
        self.joy_msg.buttons[JoystickBase.BUTTON_POSE_Z] = joy.buttons[0]
        self.joy_msg.buttons[JoystickBase.BUTTON_TWIST_W] = joy.buttons[3]
        if joy.buttons[0] == 1.0: 
            self.up_down = 0.0
            rospy.loginfo("%s: Reset up_down counter", self.name)

        # enable/disable yaw control position
        self.joy_msg.buttons[JoystickBase.BUTTON_POSE_YAW] = joy.buttons[1]
        self.joy_msg.buttons[JoystickBase.BUTTON_TWIST_R] = joy.buttons[2]
        if joy.buttons[1] == 1.0: 
            self.left_right = 0.0
            rospy.loginfo("%s: Reset left_right counter", self.name)

if __name__ == '__main__':
    """ Initialize the logitech_fx10 node. """
    try:
        rospy.init_node('logitech_fx10')
        map_ack = LogitechFX10(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
