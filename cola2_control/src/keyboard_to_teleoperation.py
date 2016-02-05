#! /usr/bin/env python
import rospy
from cola2_lib.JoystickBase import JoystickBase

class KeyboardToTeleoperation(JoystickBase):
    """ This class inherent from JoystickBase. It has to overload the 
    method update_joy(self, joy) that receives a sensor_msgs/Joy 
    message and fill the var self.joy_msg as described in the class
    JoystickBase. 
    From this class it is also possible to call services or anything 
    else reading the buttons in the update_joy method."""
    
    # BUTTONS DEFINITION:
    # [SPACE] [W] [S] [A] [D] [UP_ARROW][DOWN_ARROW][RIGTH_ARROW][LEFT_ARROW]...
    # [  0  ] [1] [2] [3] [4] [    5   ][    6     ][     7     ][    8     ]...
    # [ZERO ] [u+][u-][r-][r+][w-]      [w+]        [v+]         [v-]
        
    
    def __init__(self, name):
        """ Constructor """
        JoystickBase.__init__(self, name)
        # rospy.loginfo("%s: LogitechFX10 constructor", name)
        
        # To transform button into axis
        self.desired_vel = [0.0, 0.0, 0.0, 0.0] # u, v, w, r
        
        
    def update_joy(self, joy):
        """ Transform FX10 joy data into 12 axis data (pose + twist) 
        and sets the buttons that especify if position or velocity 
        commands are used in the teleoperation."""

        KEY_SPACE = 0
        KEY_W = 1
        KEY_S = 2
        KEY_A = 3
        KEY_D = 4
        KEY_UP = 5
        KEY_DOWN = 6
        KEY_RIGHT = 7
        KEY_LEFT = 8
        TWIST_U = 0
        TWIST_V = 1
        TWIST_W = 2
        TWIST_R = 3

        # rospy.loginfo("%s: Received:\n %s", self.name, joy)
        self.joy_msg.header = joy.header
        
        # Transform discrete axis into 'analog' axis 
        if(joy.buttons[KEY_W] == 1.0):
            self.desired_vel[TWIST_U] = self.desired_vel[TWIST_U] + 0.1
        elif(joy.buttons[KEY_S] == 1.0):
            self.desired_vel[TWIST_U] = self.desired_vel[TWIST_U] - 0.1
        elif(joy.buttons[KEY_A] == 1.0):
            self.desired_vel[TWIST_R] = self.desired_vel[TWIST_R] - 0.1
        elif(joy.buttons[KEY_D] == 1.0):
            self.desired_vel[TWIST_R] = self.desired_vel[TWIST_R] + 0.1
        elif(joy.buttons[KEY_UP] == 1.0):
            self.desired_vel[TWIST_W] = self.desired_vel[TWIST_W] - 0.1
        elif(joy.buttons[KEY_DOWN] == 1.0):
            self.desired_vel[TWIST_W] = self.desired_vel[TWIST_W] + 0.1
        elif(joy.buttons[KEY_LEFT] == 1.0):
            self.desired_vel[TWIST_V] = self.desired_vel[TWIST_V] - 0.1
        elif(joy.buttons[KEY_RIGHT] == 1.0):
            self.desired_vel[TWIST_V] = self.desired_vel[TWIST_V] + 0.1
        elif(joy.buttons[KEY_SPACE] == 1.0):
            self.desired_vel = [0.0, 0.0, 0.0, 0.0]
        
        # Saturate values between -1 and 1
        for i in range(4):
            if self.desired_vel[i] > 1.0:
                self.desired_vel[i] = 1.0
            elif self.desired_vel[i] < -1.0:
                self.desired_vel[i] = -1.0
        
        # Copy values into teleoperation format
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_U] = self.desired_vel[TWIST_U]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_V] = self.desired_vel[TWIST_V]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_W] = self.desired_vel[TWIST_W]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_R] = self.desired_vel[TWIST_R]
        
                
if __name__ == '__main__':
    """ Initialize the keyboard_to_teleoperation node. """
    try:
        rospy.init_node('keyboard_to_teleoperation')
        map_ack = KeyboardToTeleoperation(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
