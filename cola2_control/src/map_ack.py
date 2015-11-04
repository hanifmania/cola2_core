#! /usr/bin/env python

#  map_ack.py
#  Created on: 19/04/2013
#  Author: narcis & eduard

"""@@This node is used to join all input devices, such as keyboards and joystics.
Once joined, the map_ack node publishes a message. This node mainly interacts with
the teleoperation node. When teleoperating the without cable, map_ack should be
run outside the robot computer.@@"""

# ROS imports
import rospy
from std_srvs.srv import Empty

# Import messages
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# More imports
import numpy as np
from cola2_lib import cola2_lib, cola2_ros_lib


ALLOW_NEGATIVE_DEPTH = False  # Flag used for heave pose dof


class MapAck:
    """ This class is required to join all joy messages that come from all
        devices and to send the joined joy message to the teleoperation node """

    def __init__(self, name):
        """ Constructor """
        self.name = name

        # Get config
        self.get_config()

        # Initialize some vars
        self.fake_axes = np.matrix(np.zeros((self.n_output_axes, 1)))
        self.last_fake_axes_increment = np.matrix(np.zeros((self.n_output_axes, 1)))
        self.map_ack_data_msg = Joy()
        self.map_ack_data_msg.axes = np.matrix(np.zeros((self.n_output_axes, 1)))
        self.map_ack_data_msg.buttons = np.matrix(np.zeros((self.n_output_buttons, 1)))

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

        # Create slots subscribers
        if self.slotA[1] != '':
            rospy.Subscriber(self.slotA[1], Joy, self.update_slotA, queue_size = 4)
            rospy.loginfo("%s: subscribed to %s", self.name, self.slotA[0])
        if self.slotB[1] != '':
            rospy.Subscriber(self.slotB[1], Joy, self.update_slotB, queue_size = 4)
            rospy.loginfo("%s: subscribed to %s", self.name, self.slotB[0])
        if self.slotC[1] != '':
            rospy.Subscriber(self.slotC[1], Joy, self.update_slotC, queue_size = 4)
            rospy.loginfo("%s: subscribed to %s", self.name, self.slotC[0])
        if self.slotD[1] != '':
            rospy.Subscriber(self.slotD[1], Joy, self.update_slotD, queue_size = 4)
            rospy.loginfo("%s: subscribed to %s", self.name, self.slotD[0])
        if self.slotE[1] != '':
            rospy.Subscriber(self.slotE[1], Joy, self.update_slotE, queue_size = 4)
            rospy.loginfo("%s: subscribed to %s", self.name, self.slotE[0])

        # Timer for the publish method
        rospy.Timer(rospy.Duration(1.0/self.rate), self.iterate)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def update_ack(self, ack):
        """ Ack safety method """
        ack_list = ack.data.split()
        if len(ack_list) == 2 and ack_list[1] == 'ok':
            seq = int(ack_list[0]) + 1
            self.pub_map_ack_ack_teleop.publish(str(seq) + " ack")
        else:
            rospy.logerr("%s: received invalid teleoperation heart beat!",
                                                                     self.name)

    def call_service_from_joystick(self, data):        
        if data.buttons[self.service_button_1] == 1:
            rospy.wait_for_service(self.service_call_1, 5)
            service_button = rospy.ServiceProxy(self.service_call_1, Empty)
            try:
              service_button()
            except rospy.ServiceException as exc:
              print("Service did not process request: " + str(exc))
        if data.buttons[self.service_button_2] == 1:
            rospy.wait_for_service(self.service_call_2, 5)
            service_button = rospy.ServiceProxy(self.service_call_2, Empty)
            try:
              service_button()
            except rospy.ServiceException as exc:
              print("Service did not process request: " + str(exc))



    def update_slotA(self, data):
        """ SlotA callback """
        self.compute_device(data,
                           vars(self)[self.slotA[0] + '_n_axes'],
                           vars(self)[self.slotA[0] + '_n_buttons'],
                           vars(self)[self.slotA[0] + '_axes_to_output_axes'],
                           vars(self)[self.slotA[0] + '_axes_to_shadow_buttons'],
                           vars(self)[self.slotA[0] + '_shadow_buttons_to_output_axes'],
                           vars(self)[self.slotA[0] + '_buttons_to_shadow_buttons'])
                          
        # If slot_A = joystick call service when a button is pressed
        if self.slotA[0] == 'joystick':
            self.call_service_from_joystick(data)


    def update_slotB(self, data):
        """ SlotB callback """
        self.compute_device(data,
                           vars(self)[self.slotB[0] + '_n_axes'],
                           vars(self)[self.slotB[0] + '_n_buttons'],
                           vars(self)[self.slotB[0] + '_axes_to_output_axes'],
                           vars(self)[self.slotB[0] + '_axes_to_shadow_buttons'],
                           vars(self)[self.slotB[0] + '_shadow_buttons_to_output_axes'],
                           vars(self)[self.slotB[0] + '_buttons_to_shadow_buttons'])
                           
        # If slot_B = joystick call service when a button is pressed
        if self.slotB[0] == 'joystick':
            self.call_service_from_joystick(data)


    def update_slotC(self, data):
        """ SlotC callback """
        self.compute_device(data,
                           vars(self)[self.slotC[0] + '_n_axes'],
                           vars(self)[self.slotC[0] + '_n_buttons'],
                           vars(self)[self.slotC[0] + '_axes_to_output_axes'],
                           vars(self)[self.slotC[0] + '_axes_to_shadow_buttons'],
                           vars(self)[self.slotC[0] + '_shadow_buttons_to_output_axes'],
                           vars(self)[self.slotC[0] + '_buttons_to_shadow_buttons'])

                           
        # If slot_C = joystick call service when a button is pressed
        if self.slotC[0] == 'joystick':
            self.call_service_from_joystick(data)


    def update_slotD(self, data):
        """ SlotD callback """
        self.compute_device(data,
                           vars(self)[self.slotD[0] + '_n_axes'],
                           vars(self)[self.slotD[0] + '_n_buttons'],
                           vars(self)[self.slotD[0] + '_axes_to_output_axes'],
                           vars(self)[self.slotD[0] + '_axes_to_shadow_buttons'],
                           vars(self)[self.slotD[0] + '_shadow_buttons_to_output_axes'],
                           vars(self)[self.slotD[0] + '_buttons_to_shadow_buttons'])
                           
        # If slot_C = joystick call service when a button is pressed
        if self.slotC[0] == 'joystick':
            self.call_service_from_joystick(data)


    def update_slotE(self, data):
        """ SlotE callback """
        self.compute_device(data,
                           vars(self)[self.slotE[0] + '_n_axes'],
                           vars(self)[self.slotE[0] + '_n_buttons'],
                           vars(self)[self.slotE[0] + '_axes_to_output_axes'],
                           vars(self)[self.slotE[0] + '_axes_to_shadow_buttons'],
                           vars(self)[self.slotE[0] + '_shadow_buttons_to_output_axes'],
                           vars(self)[self.slotE[0] + '_buttons_to_shadow_buttons'])

                           
        # If slot_E = joystick call service when a button is pressed
        if self.slotE[0] == 'joystick':
            self.call_service_from_joystick(data)



    def compute_device(self, data, device_axes, device_buttons, device_AA,
                      device_ASB, device_SBA, device_BSB):
        """ This method is used to compute all matrices products and store
            the result to a joy message """
        # Convert input data to column matrix
        input_axes = np.matrix(data.axes).ravel().transpose()
        input_buttons = np.matrix(data.buttons).ravel().transpose()

        # Compute shadow buttons
        shadow_buttons = np.matrix(np.zeros((self.n_shadow_buttons, 1)))
        if (device_axes > 0):
            shadow_buttons = shadow_buttons + device_ASB * input_axes
        if (device_buttons > 0):
            shadow_buttons = shadow_buttons + device_BSB * input_buttons

        # Prefilter shadow_buttons
        shadow_buttons = shadow_buttons.clip(min=0, max=1)

        # Compute axes from buttons
        if (device_buttons > 0):
            # Here we use shadow buttons!
            fake_axes_increment = device_SBA * shadow_buttons

            # Not to increase at each message if fake buttons doesn't change
            if (not(fake_axes_increment == self.last_fake_axes_increment).all()):
                self.fake_axes = self.fake_axes + fake_axes_increment
                self.fake_axes[0, 0] = cola2_lib.saturateValueFloat(self.fake_axes[0, 0], 1.0)
                self.fake_axes[1, 0] = cola2_lib.saturateValueFloat(self.fake_axes[1, 0], 1.0)
                self.fake_axes[2, 0] = cola2_lib.saturateValueFloat(self.fake_axes[2, 0], 1.0)
                if not ALLOW_NEGATIVE_DEPTH:  # Saturate depth pose from 0 to 1
                    if (self.fake_axes[2, 0] < 0):
                        self.fake_axes[2, 0] = 0.0
                self.fake_axes[6, 0] = cola2_lib.saturateValueFloat(self.fake_axes[6, 0], 1.0)
                self.fake_axes[7, 0] = cola2_lib.saturateValueFloat(self.fake_axes[7, 0], 1.0)
                self.fake_axes[8, 0] = cola2_lib.saturateValueFloat(self.fake_axes[8, 0], 1.0)
                self.fake_axes[9, 0] = cola2_lib.saturateValueFloat(self.fake_axes[9, 0], 1.0)
                self.fake_axes[10, 0] = cola2_lib.saturateValueFloat(self.fake_axes[10, 0], 1.0)
                self.fake_axes[11, 0] = cola2_lib.saturateValueFloat(self.fake_axes[11, 0], 1.0)
            self.last_fake_axes_increment = fake_axes_increment

        # fake_axes velocities to zero if stop button pressed
        if (shadow_buttons[0] > 0):
            self.fake_axes[6:12, 0] = np.matrix(np.zeros((6, 1)))

        # Compute axes
        output_axes = self.fake_axes
        if (device_axes > 0):
            output_axes = device_AA * input_axes + self.fake_axes

        # Angles from -1 to 1
        output_axes[3, 0] = __special_wrap_angle__(output_axes[3, 0])
        output_axes[4, 0] = __special_wrap_angle__(output_axes[4, 0])
        output_axes[5, 0] = __special_wrap_angle__(output_axes[5, 0])

        # Save axes and buttons, filtered (clipped)
        self.map_ack_data_msg.axes = output_axes.clip(min=-1, max=1)
        shadow_buttons = np.matrix(shadow_buttons.clip(min=0, max=1)).ravel().transpose()
        self.map_ack_data_msg.buttons = (self.map_ack_data_msg.buttons + self.shadow_buttons_to_output_buttons * shadow_buttons).clip(min=0, max=1)


    def iterate(self, event):
        """ This method is a callback of a timer. This is used to publish the
            output joy message """
        # Publish message
        self.map_ack_data_msg.header.stamp = rospy.Time().now()
        self.pub_map_ack_data.publish(self.map_ack_data_msg)

        # Reset buttons
        self.map_ack_data_msg.buttons = np.matrix(np.zeros((self.n_output_buttons, 1)))


    def get_config(self):
        """ Get config from param server """
        # Get common parameters
        param_dict = {'rate': 'map_ack/rate',
                      'service_button_1': 'map_ack/service_button_1',
                      'service_button_2': 'map_ack/service_button_2',
                      'service_call_1': 'map_ack/service_call_1',
                      'service_call_2': 'map_ack/service_call_2',
                      'n_output_axes': 'map_ack/n_output_axes',
                      'n_output_buttons': 'map_ack/n_output_buttons',
                      'n_shadow_buttons': 'map_ack/n_shadow_buttons',
                      'slotA': 'map_ack/slotA',
                      'slotB': 'map_ack/slotB',
                      'slotC': 'map_ack/slotC',
                      'slotD': 'map_ack/slotD',
                      'slotE': 'map_ack/slotE',
                      'shadow_buttons_to_output_buttons': 'map_ack/shadow_buttons_to_output_buttons'}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way

        self.shadow_buttons_to_output_buttons = np.matrix(self.shadow_buttons_to_output_buttons).reshape(self.n_output_buttons, self.n_shadow_buttons)

        # Generate a devices list
        devices = []
        if self.slotA[0] != '':
            devices.append(self.slotA[0])
        if self.slotB[0] != '':
            devices.append(self.slotB[0])
        if self.slotC[0] != '':
            devices.append(self.slotC[0])
        if self.slotD[0] != '':
            devices.append(self.slotD[0])
        if self.slotE[0] != '':
            devices.append(self.slotE[0])

        # Read devices
        for i in range(len(devices)):  # For each defined device
            # Get the name
            name = devices[i]

            # Get parameters from current device
            param_dict = {name + '_n_axes': 'map_ack/' + name + '/n_axes',
                          name + '_n_buttons': 'map_ack/' + name + '/n_buttons',
                          name + '_axes_to_output_axes': 'map_ack/' + name + '/axes_to_output_axes',
                          name + '_axes_to_shadow_buttons': 'map_ack/' + name + '/axes_to_shadow_buttons',
                          name + '_buttons_to_shadow_buttons': 'map_ack/' + name + '/buttons_to_shadow_buttons',
                          name + '_shadow_buttons_to_output_axes': 'map_ack/' + name + '/shadow_buttons_to_output_axes'}
            cola2_ros_lib.getRosParams(self, param_dict, 'Device ' + name)

            # Reshape matrices of the current device
            if (vars(self)[name + '_n_axes'] > 0):
                vars(self)[name + '_axes_to_output_axes'] = np.matrix(vars(self)[name + '_axes_to_output_axes']).reshape(self.n_output_axes, vars(self)[name + '_n_axes'])
                vars(self)[name + '_axes_to_shadow_buttons'] = np.matrix(vars(self)[name + '_axes_to_shadow_buttons']).reshape(self.n_shadow_buttons, vars(self)[name + '_n_axes'])
            if (vars(self)[name + '_n_buttons'] > 0):
                vars(self)[name + '_shadow_buttons_to_output_axes'] = np.matrix(vars(self)[name + '_shadow_buttons_to_output_axes']).reshape(self.n_output_axes, self.n_shadow_buttons)
                vars(self)[name + '_buttons_to_shadow_buttons'] = np.matrix(vars(self)[name + '_buttons_to_shadow_buttons']).reshape(self.n_shadow_buttons, vars(self)[name + '_n_buttons'])


def __special_wrap_angle__(angle):
    """ This function is used to wrap an angle to a value of [-1, 1] """
    # Angles from -1 to 1
    while (angle > 1.0):
        angle = angle - 2.0
    while (angle < -1.0):
        angle = angle + 2.0
    return angle


if __name__ == '__main__':
    try:
        rospy.init_node('map_ack')
        map_ack = MapAck(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
