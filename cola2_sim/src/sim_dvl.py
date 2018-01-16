#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



""" 
@@>This node is able to simulate a DVL<@@
"""

"""
Modified 11/2015
@author: narcis palomeras
"""

import rospy

import PyKDL
import math
import numpy as np

from cola2_lib import cola2_ros_lib

from cola2_msgs.msg import Setpoints
from cola2_msgs.msg import LinkquestDvl
from cola2_msgs.msg import TeledyneExplorerDvl
from cola2_msgs.msg import PressureSensor


class SIM_DVL:

    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Init some vars
        self.surge_velocity = 0.0
        self.depth_velocity = 0.0
        self.depth_velocity_vector = []
        self.depth = 0.0
        self.setpoints = [0.0] * 3
        self.last_thrusters_callback = rospy.Time.now()
        self.last_pressure_callback = rospy.Time.now()

        # Config
        self.get_config()

        # Init linkquest dvl msg
        self.linkquest_dvl = LinkquestDvl()
        self.linkquest_dvl.header.frame_id = 'linkquest_navquest600_dvl_from_sim_dvl'
        self.linkquest_dvl.velocityInstFlag = 1
        for beam in range(4):
            self.linkquest_dvl.dataGood[beam] = 1
            self.linkquest_dvl.altitudeBeam[beam] = round(self.bottom_depth, 1)
        self.linkquest_dvl.altitude = min(self.linkquest_dvl.altitudeBeam)
        self.linkquest_dvl.temperature = 14.0
        self.linkquest_dvl.soundSpeed = 1500.0

        # Init rdi dvl msg
        self.rdi_dvl = TeledyneExplorerDvl()
        self.rdi_dvl.header.frame_id = 'teledyne_explorer_dvl_from_sim_dvl'
        self.rdi_dvl.bi_status = "A"

        # Publishers and subscribers
        rospy.Subscriber('/cola2_control/thrusters_data',
                         Setpoints,
                         self.update_thrusters,
                         queue_size = 1)
        rospy.Subscriber("/cola2_navigation/pressure_sensor",
                         PressureSensor,
                         self.update_pressure,
                         queue_size = 1)

        if self.dvl_type == 'rdi':
            self.pub_rdi = rospy.Publisher('/cola2_navigation/teledyne_explorer_dvl', 
                                           TeledyneExplorerDvl,
                                           queue_size = 2)
                                           
            rospy.Timer(rospy.Duration(0.2), self.publish_rdi)
        else:  # Linkquest
            self.pub_linkquest = rospy.Publisher('/cola2_navigation/linkquest_navquest600_dvl', 
                                                 LinkquestDvl,
                                                 queue_size = 2)
                                                 
            rospy.Timer(rospy.Duration(0.4), self.publish_linkquest)

        rospy.Timer(rospy.Duration(0.1), self.iterate)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def update_thrusters(self, data):
        """ Callback of setpoints """
        self.setpoints = list(data.setpoints)
        self.last_thrusters_callback = rospy.Time.now()


    def update_pressure(self, data):
        """ This method is the presure_sensor callback, input is in bars """
        # Check input data and create a new pose
        if data.pressure < -0.5:
            rospy.logfatal('%s: pressure sensor probably not connected', self.name)
            depth = 0.0
        else:
            depth = data.pressure * 10.043  # Bars to depth

        # Compute dt
        now = rospy.Time.now()
        dt = (now - self.last_pressure_callback).to_sec()
        self.last_pressure_callback = now
        if dt < 0.001:
            dt = 0.001

        # Compute depth velocity
        self.depth_velocity = (depth - self.depth) / dt
        #rospy.loginfo("%s: unfiltered depth_velocity: %s", self.name, self.depth_velocity)

        # Mean filter TODO: Savitsky Golay here!
        self.depth_velocity_vector.append(self.depth_velocity)
        while len(self.depth_velocity_vector) > 10:
            self.depth_velocity_vector.pop(0)
        if len(self.depth_velocity_vector) == 10:
            self.depth_velocity = sum(self.depth_velocity_vector) / 10.0

        #rospy.loginfo("%s:   filtered depth_velocity: %s", self.name, self.depth_velocity)
        #rospy.loginfo("%s: depth: %s", self.name, depth)
        #rospy.loginfo("%s: self.depth: %s", self.name, self.depth)
        #rospy.loginfo("%s: dt: %s", self.name, dt)

        # Store depth and compute altitude
        self.depth = depth

        # Compute altitude and store to linkquest and rdi
        for beam in range(4):
            self.linkquest_dvl.altitudeBeam[beam] = round(self.bottom_depth - self.depth, 1)
        self.linkquest_dvl.altitude = min(self.linkquest_dvl.altitudeBeam)
        self.rdi_dvl.bd_range = self.bottom_depth - self.depth


    def iterate(self, event):
        """ Main method """
        # Compute callback period
        callback_period = (rospy.Time.now() - self.last_thrusters_callback).to_sec()
        if callback_period > 2.0:
            self.setpoints = [0.0] * 3

        # Restore asymmetry
        setpoints = list(self.setpoints)
        for i in range(len(setpoints)):
            if setpoints[i] < 0.0:
                setpoints[i] = setpoints[i] / self.asymmetry

        # Compute thrust setpoint
        thrust_setpoint = (setpoints[1] + setpoints[2]) / 2.0

        # Compute thrust force
        thrust_force = thrust_setpoint * (self.thruster_thrust * 2.0)

        # Compute damping
        damping = - self.linear_damping * self.surge_velocity - self.quadratic_damping * abs(self.surge_velocity) * self.surge_velocity

        # Compute total force
        force = thrust_force + damping

        # Compute acceleration
        acceleration = force / self.mass

        # Compute new velocity
        self.surge_velocity = self.surge_velocity + 0.1 * acceleration

        # Print some info
        #rospy.loginfo("%s: thrust_setpoint: %s", self.name, thrust_setpoint)
        #rospy.loginfo("%s: thrust_force: %s", self.name, thrust_force)
        #rospy.loginfo("%s: damping: %s", self.name, damping)
        #rospy.loginfo("%s: force: %s", self.name, force)
        #rospy.loginfo("%s: acceleration: %s", self.name, acceleration)
        #rospy.loginfo("%s: surge_velocity: %s", self.name, self.surge_velocity)
        #rospy.loginfo("%s: depth_velocity: %s", self.name, self.depth_velocity)


    def publish_linkquest(self, event):
        """ Publish linkquest dvl message """
        # TODO: compute translation
        # Rotate velocity and store it into the linkquest dvl message
        vel = PyKDL.Vector(self.surge_velocity, 0.0, self.depth_velocity)  # Aprox in heave
        vel_rot = self.dvl_rotation.Inverse() * vel
        self.linkquest_dvl.velocityInst[0] = -vel_rot[0]  # Linkquest not dextrogyre
        self.linkquest_dvl.velocityInst[1] = -vel_rot[1]
        self.linkquest_dvl.velocityInst[2] = -vel_rot[2]

        # Publish
        self.linkquest_dvl.header.stamp = rospy.Time.now()
        self.pub_linkquest.publish(self.linkquest_dvl)
        rospy.loginfo("%s: publishing linkquest sim dvl", self.name)


    def publish_rdi(self, event):
        """ Publish rdi dvl message """
        # TODO: compute translation
        # Rotate velocity and store it into the linkquest dvl message
        vel = PyKDL.Vector(self.surge_velocity, 0.0, self.depth_velocity)  # Aprox in heave
        vel_rot = self.dvl_rotation.Inverse() * vel
        self.rdi_dvl.bi_x_axis = vel_rot[0]
        self.rdi_dvl.bi_y_axis = vel_rot[1]
        self.rdi_dvl.bi_z_axis = vel_rot[2]

        # Publish
        self.rdi_dvl.header.stamp = rospy.Time.now()
        self.pub_rdi.publish(self.rdi_dvl)
        rospy.loginfo("%s: publishing rdi sim dvl", self.name)


    def get_config(self):
        """ Get config from param server """
        param_dict = {'dvl_rotation': 'sim_dvl/dvl_rotation',
                      'dvl_type': 'sim_dvl/dvl_type',
                      'asymmetry': 'sim_dvl/asymmetry',
                      'mass': 'sim_dvl/mass',
                      'linear_damping': 'sim_dvl/linear_damping',
                      'quadratic_damping': 'sim_dvl/quadratic_damping',
                      'thruster_thrust': 'sim_dvl/thruster_thrust',
                      'bottom_depth': 'sim_dvl/bottom_depth'}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            self.bad_config_timer = rospy.Timer(rospy.Duration(0.4), self.bad_config_message)

        self.dvl_rotation = PyKDL.Rotation.RPY(self.dvl_rotation[0] * math.pi / 180.0, self.dvl_rotation[1] * math.pi / 180.0, self.dvl_rotation[2] * math.pi / 180.0)


    def bad_config_message(self, event):
        """ Timer to show an error if loading parameters failed """
        rospy.logfatal('%s: bad parameters in param server!', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('sim_dvl')
        sim_dvl = SIM_DVL(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
