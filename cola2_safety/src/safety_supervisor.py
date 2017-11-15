#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""@@>The diagnostics supervisor is in charge of receiving the aggregated 
diagnostics message and apply a set of rules aimed at detecting errors in 
the system (a sensor not giving data, low battery level, detection from 
water leak sensors, etc.). These rules are based on the configuration values
set on the cola2_s2/config/safety.yaml file. If any of the rule checks is 
triggered, the diagnostics supervisor calls a recovery action, which acts
appropriately depending on the type of error.<@@"""

"""
Created on 02/19/2015
Modified 11/2016
Modified 11/2017

@author: Narcis Palomeras, Tali Hurtos
"""

# ROS imports
import roslib
import rospy
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_msgs.msg import Int16
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

from cola2_msgs.msg import VehicleStatus
from cola2_msgs.srv import RecoveryAction, RecoveryActionRequest
from cola2_msgs.cfg import SafetyConfig

from cola2_lib.diagnostic_helper import DiagnosticHelper
from cola2_lib import cola2_ros_lib
from cola2_lib import cola2_lib



# Actions ID
INFORMATION = 0
ABORT_AND_STOP_THRUSTERS = 1

class Cola2Safety(object):

    def __init__(self, name):
        """ Init the class. """
        self.name = name

        # Vehicle initialized
        self.vehicle_init = False

        # Init error code
        self.error_code = ['0' for i in range(16)]

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")
        self.is_recovery_enabled = False

        # Diagnostic thresholds
        self.min_altitude = 1.5
        self.max_depth = 20.0
        self.max_temperatures_ids = ['batteries', 'pc', 'thrusters']
        self.max_temperatures_values = [55.0, 80.0, 95.0]
        self.min_battery_charge = 15.0
        self.min_battery_voltage = 25.0
        self.min_imu_update = 2.0
        self.min_depth_update = 2.0
        self.min_altitude_update = 2.0
        self.min_gps_update = 2.0
        self.min_dvl_update = 2.0
        self.min_range_update = 2.0
        self.min_wifi_update = 20
        self.min_dvl_good_data = 30
        self.water_leaks = True
        self.working_area_north_origin = -50.0
        self.working_area_east_origin = -50.0
        self.working_area_north_length = 100.0
        self.working_area_east_length = 100.0
        self.timeout = 600
        self.timeout_reset = 10

        # Get config parameters
        self.get_config()

        # Create Publishers
        self.pub_error_code = rospy.Publisher("/cola2_safety/error_code",
                                              Int16,
                                              queue_size = 2)

        # Subscriber
        rospy.Subscriber("/cola2_safety/vehicle_status",
                         VehicleStatus,
                         self.check_vehicle_status,
                         queue_size = 1)

        # Init Service Client
        try:
            rospy.wait_for_service('/cola2_safety/recovery_action', 20)
            self.recover_action_srv = rospy.ServiceProxy(
                        '/cola2_safety/recovery_action', RecoveryAction)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error creating client to recovery action.',
                         self.name)
            rospy.signal_shutdown('Error creating recover action client')

        try:
            rospy.wait_for_service('/cola2_safety/reset_timeout', 20)
            self.reset_timeout_srv = rospy.ServiceProxy(
                        '/cola2_safety/reset_timeout', Empty)

        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error creating client to reset timeout.',
                         self.name)
            rospy.signal_shutdown('Error creating reset timeout client')

        # Create dynamic reconfigure service
        self.dynamic_reconfigure_srv = Server(SafetyConfig,
                                              self.dynamic_reconfigure_callback)

        # Create service
        self.reload_params_srv = rospy.Service('/cola2_safety/reload_safety_params',
                                               Empty,
                                               self.reload_params_srv)

    def reload_params_srv(self, req):
        """ Callback of reload params service """
        rospy.loginfo('%s: received reload params service', self.name)
        self.get_config()
        return EmptyResponse()

    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {timeout}""".format(**config))
        self.timeout_reset = 10
        self.timeout = config.timeout
        self.reset_timeout_srv(EmptyRequest())
        return config

    def compute_error_byte(self):
        """ Update ERROR CODE with the captain status information. """

        current_step = bin(self.status.current_step % 256)
        # delete previous bits
        for b in range(8):
            self.error_code[cola2_lib.ErrorCode.CURRENT_WAYPOINT_BASE - b] = '0'
        # Fill current step
        for b in range(len(current_step) - 2):
            self.error_code[cola2_lib.ErrorCode.CURRENT_WAYPOINT_BASE - b] = current_step[-(1 + b)]

    def check_vehicle_status(self, vehicle_status):
        """ Check all the diagnostics to see if any of the following rules
            is true. When a rule is accomplished call the appropriated
            recovery action. """

        self.is_recovery_enabled = False

        # Rule: Init vehicle
        if vehicle_status.vehicle_initialized:
            self.vehicle_init = True
            self.error_code[cola2_lib.ErrorCode.INIT] = '1'

        # Rule: Battery Level
        battery_charge = vehicle_status.battery_charge
        battery_voltage = vehicle_status.battery_voltage
        if battery_charge < self.min_battery_charge or battery_voltage < self.min_battery_voltage:
            self.error_code[cola2_lib.ErrorCode.BAT_ERROR] = '1'
            self.error_code[cola2_lib.ErrorCode.BAT_WARNING] = '0'
            self.call_recovery_action("Battery Level below threshold!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)
        elif battery_charge < 1.5*self.min_battery_charge:
            self.error_code[cola2_lib.ErrorCode.BAT_ERROR] = '0'
            self.error_code[cola2_lib.ErrorCode.BAT_WARNING] = '1'
            self.call_recovery_action("Battery Level Low",
                                      RecoveryActionRequest.INFORMATIVE)
        else:
            self.diagnostic.add('battery_charge', str(battery_charge))
            rospy.loginfo("%s: Battery charge %s", self.name, str(battery_charge))

        # Rule: No IMU data
        last_imu = vehicle_status.imu_data_age
        self.diagnostic.add('last_imu_data', str(last_imu))
        if last_imu > self.min_imu_update:
            rospy.logerr("%s: No IMU data since %s", self.name, str(last_imu))
            self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
            self.call_recovery_action("No IMU data!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)
        else:
            rospy.loginfo("%s: Last IMU data %s", self.name, str(last_imu))

        # Rule: No Depth data
        last_depth = vehicle_status.depth_data_age
        self.diagnostic.add('last_depth_data', str(last_depth))
        if last_depth > self.min_depth_update:
            self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
            self.call_recovery_action("No Depth data!",
                                      RecoveryActionRequest.EMERGENCY_SURFACE)
        else:
            rospy.loginfo("%s: Last DEPTH data %s", self.name, str(last_depth))

        # Rule: No Altitude data
        last_altitude = vehicle_status.altitude_data_age
        self.diagnostic.add('last_altitude_data', str(last_altitude))
        if last_altitude > self.min_altitude_update:
            rospy.logerr("%s: last_altiude %s/%s", self.name, str(last_altitude), str(self.min_altitude_update) )
            self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
            self.call_recovery_action("No Altitude data!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)
        else:
            rospy.loginfo("%s: Last ALTITUDE data %s", self.name, str(last_altitude))

        # Rule: No DVL data
        last_dvl = vehicle_status.dvl_data_age
        self.diagnostic.add('last_dvl_data', str(last_dvl))
        if last_dvl > self.min_dvl_update:
            self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
            self.call_recovery_action("No DVL data!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)
        else:
            rospy.loginfo("%s: Last DVL data %s", self.name, str(last_dvl))

        # Rule: No GPS data
        last_gps = vehicle_status.gps_data_age
        self.diagnostic.add('last_gps_data', str(last_gps))
        if last_gps > self.min_gps_update:
            self.error_code[cola2_lib.ErrorCode.NAV_STS_WARNING] = '1'
            self.call_recovery_action("No GPS data!",
                                      RecoveryActionRequest.INFORMATIVE)
        else:
            rospy.loginfo("%s: Last GPS data %s", self.name, str(last_gps))

        # Rule: No Navigation data
        last_nav = vehicle_status.navigation_data_age
        self.diagnostic.add('last_nav_data', str(last_nav))
        if last_nav > self.min_nav_update:
            self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
            self.call_recovery_action("No Navigation data!",
                                      RecoveryActionRequest.EMERGENCY_SURFACE)
        else:
            rospy.loginfo("%s: Last Nav data %s", self.name, str(last_nav))

        # Rule: No WIFI data and not in a mission
        last_ack = vehicle_status.wifi_data_age
        if not vehicle_status.mission_active:
            self.diagnostic.add('last_ack', str(last_ack))
            if last_ack > self.min_wifi_update:
                self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_WARNING] = '0'
                self.call_recovery_action("No WiFi data!",
                                          RecoveryActionRequest.ABORT_AND_SURFACE)
            else:
                rospy.loginfo("%s: Last WIFI data %s", self.name, str(last_ack))
        else:
            rospy.loginfo("%s: A mission is active. WiFi timeout disabled.", self.name)
            self.diagnostic.add('last_ack', 'Mission active')

        # Rule: No Modem data
        last_modem = vehicle_status.modem_data_age
        self.diagnostic.add('last_modem_data', str(last_modem))
        if last_modem > self.min_modem_update:
            self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_WARNING] = '0'
            self.call_recovery_action("No Modem data!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)
        else:
            rospy.loginfo("%s: Last Modem data %s", self.name, str(last_modem))

        # Rule: No DVL good data
        last_good_dvl_data = vehicle_status.dvl_valid_data_age
        self.diagnostic.add('last_dvl_good_data', str(last_good_dvl_data))
        if last_good_dvl_data > self.min_dvl_good_data:
            self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_WARNING] = '0'
            self.call_recovery_action("No DVL good data!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)
        else:
            rospy.loginfo("%s: No DVL good data %s", self.name, str(last_good_dvl_data))

        # Rule: Water Leak
        if vehicle_status.water_detected:
            self.diagnostic.add('water_detected', 'True')
            self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
            self.call_recovery_action("Water Inside!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)
        else:
            rospy.loginfo("%s: no water", self.name)
            self.diagnostic.add('water_detected', 'False')

        # Rule: High Temperature
        temperatures = vehicle_status.internal_temperature
        if temperatures:
            for t in range(0, len(temperatures)):
                if temperatures[t] > self.max_temperatures_values[t]:
                    self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
                    self.call_recovery_action(self.max_temperatures_ids[0] + " high temperature",
                                              RecoveryActionRequest.ABORT_AND_SURFACE)
                else:
                    self.diagnostic.add('vehicle_temperature', 'Ok')
                    rospy.loginfo("%s: Vehicle temperature Ok", self.name)

        # Rule: Absolute timeout
        up_time = vehicle_status.up_time
        self.diagnostic.add('up_time', str(up_time))
        if float(up_time) > self.timeout and self.timeout_reset < 0:
            # self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
            self.call_recovery_action("Absolute Timeout reached!",
                                      RecoveryActionRequest.ABORT_AND_SURFACE)

        else:
            rospy.loginfo("%s: up_time (%s) < timeout (%s)", self.name, up_time, self.timeout)
            self.timeout_reset = self.timeout_reset - 1

        # Publish error code
        self.publishErrorCode()

        if not self.is_recovery_enabled:
            self.diagnostic.setLevel(DiagnosticStatus.OK)

    def publishErrorCode(self):
        error_code_str = ''.join(self.error_code)
        error_code_int16 = int(error_code_str, 2)
        self.pub_error_code.publish(Int16(error_code_int16))

    def get_config(self):
        """ Read parameters from ROS Param Server."""

        param_dict = {'min_altitude': 'safe_depth_altitude/min_altitude',
                      'max_depth': 'safe_depth_altitude/max_depth',
                      'min_battery_charge': 'safety/min_battery_charge',
                      'min_battery_voltage': 'safety/min_battery_voltage',
                      'min_imu_update': 'safety/min_imu_update',
                      'min_depth_update': 'safety/min_depth_update',
                      'min_altitude_update': 'safety/min_altitude_update',
                      'min_gps_update': 'safety/min_gps_update',
                      'min_dvl_update': 'safety/min_dvl_update',
                      'min_nav_update': 'safety/min_nav_update',
                      'min_wifi_update': 'safety/min_wifi_update',
                      'working_area_north_origin': 'virtual_cage/north_origin',
                      'working_area_east_origin': 'virtual_cage/east_origin',
                      'working_area_north_length': 'virtual_cage/north_longitude',
                      'working_area_east_length': 'virtual_cage/east_longitude',
                      'timeout': 'safety/timeout',
                      'min_modem_update': 'safety/min_modem_update',
                      'min_dvl_good_data': 'safety/min_dvl_good_data',
                      'max_temperatures_ids': 'safety/max_temperatures_ids',
                      'max_temperatures_values': 'safety/max_temperatures_values'}

        cola2_ros_lib.getRosParams(self, param_dict, self.name)

        # Define min altitude and max depth
        try:
            client2 = dynamic_reconfigure.client.Client("safe_depth_altitude",
                                                        timeout=10)
            client2.update_configuration({"min_altitude": self.min_altitude,
                                          "max_depth": self.max_depth})

        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error modifying safe_depth_altitude params.',
                         self.name)

        # Define virtual cage limits
        try:
            client3 = dynamic_reconfigure.client.Client("virtual_cage",
                                                        timeout=10)
            client3.update_configuration({"north_origin": self.working_area_north_origin,
                                          "east_origin": self.working_area_east_origin,
                                          "north_longitude": self.working_area_north_length,
                                          "east_longitude": self.working_area_east_length,
                                          "enable": True})
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error modifying virtual_cage params.',
                         self.name)

    def call_recovery_action(self,
                             str_err="Error!",
                             level=RecoveryActionRequest.INFORMATIVE):

        self.is_recovery_enabled = True
        print('--> recovery action!!: ', self.vehicle_init)
        self.diagnostic.setLevel(DiagnosticStatus.ERROR, str_err)

        if self.vehicle_init:
            rospy.logerr("%s: %s", self.name, str_err)
            req = RecoveryActionRequest()
            req.error_level = level
            ans = self.recover_action_srv(req)
            rospy.logerr("%s: Recovery action called --> %s", self.name, ans)

        rospy.sleep(5.0)


def __getDiagnostic__(status, name, key='none', default=0.0):
    if status.name == name:
        if key != 'none':
            return __getValue__(status.values, key, default)
        else:
            return True
    return False


def __getValue__(values, key, default):
    for pair in values:
        if pair.key == key:
            return pair.value
    return default


if __name__ == '__main__':
    try:
        rospy.init_node('safety_supervisor')
        C2S = Cola2Safety(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
