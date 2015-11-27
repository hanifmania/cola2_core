#!/usr/bin/env python

"""
Created on 02/19/2015
Modified 11/2015

@author: Narcis Palomeras
"""

# ROS imports
import roslib
import rospy
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server

from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Int16
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

from cola2_msgs.msg import MissionStatus
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

        # Disagnostic thresholds
        self.min_altitude =  1.5
        self.max_depth = 20.0
        self.max_temperature_pc = 75.0
        self.max_temperature_bat = 60.0
        self.min_battery_charge = 15.0
        self.min_imu_update = 2.0
        self.min_depth_update = 2.0
        self.min_altitude_update = 2.0
        self.min_gps_update = 2.0
        self.min_dvl_update = 2.0
        self.min_range_update = 2.0
        self.min_wifi_update = 20
        self.min_acomms_update = 30
        self.min_distance_to_wall = 5.0
        self.min_dvl_good_data = 30

        self.water_leacks = True
        self.working_area_north_origin = -50.0
        self.working_area_east_origin = -50.0
        self.working_area_north_length = 100.0
        self.working_area_east_length = 100.0
        self.min_mission_handler_heartbeat_update = 5.0
        self.timeout = 600

        self.timeout_reset = 10

        # Get config parameters
        self.get_config()

        # Create Publishers
        self.pub_error_code = rospy.Publisher("/cola2_safety/error_code",
                                              Int16,
                                              queue_size = 2)

        # Subscriber
        rospy.Subscriber("/diagnostics_agg",
                         DiagnosticArray,
                         self.update_diagnostics,
                         queue_size = 1)

        # Create Subscriber
        rospy.Subscriber("/cola2_control/mission_status",
                         MissionStatus,
                         self.updateMissionStatus,
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

        # Create dynamic reconfigure servoce
        self.dynamic_reconfigure_srv = Server( SafetyConfig,
                                               self.dynamic_reconfigure_callback )
                                               # Define mission timeout


    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {timeout}""".format(**config))
        self.timeout_reset = 10
        self.timeout = config.timeout
        self.reset_timeout_srv( EmptyRequest() )
        return config


    def updateMissionStatus(self, mission_status):
        """ Update ERROR CODE with the mission status information. """

        current_wp = bin(mission_status.current_wp % 64)
        for b in range(len(current_wp) - 2):
            self.error_code[cola2_lib.ErrorCode.CURRENT_WAYPOINT_BASE - b] = current_wp[-(1+b)]


    def update_diagnostics(self, diagnostics):
        """ Check all the diagnostics to see if any of the following rules
            is true. When a rule is accomplished call the appropiated
            recovery action. """

        for status in diagnostics.status:
            self.is_recovery_enabled = False

            # Rule: Init vehicle
            if __getDiagnostic__( status, '/navigation/ navigator'):
                if __getDiagnostic__(status, '/navigation/ navigator', 'ekf_init', 'False') == 'True' :
                    # rospy.sleep(10.0) # Once the vehicle is init wait 10s to receive all the diagnostics
                    self.vehicle_init = True
                    self.error_code[cola2_lib.ErrorCode.INIT] = '1'

            # Rule: Battery Level
            if __getDiagnostic__( status, '/safety/ battery'):
                battery_charge = float(__getDiagnostic__(status, '/safety/ battery', 'charge', 100.0))

                self.diagnostic.add('battery_charge', str(battery_charge))
                if battery_charge < self.min_battery_charge:
                    self.error_code[cola2_lib.ErrorCode.BAT_ERROR] = '1'
                    self.error_code[cola2_lib.ErrorCode.BAT_WARNING] = '0'
                    self.call_recovery_action(
                                    "Battery Level below threshold!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                elif battery_charge < 1.5*self.min_battery_charge:
                    self.error_code[cola2_lib.ErrorCode.BAT_ERROR] = '0'
                    self.error_code[cola2_lib.ErrorCode.BAT_WARNING] = '1'
                    self.call_recovery_action(
                                    "Battery Level Low",
                                     RecoveryActionRequest.INFORMATIVE)
                else:
                    rospy.loginfo("%s: Battery charge %s", self.name, str(battery_charge))



            # Rule: No IMU data
            if __getDiagnostic__( status, '/navigation/ navigator'):
                last_imu = float(__getDiagnostic__(status, '/navigation/ navigator', 'last_imu_data', 0.0))
                if last_imu > 10000.0: # To avoid initialization problems
                    last_imu = 0.0

                self.diagnostic.add('last_imu_data', str(last_imu))
                if last_imu > self.min_imu_update:
                    rospy.logerr("%s: No IMU data since %s", self.name, str(last_imu))
                    self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
                    self.call_recovery_action(
                                    "No IMU data!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                else:
                    rospy.loginfo("%s: Last IMU data %s", self.name, str(last_imu))



            # Rule: No Depth data
            if __getDiagnostic__( status, '/navigation/ navigator'):
                last_depth = float(__getDiagnostic__(status, '/navigation/ navigator', 'last_depth_data', 0.0))
                if last_depth > 10000.0: # To avoid initialization problems
                    last_depth = 0.0

                self.diagnostic.add('last_depth_data', str(last_depth))
                if last_depth > self.min_depth_update:
                    self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
                    self.call_recovery_action(
                                    "No Depth data!",
                                     RecoveryActionRequest.EMERGENCY_SURFACE)
                else:
                    rospy.loginfo("%s: Last DEPTH data %s", self.name, str(last_depth))


            # Rule: No Altitude data
            if __getDiagnostic__( status, '/navigation/ navigator'):
                last_altitude = float(__getDiagnostic__(status, '/navigation/ navigator', 'last_altitude_data', 0.0))
                if last_altitude > 10000.0: # To avoid initialization problems
                    last_altitude = 0.0

                self.diagnostic.add('last_altitude_data', str(last_altitude))
                if last_altitude > self.min_altitude_update:
                    rospy.logerr("%s: last_altiude %s/%s", self.name, str(last_altitude), str(self.min_altitude_update) )
                    self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
                    self.call_recovery_action(
                                    "No Altitude data!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                else:
                    rospy.loginfo("%s: Last ALTITUDE data %s", self.name, str(last_altitude))


            # Rule: No DVL data
            if __getDiagnostic__( status, '/navigation/ navigator'):
                last_dvl = float(__getDiagnostic__(status, '/navigation/ navigator', 'last_dvl_data', 0.0))
                if last_dvl > 10000.0: # To avoid initialization problems
                    last_dvl = 0.0

                self.diagnostic.add('last_dvl_data', str(last_dvl))
                if last_dvl > self.min_dvl_update:
                    self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
                    self.call_recovery_action(
                                    "No DVL data!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                else:
                    rospy.loginfo("%s: Last DVL data %s", self.name, str(last_dvl))


            # Rule: No GPS data
            if __getDiagnostic__( status, '/navigation/ navigator'):
                last_gps = float( __getDiagnostic__(status, '/navigation/ navigator', 'last_gps_data', 0.0) )
                if last_gps > 10000.0: # To avoid initialization problems
                    last_gps = 0.0

                self.diagnostic.add('last_gps_data', str(last_gps))
                if last_gps > self.min_gps_update:
                    self.error_code[cola2_lib.ErrorCode.NAV_STS_WARNING] = '1'
                    self.call_recovery_action(
                                    "No GPS data!",
                                     RecoveryActionRequest.INFORMATIVE)
                else:
                    rospy.loginfo("%s: Last GPS data %s", self.name, str(last_gps))


            # Rule: No Navigation data
            if __getDiagnostic__( status, '/safety/ up_time'):
                last_nav = float(__getDiagnostic__(status, '/safety/ up_time', 'last_nav_data', 0.0))
                if last_nav > 10000.0: # To avoid initialization problems
                    last_nav = 0.0

                self.diagnostic.add('last_nav_data', str(last_nav))
                if last_nav > self.min_nav_update:
                    self.error_code[cola2_lib.ErrorCode.NAV_STS_ERROR] = '1'
                    self.call_recovery_action(
                                    "No Navigation data!",
                                     RecoveryActionRequest.EMERGENCY_SURFACE)
                else:
                    rospy.loginfo("%s: Last Nav data %s", self.name, str(last_nav))


            # Rule: No WIFI data and not in a mission
            if __getDiagnostic__( status, '/control/ teleoperation'):
                last_ack = float(__getDiagnostic__(status, '/control/ teleoperation', 'last_ack', 0.0))
                if last_ack > 100000.0: last_ack = 0.0

                # Check if trajectory is enabled
                trajectory_enabled = 'False'
                for status_2 in diagnostics.status:
                    if __getDiagnostic__( status_2, '/control/ captain'):
                        trajectory_enabled = __getDiagnostic__(status_2, '/control/ captain', 'trajectory_enabled', 'False')

                if trajectory_enabled == 'False':
                    self.diagnostic.add('last_ack', str(last_ack))
                    if last_ack > self.min_wifi_update:
                        self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_WARNING] = '0'
                        self.call_recovery_action(
                                        "No WiFi data!",
                                         RecoveryActionRequest.ABORT_AND_SURFACE)
                    else:
                        rospy.loginfo("%s: Last WIFI data %s", self.name, str(last_ack))
                else:
                    rospy.loginfo("%s: Mission enabled %s. WiFi timeout disabled.", self.name, trajectory_enabled)
                    self.diagnostic.add('last_ack', 'Mission enabled')


            # Rule: No Modem data
            if __getDiagnostic__( status, '/safety/ evologics_modem'):
                last_modem = float(__getDiagnostic__(status, '/safety/ evologics_modem', 'last_modem_data', 0.0))
                if last_modem > 100000.0: last_modem = 0.0

                self.diagnostic.add('last_modem_data', str(last_modem))
                if last_modem > self.min_modem_update:
                    self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_WARNING] = '0'
                    self.call_recovery_action(
                                    "No Modem data!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                else:
                    rospy.loginfo("%s: Last Modem data %s", self.name, str(last_modem))


            # Rule: No DVL good data
            if __getDiagnostic__( status, '/navigation/ teledyne_explorer_dvl'):
                last_good_dvl_data = float(__getDiagnostic__(status, '/navigation/ teledyne_explorer_dvl', 'last_good_data', 0.0))
                if last_good_dvl_data > 100000.0: last_good_dvl_data = 0.0

                self.diagnostic.add('last_dvl_good_data', str(last_good_dvl_data))
                if last_good_dvl_data > self.min_dvl_good_data:
                    self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_WARNING] = '0'
                    self.call_recovery_action(
                                    "No DVL good data!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                else:
                    rospy.loginfo("%s: No DVL good data %s", self.name, str(last_good_dvl_data))


            # Rule: Water Leack --> G500
            if __getDiagnostic__( status, '/safety/ internal_sensors'):
                water_pc = __getDiagnostic__(status, '/safety/ internal_sensors', 'PC_water', 'False')
                water_bat = __getDiagnostic__(status, '/safety/ internal_sensors', 'BAT_water', 'False')

                if water_pc == 'True' or water_bat == 'True':
                    self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
                    self.call_recovery_action(
                                    "Water Inside!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                    self.diagnostic.add('water_detected', 'True')
                    self.diagnostic.add('g500_water', 'True')
                else:
                    rospy.loginfo("%s: G500 no water", self.name)
                    self.diagnostic.add('g500_water', 'False')


            # Rule: Water Leack --> S2
            if __getDiagnostic__( status, '/safety/ mon_control_board') or __getDiagnostic__( status, '/control/ actuators'):
                water_main = __getDiagnostic__(status, '/safety/ mon_control_board', 'water_inside', 'False')
                water_fins = __getDiagnostic__(status, '/control/ actuators', 'water_inside_fins', 'False')

                if water_main == 'True' or water_fins == 'True':
                    self.diagnostic.add('s2_water', 'True')
                    self.diagnostic.add('water_detected', 'True')
                    self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
                    self.call_recovery_action(
                                    "Water Inside!",
                                     RecoveryActionRequest.ABORT_AND_SURFACE)
                else:
                    rospy.loginfo("%s: S2 no water", self.name)
                    self.diagnostic.add('s2_water', 'False')



            # Rule: High Temperature
            if __getDiagnostic__( status, '/safety/ internal_sensors'):
                temp_pc = __getDiagnostic__(status, '/safety/ internal_sensors', 'PC_temperature', 0.0)
                temp_bat = __getDiagnostic__(status, '/safety/ internal_sensors', 'BAT_temperature', 0.0)

                if temp_pc != 'none' and temp_bat != 'none':
                    self.diagnostic.add('g500_temperature', 'Ok')
                    if float(temp_pc) > self.max_temperature_pc:
                        self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
                        self.call_recovery_action(
                                        "PCs High temperature",
                                         RecoveryActionRequest.ABORT_AND_SURFACE)
                    elif float(temp_bat) > self.max_temperature_bat:
                        self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
                        self.call_recovery_action(
                                        "Batteries High temperature",
                                         RecoveryActionRequest.ABORT_AND_SURFACE)
                    else:
                        rospy.loginfo("%s: G500 temperature ok", self.name)

            # Rule: absolute timeout
            if __getDiagnostic__( status, '/safety/ up_time'):
                up_time = __getDiagnostic__(status, '/safety/ up_time', 'up_time', '0.0')
                self.diagnostic.add('up_time', up_time)
                if float(up_time) > self.timeout and self.timeout_reset < 0:
                    self.error_code[cola2_lib.ErrorCode.INTERNAL_SENSORS_ERROR] = '1'
                    self.call_recovery_action(
                                    "Absolute Timeout reached!",
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
                      'max_temperature_pc': 'safety/max_temperature_pc',
                      'max_temperature_bat': 'safety/max_temperature_bat',
                      'min_battery_charge': 'safety/min_battery_charge',
                      'min_imu_update': 'safety/min_imu_update',
                      'min_depth_update': 'safety/min_depth_update',
                      'min_altitude_update': 'safety/min_altitude_update',
                      'min_gps_update': 'safety/min_gps_update',
                      'min_dvl_update': 'safety/min_dvl_update',
                      'min_nav_update': 'safety/min_nav_update',
                      'min_wifi_update': 'safety/min_wifi_update',
                      'min_distance_to_wall': 'safety/min_distance_to_wall',
                      'working_area_north_origin': 'virtual_cage/north_origin',
                      'working_area_east_origin': 'virtual_cage/east_origin',
                      'working_area_north_length': 'virtual_cage/north_longitude',
                      'working_area_east_length': 'virtual_cage/east_longitude',
                      'timeout': 'diagnostics_supervisor/timeout',
                      'min_modem_update': 'safety/min_modem_update',
                      'min_dvl_good_data': 'safety/min_dvl_good_data'}

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
        print '--> recovery action!!: ', self.vehicle_init
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
        rospy.init_node('diagnostics_supervisor')
        C2S = Cola2Safety(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
