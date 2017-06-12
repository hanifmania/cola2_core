Header header

# Basic navigation status
float64 latitude
float64 longitude
float32 depth
float32 heading

# Safety
float32 up_time
bool vehicle_initialized
bool water_detected
float32 battery_charge
float32 internal_temperature
float32 internal_pressure

# Safety: data age
float32 navigation_data_age
float32 imu_data_age
float32 depth_data_age
float32 altitude_data_age
float32 dvl_valid_data_age
float32 gps_data_age
float32 modem_data_age
float32 wifi_data_age

# Running controller
uint8 CONTROLLER_NONE = 0
uint8 CONTROLLER_GOTO = 1
uint8 CONTROLLER_SECTION = 2
uint8 CONTROLLER_KEEP_POSITION = 3
uint8 CONTROLLER_MISSION = 4
uint8 active_controller

# Thrusters enabled
bool thrusters_enabled

# Mission status
int32 current_waypoint
int32 total_waypoints
float64 waypoint_latitude
float64 waypoint_longitude
float32 waypoint_z
