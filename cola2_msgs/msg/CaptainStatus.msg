Header header

# Running controller
uint8 CONTROLLER_NONE = 0
uint8 CONTROLLER_WAYPOINT = 1
uint8 CONTROLLER_SECTION = 2
uint8 CONTROLLER_PARK = 3
uint8 active_controller
bool altitude_mode

# Mission status
bool mission_active
int32 current_step
int32 total_steps
