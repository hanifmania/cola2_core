#ifndef __CONTROLLER_HOLONOMIC_GOTO__
#define __CONTROLLER_HOLONOMIC_GOTO__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "types.hpp"
#include <math.h>
#include <algorithm>
// #include <cola2_lib/cola2_util.h>

typedef struct {
    double no_params;
} HolonomicGotoControllerConfig;


class HolonomicGotoController {
public:
    // Methods
    HolonomicGotoController(HolonomicGotoControllerConfig);
    void compute(const control::State&,
                 const control::Waypoint&,
                 control::State&,
                 control::Feedback&,
                 control::PointsList&);
    void setConfig(const HolonomicGotoControllerConfig&);

private:
    // Config
    HolonomicGotoControllerConfig _config;
};

// Constructor
HolonomicGotoController::HolonomicGotoController(HolonomicGotoControllerConfig config):
    _config(config)
{ }

// Compute Method
void
HolonomicGotoController::compute(const control::State& current_state,
                                 const control::Waypoint& waypoint,
                                 control::State& controller_output,
                                 control::Feedback& feedback,
                                 control::PointsList& marker)
{
    // Set all axis as disabled by default
    controller_output.pose.disable_axis.x = true;
    controller_output.pose.disable_axis.y = true;
    controller_output.pose.disable_axis.z = true;
    controller_output.pose.disable_axis.roll = true;
    controller_output.pose.disable_axis.pitch = true;
    controller_output.pose.disable_axis.yaw = true;
    controller_output.velocity.disable_axis.x = true;
    controller_output.velocity.disable_axis.y = true;
    controller_output.velocity.disable_axis.z = true;
    controller_output.velocity.disable_axis.roll = true;
    controller_output.velocity.disable_axis.pitch = true;
    controller_output.velocity.disable_axis.yaw = true;

    // Take desired and current Z
    double desired_z = waypoint.position.depth;
    double current_z = current_state.pose.position.depth;
    if(waypoint.altitude_mode){
        desired_z = waypoint.altitude;
        current_z = current_state.pose.altitude;
    }
    // If necessary, check if final position X, Y is reached
    feedback.success = true;
    if(!waypoint.disable_axis.x){  // X-Y tolerance must be checked
        if(fabs(current_state.pose.position.north - waypoint.position.north) >= waypoint.position_tolerance.x ||
           fabs(current_state.pose.position.east - waypoint.position.east) >= waypoint.position_tolerance.y){
            feedback.success = false;
        }
    }
    // If necessary, check if final position Z is reached
    if(!waypoint.disable_axis.z &&
       fabs(current_z - desired_z) >= waypoint.position_tolerance.z) {
        feedback.success = false;
    }
    // If necessary, check if final angle yaw is reached
    if(!waypoint.disable_axis.yaw &&
        fabs(cola2::util::normalizeAngle(current_state.pose.orientation.yaw - waypoint.orientation.yaw)) >= waypoint.orientation_tolerance.yaw) {
        feedback.success = false;
    }

    // Set up controller's output and feedback
    // Set desired Z
    if (!waypoint.disable_axis.z){
        controller_output.pose.altitude_mode = waypoint.altitude_mode;
        controller_output.pose.altitude = waypoint.altitude;
        controller_output.pose.position.depth = waypoint.position.depth;
        controller_output.pose.disable_axis.z = false;
        feedback.desired_depth = desired_z;
        // std::cout << "Desired Z: " << section.final_position.z << "\n";
    }

    // If X-Y motion is enabled
    if (!waypoint.disable_axis.x){
        // Set Surge velocity
        controller_output.pose.position.north = waypoint.position.north;
        controller_output.pose.position.east = waypoint.position.east;
        controller_output.pose.disable_axis.x = false;
        controller_output.pose.disable_axis.y = false;
    }

    if(!waypoint.disable_axis.yaw){
        // Set desired yaw
        controller_output.pose.orientation.yaw = waypoint.orientation.yaw;
        controller_output.pose.disable_axis.yaw = false;
        feedback.desired_yaw = waypoint.orientation.yaw;
    }

    // Fill additional feedback vars
    feedback.distance_to_end = sqrt(pow(current_state.pose.position.north - waypoint.position.north, 2) +
                                    pow(current_state.pose.position.east - waypoint.position.east, 2));

    //Fill marker
    control::point initial_point;
    control::point final_point;

    initial_point.x = current_state.pose.position.north;
    initial_point.y = current_state.pose.position.east;
    initial_point.z = current_state.pose.position.depth;
    marker.points_list.push_back(initial_point);
    final_point.x = waypoint.position.north;
    final_point.y = waypoint.position.east;
    final_point.z = waypoint.position.depth;
    marker.points_list.push_back(final_point);
}


#endif  /* __CONTROLLER_HOLONOMIC_GOTO__ */
