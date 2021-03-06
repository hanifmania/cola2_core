
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_GOTO__
#define __CONTROLLER_GOTO__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "types.hpp"
#include <math.h>
#include <algorithm>
#include <cola2_lib/cola2_util.h>

typedef struct {
    double surge_proportional_gain;
    double max_angle_error;
    double max_surge;
} GotoControllerConfig;


class GotoController {
public:
    // Methods
    GotoController(GotoControllerConfig);
    void compute(const control::State&,
                 const control::Waypoint&,
                 control::State&,
                 control::Feedback&,
                 control::PointsList&);
    void setConfig(const GotoControllerConfig&);

private:
    // Config
    GotoControllerConfig _config;
};

// Constructor
GotoController::GotoController(GotoControllerConfig config):
    _config(config)
{ }

// Compute Method
void
GotoController::compute(const control::State& current_state,
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

    // Compute YAW error
    double inc_x = waypoint.position.north - current_state.pose.position.north;
    double inc_y = waypoint.position.east - current_state.pose.position.east;
    double desired_yaw = atan2(inc_y, inc_x);
    double yaw_error = cola2::util::normalizeAngle(desired_yaw - current_state.pose.orientation.yaw);
    double distance = sqrt(pow(inc_x, 2.0) + pow(inc_y, 2.0));

    // Compute SURGE error
    // If current distance to wp > 1/vel_x_k and angle_error < max_error
    // then move to max vel
    double surge = 0.0;
    if(fabs(yaw_error) < _config.max_angle_error){
        surge = sqrt(pow(inc_x, 2) + pow(inc_y, 2)) * _config.surge_proportional_gain;
        surge = cola2::util::saturate(surge, 1.0);
    }

    // Adjust Surge response depending on Yaw error
    surge = surge * (1.0 - (fabs(yaw_error) / _config.max_angle_error));

    // Move from 25% to 100% of surge max velocity, not less
    if(surge > 0.0 && surge < 0.25){
        surge = 0.25;
    }

    double velocity = waypoint.linear_velocity.x;
    if (velocity == 0.0 || velocity > _config.max_surge * 3.0) {
        velocity = _config.max_surge;
    }
    surge = surge * velocity;

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
        if(fabs(current_state.pose.position.north - waypoint.position.north) > waypoint.position_tolerance.x ||
           fabs(current_state.pose.position.east - waypoint.position.east) > waypoint.position_tolerance.y){
            feedback.success = false;
        }
    }
    // If necessary, check if final position Z is reached
    if(!waypoint.disable_axis.z &&
       fabs(current_z - desired_z) > waypoint.position_tolerance.z) {
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
        controller_output.velocity.linear.x = surge;
        controller_output.velocity.disable_axis.x = false;
        feedback.desired_surge = surge;
        controller_output.pose.orientation.yaw = desired_yaw;
        controller_output.pose.disable_axis.yaw = false;
        feedback.desired_yaw = desired_yaw;
    }

    // Set yaw if yaw is enabled and X is disabled
    if (waypoint.disable_axis.x && !waypoint.disable_axis.yaw){
        controller_output.pose.orientation.yaw = waypoint.orientation.yaw;
        controller_output.pose.disable_axis.yaw = false;
        feedback.desired_yaw = waypoint.orientation.yaw;
    }

    // Fill additional feedback vars
    feedback.distance_to_end = distance;

    //Fill marker
    control::point initial_point;
    control::point final_point;

    initial_point.x = current_state.pose.position.north;
    initial_point.y = current_state.pose.position.east;
    initial_point.z = current_state.pose.position.depth;
    marker.points_list.push_back(initial_point);
    final_point.x = waypoint.position.north;
    final_point.y = waypoint.position.east;
    if (waypoint.altitude_mode) final_point.z = current_state.pose.position.depth + (current_state.pose.altitude - waypoint.altitude);
    else final_point.z = waypoint.position.depth;

    marker.points_list.push_back(final_point);
}

void
GotoController::setConfig(const GotoControllerConfig &config)
{
    _config = config;
}

#endif  /* __CONTROLLER_GOTO__ */
