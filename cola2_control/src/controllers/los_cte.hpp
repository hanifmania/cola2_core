
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_LOS_CTE__
#define __CONTROLLER_LOS_CTE__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "types.hpp"
#include <math.h>
#include <algorithm>
#include <cola2_lib/cola2_util.h>

typedef struct {
    double min_surge_velocity;
    double max_surge_velocity;
    double min_velocity_ratio;  // from 0 to 1
    double delta;
    double distance_to_max_velocity;
    bool heave_in_3D;
} LosCteControllerConfig;


class LosCteController {
public:
    // Methods
    LosCteController(LosCteControllerConfig);
    void compute(const control::State&,
                 const control::Section&,
                 control::State&,
                 control::Feedback&,
                 control::PointsList&);
    void setConfig(const LosCteControllerConfig&);

private:
    // Config
    LosCteControllerConfig _config;
};

// Constructor
LosCteController::LosCteController(LosCteControllerConfig config):
    _config(config)
{ }

void
LosCteController::setConfig(const LosCteControllerConfig &config)
{
    _config = config;
}

// Compute Method
void
LosCteController::compute(const control::State& current_state,
                          const control::Section& section,
                          control::State& controller_output,
                          control::Feedback& feedback,
                          control::PointsList& marker)
{
    // If the user defines a surge smaller than min_surge
    // the min_surge is not used
    bool use_min_surge = true;

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

    // std::cout << section.initial_position.x << ", " << section.initial_position.y << " to " << section.final_position.x << ", " << section.final_position.y << " \n";
    // Compute desired surge and yaw
    double surge = _config.max_surge_velocity;
    if (section.final_surge != 0.0) {
        surge = section.final_surge;
        if (surge < _config.min_surge_velocity) use_min_surge = false;
    }
    double desired_yaw;

    // std::cout << "Surge: " << surge << std::endl;

    // Distance to current way-point
    double dist_final = sqrt(pow(section.final_position.x - current_state.pose.position.north, 2) +
                             pow(section.final_position.y - current_state.pose.position.east, 2));

    // Distance to previous way-point
    double dist_origin = sqrt(pow(section.initial_position.x - current_state.pose.position.north, 2) +
                              pow(section.initial_position.y - current_state.pose.position.east, 2));

    // Distance to previous way-point
    double dist_waypoints = sqrt(pow(section.initial_position.x - section.final_position.x, 2) +
                              pow(section.initial_position.y - section.final_position.y, 2));

    // Compute cross-track error
	// Angle of path
    double beta = atan2(section.final_position.y - section.initial_position.y,
                         section.final_position.x - section.initial_position.x);//Fossen LOS, Sec. 10.3.2

    double sbeta = sin(beta);
    double cbeta = cos(beta);

	// Along-track distance (s) and cross-track error (e) (rotation)
    double s = (current_state.pose.position.north - section.initial_position.x) * cbeta +
    		   (current_state.pose.position.east - section.initial_position.y) * sbeta;

    double e = -(current_state.pose.position.north - section.initial_position.x) * sbeta +
                (current_state.pose.position.east - section.initial_position.y) * cbeta;

	// Orthogonal projection
    double x_proj = section.initial_position.x + (s + _config.delta) * cbeta;
    double y_proj = section.initial_position.y + (s + _config.delta) * sbeta;

	// Compute lookahead distance (los_delta). It is always positive
    double delta = 0.0;
	if(_config.distance_to_max_velocity > abs(e))
        delta = sqrt(pow(_config.distance_to_max_velocity,2.0) - pow(e,2.0));

	if(_config.distance_to_max_velocity > dist_final)
        delta = sqrt(pow(section.final_position.x - x_proj,2.0) + pow(section.final_position.y - y_proj,2.0));

	// Compute LOS vector
	double LOSX;
	double LOSY;
	if(s < dist_waypoints)
	{
		LOSX = x_proj - current_state.pose.position.north;
		LOSY = y_proj - current_state.pose.position.east;
	}
	else
	{
		LOSX = x_proj - current_state.pose.position.north;
		LOSY = y_proj - current_state.pose.position.east;
	}

	// Compute surge
    // Take the smaller one
	double dist_angle = atan2(1 - _config.min_velocity_ratio, _config.distance_to_max_velocity);
    double distance = std::min(dist_final, dist_origin);
    if (distance < _config.distance_to_max_velocity) {
        double ratio = _config.min_velocity_ratio + tan(dist_angle) * distance;
        surge = surge * ratio;
        if (use_min_surge && surge < _config.min_surge_velocity) surge = _config.min_surge_velocity;
    }

	// Compute yaw
    double beta_low_speed = 0.25;
    double beta_high_speed = 0.5;
    //self.parameters.sparus_los.sway_correction is true was defined in old param list
    bool sway_correction = true; //TODO
    double beta_factor;
    if (sway_correction)
    {
    	if(current_state.velocity.linear.x > beta_high_speed)
    		beta_factor = 1.0;
    	else if(current_state.velocity.linear.x < beta_low_speed)
    		beta_factor = 0.0;
    	else
    		beta_factor = (current_state.velocity.linear.x - beta_low_speed) / (beta_high_speed - beta_low_speed);
    }
    else
    	beta_factor = 0.0;

    //double beta = atan2(current_state.velocity.linear.y, current_state.velocity.linear.x);

    //desired_yaw = cola2::util::normalizeAngle(alpha + atan2(-e, _config.delta) - beta); //TODO
    desired_yaw = cola2::util::normalizeAngle(atan2(LOSY, LOSX));

    // define current z according to altitude_mode
    double current_z = current_state.pose.position.depth;
    if (section.altitude_mode) current_z = current_state.pose.altitude;

    // Check if final position X, Y is reached
    feedback.success = false;

    if(fabs(current_state.pose.position.north - section.final_position.x) < section.tolerance.x &&
       fabs(current_state.pose.position.east - section.final_position.y) < section.tolerance.y){
        // If Z axis is disabled success is true ...
        if(section.disable_z) {
            feedback.success = true;
        }
        // ... otherwise check Z value
        else if (fabs(current_z - section.final_position.z) < section.tolerance.z) {
            feedback.success = true;
        }
    }

    // Check if the limit line has been crossed. Where the limit line is
    // the line perpendicular to the desired path that pass through the
    // current final_positionwaypoint.
    double inc_y = section.final_position.y - section.initial_position.y;
    double m_l = 999999.9;
    if (inc_y != 0.0) {
        m_l = -(section.final_position.x - section.initial_position.x) / inc_y;
    }
    double c_l = -m_l * section.final_position.x + section.final_position.y;
    double current_d = (m_l * current_state.pose.position.north -
                        current_state.pose.position.east + c_l) / sqrt(m_l * m_l + 1);
    double sign = (m_l * section.initial_position.x -
                   section.initial_position.y + c_l) / sqrt(m_l * m_l + 1);
    if (sign * current_d < 0.0) {
        std::cout << "LOSCTE: beyond the final section waypoint!\n";

        // The vehicle has crossed the line
        if (section.disable_z){
            // If z is uncontrolled
            feedback.success = true;
        }
        else {
            if(fabs(current_z - section.final_position.z) < section.tolerance.z){
                // If Z is ok, success = True.
                feedback.success = true;
            }
            else {
                // Otherwise X and Yaw = 0.0 and success = false
                // wait for z to be True
                desired_yaw = current_state.pose.orientation.yaw;
                surge = 0.0;
                feedback.success = false;
            }
        }
    }

    // There was a problem when final and initial points in the section
    // are the same. This particular case can make the vehicle's drift
    // TODO: COMPTE! Quan esperem nomes la Z el robot no s'hauria de moure en
    // yaw no?
    if ((section.initial_position.x == section.final_position.x) &&
        (section.initial_position.y == section.final_position.y)){
        std::cout << "LOSCTE: initial and final waypoint are the same!\n";
        surge = 0.0;
        desired_yaw = current_state.pose.orientation.yaw;
        if(abs(current_z - section.final_position.z) < section.tolerance.z){
            feedback.success = true;
        }
    }

    // Set up controller's output and feedback
    // Set desired yaw
    controller_output.pose.orientation.yaw = desired_yaw;
    controller_output.pose.disable_axis.yaw = false;
    feedback.desired_yaw = desired_yaw;
    // std::cout << "Desired yaw: " << desired_yaw << "\n";

    // Set desired Z
    double desired_depth = section.final_position.z;
    if ((!section.altitude_mode) && (_config.heave_in_3D)) {
        double s = (current_state.pose.position.north - section.initial_position.x) * cbeta +
                   (current_state.pose.position.east - section.initial_position.y) * sbeta;
        double section_dx = (section.final_position.x - section.initial_position.x);
        double section_dy = (section.final_position.y - section.initial_position.y);
        double section_length = sqrt(pow(section_dx, 2.0) + pow(section_dy, 2.0));

        if (s < 0.0) {
            desired_depth = section.initial_position.z;
        }
        else if (s > section_length) {
            desired_depth = section.final_position.z;
        }
        else {
            if (section_length != 0.0) {
                desired_depth = section.initial_position.z + (section.final_position.z - section.initial_position.z) * (s / section_length);
            }
        }
    }

    controller_output.pose.altitude_mode = section.altitude_mode;
    controller_output.pose.altitude = section.final_position.z;
    controller_output.pose.position.depth = desired_depth;
    if (!section.disable_z){
        controller_output.pose.disable_axis.z = false;
        // std::cout << "Desired Z: " << desired_depth << "\n";
    }
    feedback.desired_depth = desired_depth;

    // Set Surge velocity
    controller_output.velocity.linear.x = surge;
    controller_output.velocity.disable_axis.x = false;
    feedback.desired_surge = surge;
    // std::cout << "Desired Surge: " << surge << "\n";

    // Fill additional feedback vars
    feedback.distance_to_end = dist_final;

    // Fill marker
    control::point initial_point;
    control::point final_point;

    initial_point.x = section.initial_position.x;
    initial_point.y = section.initial_position.y;
    initial_point.z = section.initial_position.z;
    marker.points_list.push_back(initial_point);
    final_point.x = section.final_position.x;
    final_point.y = section.final_position.y;
    final_point.z = section.final_position.z;
    marker.points_list.push_back(final_point);
}

#endif  /* __CONTROLLER_LOS_CTE__ */
