#ifndef __CONTROLLER_ANCHOR__
#define __CONTROLLER_ANCHOR__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "types.hpp"
#include <math.h>
#include <algorithm>
#include <cola2_lib/cola2_util.h>

typedef struct {
    double kp;
    double radius;
    double min_surge;
    double max_surge;
    double safety_distance;
    double max_angle_error;
} AnchorControllerConfig;


class AnchorController {
public:
    // Methods
    AnchorController(AnchorControllerConfig);
    void compute(const control::State&,
                 const control::Waypoint&,
                 control::State&,
                 control::Feedback&,
                 control::PointsList&);
    void setConfig(const AnchorControllerConfig&);

private:
    // Config
    AnchorControllerConfig _config;
};

// Constructor
AnchorController::AnchorController(AnchorControllerConfig config):
    _config(config)
{ }

void
AnchorController::setConfig(const AnchorControllerConfig &config)
{
    _config = config;
}

// Compute Method
void
AnchorController::compute(const control::State& current_state,
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
    
    // Compute distance to the waypoint
    double robot_distance_2D = sqrt(pow(waypoint.position.north - current_state.pose.position.north, 2) +
                                    pow(waypoint.position.east - current_state.pose.position.east, 2));

    // Yaw
    double desired_yaw = atan2(waypoint.position.east - current_state.pose.position.east,
                               waypoint.position.north - current_state.pose.position.north);

    // Surge
    double error = _config.radius - robot_distance_2D;
    double desired_surge = -_config.kp * error;
    if (desired_surge > _config.max_surge) {
        desired_surge = _config.max_surge;
    }
    if (desired_surge < _config.min_surge) {
        desired_surge = _config.min_surge;
    }
    if (fabs(cola2::util::normalizeAngle(desired_yaw - current_state.pose.orientation.yaw)) > _config.max_angle_error) {
        desired_surge = 0.0;
    }

    // Safety
    if (robot_distance_2D > _config.safety_distance) {
        desired_surge = 0.0;
        desired_yaw = 0.0;
    }

    // Set up controller's output and feedback:
    // Set z ...
    controller_output.pose.altitude_mode = waypoint.altitude_mode;
    controller_output.pose.altitude = waypoint.altitude;
    controller_output.pose.position.depth = waypoint.position.depth;
    controller_output.pose.disable_axis.z = false;
    feedback.desired_depth = waypoint.position.depth;
    if (waypoint.altitude_mode) feedback.desired_depth = waypoint.altitude;
    // Set surge ...
    controller_output.velocity.linear.x = desired_surge;
    controller_output.velocity.disable_axis.x = false;
    feedback.desired_surge = desired_surge;
    // Set yaw ...
    controller_output.pose.orientation.yaw = desired_yaw;
    controller_output.pose.disable_axis.yaw = false;
    feedback.desired_yaw = desired_yaw;

    // Fill additional feedback vars
    feedback.distance_to_end = robot_distance_2D;

    //Fill marker
    control::point initial_point;
    control::point final_point;

    // Set marker points
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

#endif  /* __CONTROLLER_ANCHOR__ */
