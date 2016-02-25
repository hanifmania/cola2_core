#ifndef __CONTROLLER_DUBINS__
#define __CONTROLLER_DUBINS__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "types.hpp"
#include <math.h>
#include <cola2_lib/cola2_util.h>

#include <ros/ros.h>  // TODO: this should not publish the markers!


typedef struct {
    double yaw_ki;
    double yaw_kp;
    double lookahead_sec;
    double acceptance_sec;
} DubinsSectionControllerConfig;


class DubinsSectionController {
public:
    // Methods
    DubinsSectionController();
    void compute(const control::State&,
                 const control::Section&,
                 double period,
                 control::State&,
                 control::Feedback&,
                 ros::Publisher&);
    void setConfig(const DubinsSectionControllerConfig&);

private:
    // Config
    DubinsSectionControllerConfig _config;

    // Other vars
    double _yaw_old_e;
    double _yaw_old_psi;

    // Methods
    void reportError(const std::string&);
    double computeYaw(double, double, double, double);
    void computeEBetaGammaSectionLength(const control::State&,
                                        const control::Section&,
                                        double&, double&, double&, double&,
                                        ros::Publisher&);
    double wrapZeroToTwoPi(double);
};


DubinsSectionController::DubinsSectionController():
    _yaw_old_e(0.0),
    _yaw_old_psi(0.0)
{
    // Default config
    _config.yaw_ki         = 0.003;//0.003;//0.006;
    _config.yaw_kp         = 0.09;//0.12;
    _config.lookahead_sec  = 4.0;
    _config.acceptance_sec = 3.0;
}


void
DubinsSectionController::reportError(const std::string& message) {
    throw std::runtime_error("/dubins: " + message);
}


void
DubinsSectionController::compute(const control::State& current_state,
                                 const control::Section& section,
                                 double period,
                                 control::State& controller_output,
                                 control::Feedback& feedback,
                                 ros::Publisher& _pub_marker) {
    // Compute track variables
    double e, beta, gamma, section_length;
    computeEBetaGammaSectionLength(current_state,
                                   section,
                                   e,
                                   beta,
                                   gamma,
                                   section_length,
                                   _pub_marker);

    // Compute surge (linear interpolation)
    double desired_surge = section.initial_surge +
        gamma * (section.final_surge - section.initial_surge);

    // Compute depth (linear interpolation)
    double desired_depth = section.initial_position.z +
        gamma * (section.final_position.z - section.initial_position.z);

    // Compute yaw
    double desired_yaw = computeYaw(e,
                                    beta,
                                    current_state.velocity.linear.x,
                                    period);

    // Fill controller output (desired state)
    controller_output = control::State();
    controller_output.pose.position.depth = desired_depth;
    controller_output.pose.orientation.yaw = desired_yaw;
    controller_output.pose.disable_axis[2] = false;
    controller_output.pose.disable_axis[5] = false;
    controller_output.velocity.linear.x = desired_surge;
    controller_output.velocity.disable_axis[0] = false;

    // Fill feedback
    feedback = control::Feedback();
    feedback.desired_surge = desired_surge;
    feedback.desired_depth = desired_depth;
    feedback.desired_yaw = desired_yaw;
    feedback.cross_track_error = e;
    feedback.depth_error = desired_depth - current_state.pose.position.depth;
    feedback.yaw_error = cola2::util::normalizeAngle(desired_yaw - current_state.pose.orientation.yaw);
    feedback.distance_to_section_end = (1.0 - gamma) * section_length;
    feedback.success = false;
    if (feedback.distance_to_section_end < _config.acceptance_sec *
        fabs(current_state.velocity.linear.x)) feedback.success = true;

    // Debug info
    /*std::cout << "/dubins: initial_x = " << section.initial_x << std::endl;
    std::cout << "/dubins: initial_y = " << section.initial_y << std::endl;
    std::cout << "/dubins: initial_z = " << section.initial_z << std::endl;
    std::cout << "/dubins: initial_yaw = " << section.initial_yaw << std::endl;
    std::cout << "/dubins: initial_surge = " << section.initial_surge << std::endl;
    std::cout << "/dubins: final_x = " << section.final_x << std::endl;
    std::cout << "/dubins: final_y = " << section.final_y << std::endl;
    std::cout << "/dubins: final_z = " << section.final_z << std::endl;
    std::cout << "/dubins: final_yaw = " << section.final_yaw << std::endl;
    std::cout << "/dubins: final_surge = " << section.final_surge << std::endl;
    std::cout << "/dubins: e = " << e << std::endl;
    std::cout << "/dubins: beta = " << beta << std::endl;
    std::cout << "/dubins: gamma = " << gamma << std::endl;
    std::cout << "/dubins: section_length = " << section_length << std::endl;
    std::cout << "/dubins: desired_surge = " << desired_surge << std::endl;
    std::cout << "/dubins: desired_depth = " << desired_depth << std::endl;
    std::cout << "/dubins: desired_yaw = " << desired_yaw << std::endl;
    std::cout << "/dubins: distance_to_section_end = " << feedback.distance_to_section_end << std::endl;*/
}


void
DubinsSectionController::computeEBetaGammaSectionLength(
                            const control::State& current_state,
                            const control::Section& section,
                            double& e,
                            double& beta,
                            double& gamma,
                            double& section_length,
                            ros::Publisher& _pub_marker) {


    // Compute interesting variables
    double section_dx = (section.final_position.x - section.initial_position.x);
    double section_dy = (section.final_position.y - section.initial_position.y);

    // Check for repeated (x, y) points
    if ((fabs(section_dx) < 1e-2) && (fabs(section_dy) < 1e-2)) {
        reportError("Geometry error. Repeated (x, y)");
    }

    // Compute angle between final and initial section points
    double section_angle = atan2(section_dy, section_dx);

    // Check type of section to find direction
    int direction;
    if ((!section.use_initial_yaw) &&
        (!section.use_final_yaw)) {
        // Stright line
        direction = 0;
    }
    else if ((section.use_initial_yaw) &&
             (!section.use_final_yaw)) {
        // Find direction
        double angle_error = cola2::util::normalizeAngle(section.initial_yaw - section_angle);
        direction = 1;
        if (angle_error > 0.0) direction = -1;
        if (fabs(angle_error) < 0.0436) direction = 0;
    }
    else if ((!section.use_initial_yaw) &&
             (section.use_final_yaw)) {
        // Find direction
        double angle_error = cola2::util::normalizeAngle(section.final_yaw - section_angle);
        direction = -1;
        if (angle_error > 0.0) direction = 1;
        if (fabs(angle_error) < 0.0436) direction = 0;
    }
    else {
        // Not ready to process this section yet
        reportError("Not ready to process a section using both initial and final yaw");
    }

    // Follow section
    if (direction == 0) {  // Follow a line
        // Angle beta
        beta = section_angle;

        // Compute cross track error (e) and along track dinstance (s)
        double s, sbeta, cbeta, delta_x, delta_y;
        sbeta = sin(beta);
        cbeta = cos(beta);
        delta_x = current_state.pose.position.north - section.initial_position.x;
        delta_y = current_state.pose.position.east - section.initial_position.y;
        e = -delta_x * sbeta + delta_y * cbeta;
        s = delta_x * cbeta + delta_y * sbeta;

        // Compute gamma using along track distance
        section_length = sqrt(pow(section_dx, 2.0) + pow(section_dy, 2.0));
        gamma = 0.5;
        if (section_length > 0.0) {
            gamma = s / section_length;
            if (gamma < 0.0) gamma = 0.0;
            else if (gamma > 1.0) gamma = 1.0;
        }

        // Publish section markers TODO: this should be done in another place
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "/dubins";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point p1, p2;
        p1.x = section.initial_position.x;
        p1.y = section.initial_position.y;
        p1.z = section.initial_position.z;
        p2.x = section.final_position.x;
        p2.y = section.final_position.y;
        p2.z = section.final_position.z;
        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.scale.x = 0.35;
        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker.lifetime = ros::Duration(1.0);
        marker.frame_locked = false;
        _pub_marker.publish(marker);
    }
    else {  // Follow an arc
        // Find center
        double center_x, center_y;

        // Equation for the line in between both two section ends
        double ax, ay, bx, by;
        ax = 0.5 * (section.initial_position.x + section.final_position.x);
        ay = 0.5 * (section.initial_position.y + section.final_position.y);
        bx = ax - section_dy;
        by = ay + section_dx;
        double A = ay - by;
        double B = bx - ax;
        double C = ax * by - bx * ay;

        // Equation of the line perpendicular to the inital direction
        if (section.use_initial_yaw) {
            ax = section.initial_position.x;
            ay = section.initial_position.y;
            bx = ax - sin(section.initial_yaw);
            by = ay + cos(section.initial_yaw);
        }
        else {
            ax = section.final_position.x;
            ay = section.final_position.y;
            bx = ax - sin(section.final_yaw);
            by = ay + cos(section.final_yaw);
        }
        double D = ay - by;
        double E = bx - ax;
        double F = ax * by - bx * ay;

        // Find center at the intersection between both lines
        // Solve: Ax + By = -C
        //        Dx + Ey = -F
        double den = A * E - B * D;
        center_x = (B * F - E * C) / den;
        center_y = (D * C - A * F) / den;

        // Find radius
        double radius = sqrt(pow(center_x - section.initial_position.x, 2.0) +
                             pow(center_y - section.initial_position.y, 2.0));

        // Compute initial, actual and final angles. Same boundaries are
        // ensured by using the same wrapAngle function
        double angle_s = cola2::util::normalizeAngle(atan2(
            section.initial_position.y - center_y,
            section.initial_position.x - center_x));
        double angle = cola2::util::normalizeAngle(atan2(
            current_state.pose.position.east - center_y,
            current_state.pose.position.north - center_x));
        double angle_e = cola2::util::normalizeAngle(atan2(
            section.final_position.y - center_y,
            section.final_position.x - center_x));

        // Compute angle increments between [0 and 2*pi)
        double w_angle_e = wrapZeroToTwoPi(angle_e - angle_s);
        double w_angle_p = wrapZeroToTwoPi(angle - angle_s);

        // Check if the robot is inside or outside the arc, and compute gamma
        if ((w_angle_e == 0.0) or (w_angle_e == 2.0 * M_PI)) {
            // This should never happen if wrapZeroToTwoPi and the rest of the
            // code does what it has to do, but it is better to leave this
            // checking here
            reportError("Unexpected geometry error");
        }
        else if (direction == 1) {
            section_length = radius * w_angle_e;

            if (w_angle_e >= w_angle_p) {
                // Is in
                gamma = w_angle_p / w_angle_e;
            }
            else {
                if ((0.5 * w_angle_e + M_PI) >= w_angle_p) {
                    gamma = 1.0;
                    angle = angle_e;
                }
                else {
                    gamma = 0.0;
                    angle = angle_s;
                }
            }
        }
        else {  // Direction is -1
            section_length = radius * (2*M_PI - w_angle_e);

            if (w_angle_e <= w_angle_p) {
                // Is in
                gamma = (2*M_PI - w_angle_p) / (2*M_PI - w_angle_e);
            }
            else {
                if (0.5 * w_angle_e > w_angle_p) {
                    gamma = 0.0;
                    angle = angle_s;
                }
                else {
                    gamma = 1.0;
                    angle = angle_e;
                }
            }
        }

        // Compute beta with lookahead
        double surge = current_state.velocity.linear.x;
        if (surge < 0.01) surge = 0.01;
        double lookahead_m = _config.lookahead_sec * surge;
        double lookahead_rad = lookahead_m / radius;
        if (lookahead_rad > M_PI) lookahead_rad = M_PI;
        double direction_d = static_cast<double>(direction);
        if (direction == 1) {
            beta = cola2::util::normalizeAngle(angle + 0.5 * M_PI * direction_d + lookahead_rad);
        }
        else {
            beta = cola2::util::normalizeAngle(angle + 0.5 * M_PI * direction_d - lookahead_rad);
        }

        // Nearest point in the arc
        double nearest_north = center_x + radius * cos(angle);
        double nearest_east = center_y + radius * sin(angle);

        // Compute cross track error (e)
        e = -(current_state.pose.position.north - nearest_north) * sin(beta) +
             (current_state.pose.position.east - nearest_east) * cos(beta);

        // Publish section markers TODO: this should be done in another place
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "/dubins";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        double delta_angle = w_angle_e;
        if (direction == -1) delta_angle = 2*M_PI - delta_angle;

        int pieces = 36;
        for (int i = 0; i < pieces; i++) {
            double ax = center_x + radius * cos(angle_s + direction * delta_angle * i / pieces);
            double ay = center_y + radius * sin(angle_s + direction * delta_angle * i / pieces);
            double bx = center_x + radius * cos(angle_s + direction * delta_angle * (i + 1) / pieces);
            double by = center_y + radius * sin(angle_s + direction * delta_angle * (i + 1) / pieces);

            geometry_msgs::Point p1, p2;
            p1.x = ax;
            p1.y = ay;
            p1.z = section.initial_position.z + (i / pieces) * (section.final_position.z - section.initial_position.z);
            p2.x = bx;
            p2.y = by;
            p2.z = section.initial_position.z + ((i + 1) / pieces) * (section.final_position.z - section.initial_position.z);
            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }

        marker.scale.x = 0.35;
        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker.lifetime = ros::Duration(1.0);
        marker.frame_locked = false;
        _pub_marker.publish(marker);
    }
}


double
DubinsSectionController::computeYaw(double e,
                                    double beta,
                                    double surge,
                                    double period) {
    // Saturate surge to avoid numerical problems
    if (surge < 0.01) surge = 0.01;

    // Compute psi using a PI controller
    double e_dot = (e - _yaw_old_e) / period;
    double psi_dot = -(_config.yaw_ki * e + _config.yaw_kp * e_dot) / surge;
    double psi = _yaw_old_psi + psi_dot * period;  // - 0.01 * e_dot;
    if (psi > 1.0) psi = 1.0;
    else if (psi < -1.0) psi = -1.0;

    // Compute yaw setpoint
    double yaw = cola2::util::normalizeAngle(beta + asin(psi));

    // Update old variables
    _yaw_old_e = e;
    _yaw_old_psi = psi;

    return yaw;
}

double
DubinsSectionController::wrapZeroToTwoPi(double angle) {
    // Wrap angle from [0 to 2*pi)
    angle = cola2::util::normalizeAngle(angle);
    if (angle < 0.0) angle += 2*M_PI;
    return angle;
}


void
DubinsSectionController::setConfig(const DubinsSectionControllerConfig& config) {
    // Copy input config
    _config = config;
}

#endif  /* __CONTROLLER_DUBINS__ */
