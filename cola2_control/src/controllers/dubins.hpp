#ifndef __CONTROLLER_DUBINS__
#define __CONTROLLER_DUBINS__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "types.hpp"


typedef struct {
    double yaw_ki;
    double yaw_kp;
} DubinsSectionControllerConfig;


// Constant definitions
#define CT_PI 3.141592653589793238462643383279502884197169399375105820974
#define CT_2PI 6.283185307179586476925286766559005768394338798750211641949


class DubinsSectionController {
private:
    // Config
    DubinsSectionControllerConfig _config;

    // Other vars
    double _yaw_old_e, _yaw_old_psi;

    // Methods
    void reportError(const std::string&);
    double computeYaw(double, double, double, double);
    void computeEBetaGammaSectionLength(const control::State&,
                                        const control::Section&,
                                        double&, double&, double&, double&);
    double wrapAngle(double);
    double wrapZeroToTwoPi(double);

public:
    // Methods
    DubinsSectionController();
    void compute(const control::State&,
                 const control::Section&,
                 double period,
                 control::State&,
                 control::Feedback&);
    void setConfig(const DubinsSectionControllerConfig&);
};


DubinsSectionController::DubinsSectionController() {
    // Default config
    _config.yaw_ki = 0.0;//0.006;
    _config.yaw_kp = 0.09;//0.12;

    // Init some vars
    _yaw_old_psi = _yaw_old_e = 0.0;
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
                                 control::Feedback& feedback) {
    // Compute track variables
    double e, beta, gamma, section_length;
    computeEBetaGammaSectionLength(current_state,
                                   section,
                                   e,
                                   beta,
                                   gamma,
                                   section_length);

    // Compute surge (linear interpolation)
    double desired_surge = section.initial_surge +
        gamma * (section.final_surge - section.initial_surge);

    // Compute depth (linear interpolation)
    double desired_depth = section.initial_z +
        gamma * (section.final_z - section.initial_z);

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
    feedback.yaw_error = wrapAngle(desired_yaw - current_state.pose.orientation.yaw);
    feedback.distance_to_section_end = (1.0 - gamma) * section_length;
    feedback.success = false;
    if (feedback.distance_to_section_end <
        0.5 * fabs(current_state.velocity.linear.x)) feedback.success = true;

    // Debug info
    std::cout << "/dubins: initial_x = " << section.initial_x << std::endl;
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
    std::cout << "/dubins: distance_to_section_end = " << feedback.distance_to_section_end << std::endl;
}


void
DubinsSectionController::computeEBetaGammaSectionLength(
                            const control::State& current_state,
                            const control::Section& section,
                            double& e,
                            double& beta,
                            double& gamma,
                            double& section_length) {


    // Compute interesting variables
    double section_dx = (section.final_x - section.initial_x);
    double section_dy = (section.final_y - section.initial_y);

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
        double angle_error = wrapAngle(section.initial_yaw - section_angle);
        direction = 1;
        if (angle_error > 0.0) direction = -1;
        if (fabs(angle_error) < 0.0436) direction = 0;
    }
    else if ((!section.use_initial_yaw) &&
             (section.use_final_yaw)) {
        // Find direction
        double angle_error = wrapAngle(section.final_yaw - section_angle);
        direction = -1;
        if (angle_error > 0.0) direction = 1;
        if (fabs(angle_error) < 0.0436) direction = 0;
    }
    else {
        // Not ready to process this section yet
        reportError("Not ready to process a section using both initial and final yaw");
    }

    std::cout << "/dubins: direction = " << direction << std::endl;

    // Follow section
    if (direction == 0) {  // Follow a line
        // Angle beta
        beta = section_angle;

        // Compute cross track error (e) and along track dinstance (s)
        double s, sbeta, cbeta, delta_x, delta_y;
        sbeta = sin(beta);
        cbeta = cos(beta);
        delta_x = current_state.pose.position.north - section.initial_x;
        delta_y = current_state.pose.position.east - section.initial_y;
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
    }
    else {  // Follow an arc
        // Find center
        double center_x, center_y;

        // Equation for the line in between both two section ends
        double ax, ay, bx, by;
        ax = 0.5 * (section.initial_x + section.final_x);
        ay = 0.5 * (section.initial_y + section.final_y);
        bx = ax - section_dy;
        by = ay + section_dx;
        double A = ay - by;
        double B = bx - ax;
        double C = ax * by - bx * ay;

        // Equation of the line perpendicular to the inital direction
        if (section.use_initial_yaw) {
            ax = section.initial_x;
            ay = section.initial_y;
            bx = ax + sin(section.initial_yaw);
            by = ay + cos(section.initial_yaw);
        }
        else {
            ax = section.final_x;
            ay = section.final_y;
            bx = ax + sin(section.final_yaw);
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

        std::cout << "/dubins: center_x = " << center_x << std::endl;
        std::cout << "/dubins: center_y = " << center_y << std::endl;

        // Find radius
        double radius = sqrt(pow(center_x - section.initial_x, 2.0) +
                             pow(center_y - section.initial_y, 2.0));

        std::cout << "/dubins: radius  = " << radius << std::endl;
        std::cout << "/dubins: radius2 = " << sqrt(pow(center_x - section.final_x, 2.0) + pow(center_y - section.final_y, 2.0)) << std::endl;

        // Compute initial, actual and final angles. Same boundaries are
        // ensured by using the same wrapAngle function
        double angle_s = wrapAngle(atan2(
            section.initial_y - center_y,
            section.initial_x - center_x));
        double angle = wrapAngle(atan2(
            current_state.pose.position.east - center_y,
            current_state.pose.position.north - center_x));
        double angle_e = wrapAngle(atan2(
            section.final_y - center_y,
            section.final_x - center_x));

        // Compute angle increments between [0 and 2*pi)
        double w_angle_e = wrapZeroToTwoPi(angle_e - angle_s);
        double w_angle_p = wrapZeroToTwoPi(angle - angle_s);

        // Check if the robot is inside or outside the arc, and compute gamma
        if ((w_angle_e == 0.0) or (w_angle_e == 2.0 * CT_PI)) {
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
                if ((0.5 * w_angle_e + CT_PI) >= w_angle_p) {
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
            section_length = radius * (CT_2PI - w_angle_e);

            if (w_angle_e <= w_angle_p) {
                // Is in
                gamma = (CT_2PI - w_angle_p) / (CT_2PI - w_angle_e);
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

        // Precompute cosine and sine of the angle
        double c_angle, s_angle;
        c_angle = cos(angle);
        s_angle = sin(angle);

        // Compute vector to follow and beta
        double vtf_north = -static_cast<double>(direction) * s_angle;
        double vtf_east = static_cast<double>(direction) * c_angle;
        beta = atan2(vtf_east, vtf_north);

        std::cout << "/dubins: beta2      = " << wrapAngle(angle + 0.5 * CT_PI * static_cast<double>(direction)) << std::endl;
        std::cout << "/dubins: beta_start = " << wrapAngle(angle_s + 0.5 * CT_PI * static_cast<double>(direction)) << std::endl;
        std::cout << "/dubins: beta_end   = " << wrapAngle(angle_e + 0.5 * CT_PI * static_cast<double>(direction)) << std::endl;

        // Nearest point in the arc
        double nearest_north = center_x + radius * c_angle;
        double nearest_east = center_y + radius * s_angle;

        // Compute cross track error (e)
        e = -(current_state.pose.position.north - nearest_north) * sin(beta) +
             (current_state.pose.position.east - nearest_east) * cos(beta);
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
    double psi = _yaw_old_psi + psi_dot * period;

    psi = - e * _config.yaw_kp / surge;

    if (psi > 1.0) psi = 1.0;
    else if (psi < -1.0) psi = -1.0;

    std::cout << "/dubins: correction = " << asin(psi) * 180 / CT_PI << std::endl;

    // Compute yaw setpoint
    double yaw = wrapAngle(beta + asin(psi));

    // Update old variables
    _yaw_old_e = e;
    _yaw_old_psi = psi;

    return yaw;
}


double
DubinsSectionController::wrapAngle(double angle) {
    // Wrap angle in the range of (-pi to +pi]
    if (fabs(angle) > 20.0) angle = atan2(sin(angle), cos(angle));
    while (angle <= -CT_PI) angle += CT_2PI;
    while (angle > CT_PI) angle -= CT_2PI;
    return angle;
}


double
DubinsSectionController::wrapZeroToTwoPi(double angle) {
    // Wrap angle from [0 to 2*pi)
    angle = wrapAngle(angle);
    if (angle < 0.0) angle += CT_2PI;
    return angle;
}


void
DubinsSectionController::setConfig(const DubinsSectionControllerConfig& config) {
    // Copy input config
    _config = config;
}

#endif  /* __CONTROLLER_DUBINS__ */
