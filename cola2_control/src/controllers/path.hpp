#ifndef __CONTROLLER_PATH__
#define __CONTROLLER_PATH__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "types.hpp"
#include <math.h>
#include <cola2_lib/cola2_util.h>

typedef struct {
    double surge_speed;
    double lookahead;
    double tolerance;
} PathControllerConfig;


class PathController {
public:
    // Methods
    PathController(const PathControllerConfig&);
    void compute(const control::State&,
                 control::Path&,
                 double period,
                 control::State&,
                 control::Feedback&,
                 control::PointsList&);
    void setConfig(const PathControllerConfig&);
    void sectionListToPath(control::Path&, const std::vector<control::Section>&);

private:
    // Config
    PathControllerConfig _config;

    // Methods
    void reportError(const std::string&);
    double wrapZeroToTwoPi(double);
};


PathController::PathController(const PathControllerConfig &config) {
    _config = config;
}


void
PathController::reportError(const std::string& message) {
    throw std::runtime_error("/path_controller: " + message);
}


void
PathController::compute(const control::State& current_state,
                        control::Path& path,
                        double period,
                        control::State& controller_output,
                        control::Feedback& feedback,
                        control::PointsList& marker) {
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

    // Copy lookahead parameter
    double lookahead = _config.lookahead;

    // Find closest section
    int section_i;
    double section_s;
    double total_s;
    double min_dist = 1e50;
    double previous_total_s = 0.0;
    for (int i = 0; i < path.data.size() - 1; i++) {
        // Project robot position into the section
        double beta, length, sbeta, cbeta, delta_x, delta_y, e, s, dist;
        beta = atan2(path.data[i + 1][1] - path.data[i][1],
                     path.data[i + 1][0] - path.data[i][0]);
        length = sqrt(pow(path.data[i + 1][1] - path.data[i][1], 2.0) +
                      pow(path.data[i + 1][0] - path.data[i][0], 2.0));
        sbeta = sin(beta);
        cbeta = cos(beta);
        delta_x = current_state.pose.position.north - path.data[i][0];
        delta_y = current_state.pose.position.east  - path.data[i][1];
        e = -delta_x * sbeta + delta_y * cbeta;
        s = delta_x * cbeta + delta_y * sbeta;

        // Compute distance to the section
        if (s < 0.0) {
            dist = sqrt(pow(current_state.pose.position.east  - path.data[i][1], 2.0) +
                        pow(current_state.pose.position.north - path.data[i][0], 2.0));
        }
        else if (s > length) {
            dist = sqrt(pow(current_state.pose.position.east  - path.data[i + 1][1], 2.0) +
                        pow(current_state.pose.position.north - path.data[i + 1][0], 2.0));
        }
        else dist = fabs(e);

        // Keep track of minimum distance
        if (dist < min_dist) {
            min_dist = dist;
            section_i = i;
            section_s = s;
            total_s = previous_total_s + s;
        }

        previous_total_s += length;
    }

    // See if projected point is close
    if (fabs(total_s - path.old_total_s) > lookahead + 5.0) {
        section_i = path.old_section_i;
        section_s = path.old_section_s;
    }
    else {
        path.old_total_s = total_s;
        path.old_section_i = section_i;
        path.old_section_s = section_s;
    }

    // Compute lookahead through path
    int i = section_i;
    double s = section_s;
    std::vector<double> goal;
    while ((goal.size() == 0) && (i < path.data.size() - 1)) {
        // Section length
        double length = sqrt(pow(path.data[i + 1][1] - path.data[i][1], 2.0) +
                             pow(path.data[i + 1][0] - path.data[i][0], 2.0));

        // Compute desired s
        double desired_s = s + lookahead;
        if (desired_s > length) {
            lookahead = lookahead - (length - s);
            i += 1;
            s = 0.0;
        }
        else {
            // Project desired_s
            double beta = atan2(path.data[i + 1][1] - path.data[i][1],
                                path.data[i + 1][0] - path.data[i][0]);
            goal.push_back(path.data[i][0] + desired_s * cos(beta));
            goal.push_back(path.data[i][1] + desired_s * sin(beta));
            if (length < 0.1) {
                goal.push_back(path.data[i][2]);
            }
            else {
                goal.push_back(path.data[i][2] + desired_s / length *
                    (path.data[i + 1][2] - path.data[i][2]));
            }
        }
    }
    if (goal.size() == 0) {
        // If lookahead is greater that remaining path, goal is end of path
        goal.push_back(path.data.back()[0]);
        goal.push_back(path.data.back()[1]);
        goal.push_back(path.data.back()[2]);
    }

    // Desired yaw points towards intermediate goal
    double desired_yaw = atan2(goal[1] - current_state.pose.position.east,
                               goal[0] - current_state.pose.position.north);

    // Surge speed comes from the config
    double desired_surge = _config.surge_speed;

    // Compute depth as linear interpolation (already done)
    double desired_depth = goal[2];

    // Fill controller output (desired state)
    controller_output = control::State();
    controller_output.pose.position.depth = desired_depth;
    controller_output.pose.orientation.yaw = desired_yaw;
    controller_output.pose.disable_axis.z = false;
    controller_output.pose.disable_axis.yaw = false;
    controller_output.velocity.linear.x = desired_surge;
    controller_output.velocity.disable_axis.x = false;

    // Fill feedback
    feedback = control::Feedback();
    feedback.desired_surge = desired_surge;
    feedback.desired_depth = desired_depth;
    feedback.desired_yaw = desired_yaw;
    feedback.cross_track_error = min_dist;  // TODO: improve this
    feedback.depth_error = desired_depth - current_state.pose.position.depth;
    feedback.yaw_error = cola2::util::normalizeAngle(desired_yaw - current_state.pose.orientation.yaw);

    // Check if finished
    double dist_to_path_end = sqrt(pow(path.data.back()[0] - current_state.pose.position.north, 2.0) +
                                   pow(path.data.back()[1] - current_state.pose.position.east, 2.0)+
                                   pow(path.data.back()[2] - current_state.pose.position.depth, 2.0));
    feedback.distance_to_end = dist_to_path_end;  // TODO: improve this
    feedback.success = false;
    if (dist_to_path_end < _config.tolerance) {
        feedback.success = true;
        path.data.clear();
        path.old_total_s = 0.0;
        path.old_section_i = 0;
        path.old_section_s = 0.0;
        return;
    }

    // Points for visualization
    for (int i = 0; i < path.data.size() - 1; i++) {
        control::point p1, p2;
        p1.x = path.data[i][0];
        p1.y = path.data[i][1];
        p1.z = path.data[i][2];
        p2.x = path.data[i + 1][0];
        p2.y = path.data[i + 1][1];
        p2.z = path.data[i + 1][2];
        marker.points_list.push_back(p1);
        marker.points_list.push_back(p2);
    }
}


void
PathController::sectionListToPath(control::Path& path,
                                  const std::vector<control::Section>& section_list) {
    // Clear path
    path.data.clear();
    path.old_total_s = 0.0;
    path.old_section_i = 0;
    path.old_section_s = 0.0;

    for (int k = 0; k < section_list.size(); k++) {
        // Extract section
        control::Section section = section_list[k];

        // Compute interesting variables
        double section_dx, section_dy;
        section_dx = (section.final_position.x - section.initial_position.x);
        section_dy = (section.final_position.y - section.initial_position.y);

        // Check for repeated (x, y) points
        if ((fabs(section_dx) < 0.1) && (fabs(section_dy) < 0.1)) {
            std::vector<double> point;
            point.push_back(section.initial_position.x);
            point.push_back(section.initial_position.y);
            point.push_back(section.initial_position.z);
            path.data.push_back(point);
            continue;
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
            // Not ready to process this section yet. Act as if it was a line
            // and display a warning
            ROS_WARN_STREAM("/path_controller: not ready to process a section " <<
                "using both initial and final yaw");
            std::vector<double> point;
            point.push_back(section.initial_position.x);
            point.push_back(section.initial_position.y);
            point.push_back(section.initial_position.z);
            path.data.push_back(point);
            continue;
        }

        // Follow section
        if (direction == 0) {  // Follow a line
            std::vector<double> point;
            point.push_back(section.initial_position.x);
            point.push_back(section.initial_position.y);
            point.push_back(section.initial_position.z);
            path.data.push_back(point);
            continue;
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
            // This should never fail if previous checkings are performed correctly
            double den_inv = 1.0 / (A * E - B * D);
            center_x = (B * F - E * C) * den_inv;
            center_y = (D * C - A * F) * den_inv;

            // Find radius
            double radius = sqrt(pow(center_x - section.initial_position.x, 2.0) +
                                 pow(center_y - section.initial_position.y, 2.0));

            // Compute initial, actual and final angles. Same boundaries are
            // ensured by using the same wrapAngle function
            double angle_s = cola2::util::normalizeAngle(atan2(
                section.initial_position.y - center_y,
                section.initial_position.x - center_x));
            double angle_e = cola2::util::normalizeAngle(atan2(
                section.final_position.y - center_y,
                section.final_position.x - center_x));

            // Compute angle increments between [0 and 2*pi)
            double w_angle_e = wrapZeroToTwoPi(angle_e - angle_s);

            // Compute angle delta
            double delta_angle = w_angle_e;
            if (direction == -1) delta_angle = 2.0 * M_PI - delta_angle;

            // Compute amount of pieces according to the angle delta
            int pieces = ceil(delta_angle / (M_PI / 18.0));

            // Cut curve in straight line pieces and put them into _path
            for (int i = 0; i < pieces; i++) {
                double ax = center_x + radius *
                    cos(angle_s + direction * delta_angle * i / pieces);
                double ay = center_y + radius *
                    sin(angle_s + direction * delta_angle * i / pieces);
                double az = section.initial_position.z + i / pieces *
                    (section.final_position.z - section.initial_position.z);

                std::vector<double> point;
                point.push_back(ax);
                point.push_back(ay);
                point.push_back(az);
                path.data.push_back(point);
            }
        }
    }

    // Add last point but check that at least there is a section in the path
    if (path.data.size() > 0) {
        int i = section_list.size() - 1;
        std::vector<double> point;
        point.push_back(section_list[i].final_position.x);
        point.push_back(section_list[i].final_position.y);
        point.push_back(section_list[i].final_position.z);
        path.data.push_back(point);
    }
}


double
PathController::wrapZeroToTwoPi(double angle) {
    // Wrap angle from [0 to 2*pi)
    angle = cola2::util::normalizeAngle(angle);
    if (angle < 0.0) angle += 2 * M_PI;
    return angle;
}


void
PathController::setConfig(const PathControllerConfig& config) {
    // Copy input config
    _config = config;
}

#endif  /* __CONTROLLER_PATH__ */
