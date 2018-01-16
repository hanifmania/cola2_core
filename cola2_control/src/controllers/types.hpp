
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_TYPES__
#define __CONTROLLER_TYPES__

#include <vector>
#include <string>


namespace control {
    typedef struct {
        double x;
        double y;
        double z;
    } point;

    typedef struct {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    } vector6d;

    typedef struct {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        double altitude;
    } Nav;

    typedef struct {
        double roll;
        double pitch;
        double yaw;
    } rpy;

    typedef struct {
        double north;
        double east;
        double depth;
    } ned;


    class PointsList {
    public:
        std::vector<control::point> points_list;
        PointsList() {}
    };


    class Waypoint {
    public:
        std::string requester;
        unsigned int priority;
        bool altitude_mode;
        control::ned position;
        double altitude;
        control::rpy orientation;
        control::point position_tolerance;
        control::rpy orientation_tolerance;
        control::point linear_velocity;
        control::rpy angular_velocity;
        unsigned int controller_type;
        unsigned int timeout;
        control::vector6d disable_axis;
		bool keep_position;

        Waypoint() {
            requester = "requester_not_defined";
            priority = 0;
            altitude_mode = false;
            position.north = 0.0;
            position.east = 0.0;
            position.depth = 0.0;
            altitude = 100.0;
            orientation.roll = 0.0;
            orientation.pitch = 0.0;
            orientation.yaw = 0.0;
            position_tolerance.x = 0.0;
            position_tolerance.y = 0.0;
            position_tolerance.z = 0.0;
            orientation_tolerance.roll = 0.0;
            orientation_tolerance.pitch = 0.0;
            orientation_tolerance.yaw = 0.0;
            controller_type = 0;  // GOTO
            timeout = 0;
            disable_axis.x = true;
            disable_axis.y = true;
            disable_axis.z = true;
            disable_axis.roll = true;
            disable_axis.pitch = true;
            disable_axis.yaw = true;
            linear_velocity.x = 0.0;
            linear_velocity.y = 0.0;
            linear_velocity.z = 0.0;
            angular_velocity.roll = 0.0;
            angular_velocity.pitch = 0.0;
            angular_velocity.yaw = 0.0;
            keep_position = false;
        }
    };


    class Section {
    public:
        // Initial state
        control::point initial_position;
        // Only for DUBINS (not for LOS)
        double initial_yaw;
        double initial_surge;
        bool use_initial_yaw;

        // Final state
        control::point final_position;
        // Only for DUBINS (not for LOS)
        double final_yaw;
        double final_surge;
        bool use_final_yaw;

        // Flag to consider z as altitude
        bool altitude_mode;

        // Only for LOS (Not DUBINS)
        // Flag to not control z axis
        bool disable_z;
        // Tolerance (LOS only uses position tolerance)
        control::point tolerance;

        // Constructor with default values
        Section() {
            initial_position.x = 0.0;
            initial_position.y = 0.0;
            initial_position.z = 0.0;
            initial_yaw = 0.0;
            initial_surge = 0.0;
            use_initial_yaw = false;
            final_position.x = 0.0;
            final_position.y = 0.0;
            final_position.z = 0.0;
            final_yaw = 0.0;
            final_surge = 0.0;
            use_final_yaw = false;
            altitude_mode = false;
            disable_z = false;
            tolerance.x = 0.0;
            tolerance.y = 0.0;
            tolerance.z = 0.0;
        }
    };


    class Path {
    public:
        std::vector<std::vector<double> > data;
        double old_total_s, old_section_s;
        int  old_section_i;

        // Constructor with default values
        Path() {
            old_total_s = 0.0;
            old_section_i = 0;
            old_section_s = 0.0;
        }
    };


    class State {
    public:
        // Position
        struct {
            control::ned position;
            control::rpy orientation;
            control::vector6d disable_axis;
            double altitude;
            bool altitude_mode;
        } pose;

        // Velocity
        struct {
            control::point linear;
            control::point angular;
            control::vector6d disable_axis;
        } velocity;

        // Constructor with default values
        State() {
            pose.position.north = 0.0;
            pose.position.east  = 0.0;
            pose.position.depth = 0.0;
            pose.orientation.roll  = 0.0;
            pose.orientation.pitch = 0.0;
            pose.orientation.yaw   = 0.0;
            pose.altitude = 100000.0;  // Just in case
            pose.altitude_mode = false;
            pose.disable_axis.x = true;
            pose.disable_axis.y = true;
            pose.disable_axis.z = true;
            pose.disable_axis.roll = true;
            pose.disable_axis.pitch = true;
            pose.disable_axis.yaw = true;
            velocity.linear.x = 0.0;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;
            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;
            velocity.disable_axis.x = 0.0;
            velocity.disable_axis.y = 0.0;
            velocity.disable_axis.z = 0.0;
            velocity.disable_axis.roll = 0.0;
            velocity.disable_axis.pitch = 0.0;
            velocity.disable_axis.yaw = 0.0;
        }
    };


    class Feedback {
    public:
        // Current output of the controller
        double desired_surge;
        double desired_depth;
        double desired_yaw;

        // Errors
        double cross_track_error;
        double depth_error;
        double yaw_error;
        double distance_to_end;

        // Success
        bool success;

        Feedback() {
            desired_surge = 0.0;
            desired_depth = 0.0;
            desired_yaw = 0.0;
            cross_track_error = 0.0;
            depth_error = 0.0;
            yaw_error = 0.0;
            distance_to_end = 0.0;
            success = false;
        }
    };
}

#endif  /* __CONTROLLER_TYPES__ */
