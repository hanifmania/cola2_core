#ifndef __CONTROLLER_TYPES__
#define __CONTROLLER_TYPES__

#include <vector>


namespace control {
    class SectionMarker {
    public:
        struct {
            double x;
            double y;
            double z;
        } initial_point;

        struct {
            double x;
            double y;
            double z;
        } final_point;

        SectionMarker() {
            initial_point.x = 0.0;
            initial_point.y = 0.0;
            initial_point.z = 0.0;
            final_point.x = 0.0;
            final_point.y = 0.0;
            final_point.z = 0.0;
        }
    };


    class Section {
    public:
        // Initial state
        struct {
            double x, y, z;
        } initial_position;
        double initial_yaw;
        double initial_surge;
        bool use_initial_yaw;

        // Final state
        struct {
            double x, y, z;
        } final_position;
        double final_yaw;
        double final_surge;
        bool use_final_yaw;

        // Flag to consider z as altitude
        bool altitude_mode;

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
        }
    };


    class State {
    public:
        // Position
        struct {
            struct {
                double north, east, depth;
            } position;
            struct {
                double roll, pitch, yaw;
            } orientation;
            bool disable_axis[6];
            double altitude;
            bool altitude_mode;
        } pose;

        // Velocity
        struct {
            struct {
                double x, y, z;
            } linear;
            struct {
                double x, y, z;
            } angular;
            bool disable_axis[6];
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
            pose.disable_axis[0] = true;
            pose.disable_axis[1] = true;
            pose.disable_axis[2] = true;
            pose.disable_axis[3] = true;
            pose.disable_axis[4] = true;
            pose.disable_axis[5] = true;
            velocity.linear.x = 0.0;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;
            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;
            velocity.disable_axis[0] = 0.0;
            velocity.disable_axis[1] = 0.0;
            velocity.disable_axis[2] = 0.0;
            velocity.disable_axis[3] = 0.0;
            velocity.disable_axis[4] = 0.0;
            velocity.disable_axis[5] = 0.0;
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
        double distance_to_section_end;

        // Success
        bool success;

        Feedback() {
            desired_surge = 0.0;
            desired_depth = 0.0;
            desired_yaw = 0.0;
            cross_track_error = 0.0;
            depth_error = 0.0;
            yaw_error = 0.0;
            distance_to_section_end = 0.0;
            success = false;
        }
    };
}

#endif  /* __CONTROLLER_TYPES__ */
