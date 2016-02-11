#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <actionlib/server/simple_action_server.h>
#include <cola2_msgs/SectionAction.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/WorldWaypointReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <visualization_msgs/Marker.h>
#include "controllers/types.hpp"
#include "controllers/dubins.hpp"


class Pilot {
private:
    // Node handle
    ros::NodeHandle _nh;

    // Node name
    std::string _node_name;

    // ROS variables
    ros::Subscriber _sub_nav;
    ros::Publisher _pub_wwr, _pub_bvr, _pub_marker;

    // Actionlib servers
    boost::shared_ptr<actionlib::SimpleActionServer<
        cola2_msgs::SectionAction> > _section_server;

    // Other vars
    control::State _current_state;

    // Controllers
    DubinsSectionController _dubins_controller;

    // Config
    struct {
        double rate;
        std::string section_server_name;
    } _config;

    // Methods
    void navCallback(const auv_msgs::NavSts&);
    void sectionServerCallback(const cola2_msgs::SectionGoalConstPtr&);
    void publishControlCommands(const control::State&, unsigned int);
    void publishFeedback(const control::Feedback&);
    void publishMarker(double, double, double);
    void getConfig();
    template<typename T> void getParam(std::string, T&);

public:
    Pilot();
};


Pilot::Pilot() {
    // Node name
    _node_name = ros::this_node::getName();

    // Get config
    getConfig();

    // Publishers
    _pub_wwr = _nh.advertise<auv_msgs::WorldWaypointReq>(
        "/cola2_control/world_waypoint_req", 1);
    _pub_bvr = _nh.advertise<auv_msgs::BodyVelocityReq>(
        "/cola2_control/body_velocity_req", 1);
    _pub_marker = _nh.advertise<visualization_msgs::Marker>(
        "/cola2_control/waypoint_marker", 1);

    // Subscriber
    _sub_nav = _nh.subscribe("/cola2_navigation/nav_sts", 1,
        &Pilot::navCallback, this);

    // Actionlib server. Smart pointer is used so that server construction is
    // delayed after configuration is loaded
    _section_server = boost::shared_ptr<actionlib::SimpleActionServer<
        cola2_msgs::SectionAction> >(
        new actionlib::SimpleActionServer<cola2_msgs::SectionAction>(
        _nh, _config.section_server_name,
        boost::bind(&Pilot::sectionServerCallback, this, _1), false));
    _section_server->start();

    // Display message
    ROS_INFO_STREAM(_node_name << ": initialized");
}


void
Pilot::navCallback(const auv_msgs::NavSts& data) {
    // Obtain navigation data
    _current_state.pose.position.north    = data.position.north;
    _current_state.pose.position.east     = data.position.east;
    _current_state.pose.position.depth    = data.position.depth;
    _current_state.pose.altitude          = data.altitude;
    _current_state.pose.orientation.roll  = data.orientation.roll;
    _current_state.pose.orientation.pitch = data.orientation.pitch;
    _current_state.pose.orientation.yaw   = data.orientation.yaw;
    _current_state.velocity.linear.x      = data.body_velocity.x;
    _current_state.velocity.linear.y      = data.body_velocity.y;
    _current_state.velocity.linear.z      = data.body_velocity.z;
}


void
Pilot::sectionServerCallback(const cola2_msgs::SectionGoalConstPtr& data) {
    // Conversion from actionlib goal to internal Section type
    control::Section section;
    section.initial_x       = data->initial_x;
    section.initial_y       = data->initial_y;
    section.initial_z       = data->initial_z;
    section.initial_yaw     = data->initial_yaw;
    section.initial_surge   = data->initial_surge;
    section.use_initial_yaw = data->use_initial_yaw;
    section.final_x         = data->final_x;
    section.final_y         = data->final_y;
    section.final_z         = data->final_z;
    section.final_yaw       = data->final_yaw;
    section.final_surge     = data->final_surge;
    section.use_final_yaw   = data->use_final_yaw;
    section.use_altitude    = data->use_altitude;

    // Main loop
    double init_time = ros::Time::now().toSec();
    ros::Rate r(_config.rate);
    while (ros::ok()) {
        // Declare some vars
        control::State controller_output;
        control::Feedback feedback;
        cola2_msgs::SectionResult result_msg;

        // Run controller
        try {
            switch (data->controller_type) {
                case cola2_msgs::SectionGoal::DUBINS:
                    ROS_DEBUG_STREAM(_node_name << ": DUBINS controller");
                    _dubins_controller.compute(_current_state,
                                               section,
                                               1.0 / _config.rate,
                                               controller_output,
                                               feedback);
                    break;
                default:
                    throw std::runtime_error("Unknown controller");
            }
        }
        catch (std::exception& e) {
            // Check for failure
            ROS_ERROR_STREAM(_node_name << ": controller failure\n" << e.what());
            result_msg.final_status = cola2_msgs::SectionResult::FAILURE;
            _section_server->setAborted(result_msg);
            break;
        }

        // Publish control commands
        publishControlCommands(controller_output, data->priority);

        // Publish actionlib feedback
        publishFeedback(feedback);

        // Publish marker
        publishMarker(section.final_x, section.final_y, section.final_z);

        // Check for success
        if (feedback.success) {
            ROS_INFO_STREAM(_node_name << ": section success");
            result_msg.final_status = cola2_msgs::SectionResult::SUCCESS;
            _section_server->setSucceeded(result_msg);
            break;
        }

        // Check for preempted. This happens upon user request (by preempting
        // or cancelling the goal, or when a new SectionGoal is received
        if (_section_server->isPreemptRequested()) {
            ROS_WARN_STREAM(_node_name << ": section preempted");
            _section_server->setPreempted();
            break;
        }

        // Check for timeout
        if (data->timeout > 0.0) {
            if ((ros::Time::now().toSec() - init_time) > data->timeout) {
                ROS_WARN_STREAM(_node_name << ": section timeout");
                result_msg.final_status = cola2_msgs::SectionResult::TIMEOUT;
                _section_server->setAborted(result_msg);
                break;
            }
        }

        // Sleep
        r.sleep();
    }
}


void
Pilot::publishControlCommands(const control::State& controller_output,
                              unsigned int priority) {
    // Get time
    ros::Time now = ros::Time::now();

    // Create ROS msgs for wwr
    auv_msgs::WorldWaypointReq wwr;
    wwr.header.frame_id = _node_name + "_pose";
    wwr.header.stamp = now;
    wwr.goal.priority = priority;
    wwr.goal.requester = _node_name + "_pose";
    wwr.disable_axis.x     = controller_output.pose.disable_axis[0];
    wwr.disable_axis.y     = controller_output.pose.disable_axis[1];
    wwr.disable_axis.z     = controller_output.pose.disable_axis[2];
    wwr.disable_axis.roll  = controller_output.pose.disable_axis[3];
    wwr.disable_axis.pitch = controller_output.pose.disable_axis[4];
    wwr.disable_axis.yaw   = controller_output.pose.disable_axis[5];
    wwr.position.north     = controller_output.pose.position.north;
    wwr.position.east      = controller_output.pose.position.east;
    wwr.position.depth     = controller_output.pose.position.depth;
    wwr.orientation.roll   = controller_output.pose.orientation.roll;
    wwr.orientation.pitch  = controller_output.pose.orientation.pitch;
    wwr.orientation.yaw    = controller_output.pose.orientation.yaw;
    wwr.altitude_mode = false;

    // Create ROS msgs for bvr
    auv_msgs::BodyVelocityReq bvr;
    bvr.header.frame_id = _node_name + "_velocity";
    bvr.header.stamp = now;
    bvr.goal.priority = priority;
    bvr.goal.requester = _node_name + "_velocity";
    bvr.disable_axis.x     = controller_output.velocity.disable_axis[0];
    bvr.disable_axis.y     = controller_output.velocity.disable_axis[1];
    bvr.disable_axis.z     = controller_output.velocity.disable_axis[2];
    bvr.disable_axis.roll  = controller_output.velocity.disable_axis[3];
    bvr.disable_axis.pitch = controller_output.velocity.disable_axis[4];
    bvr.disable_axis.yaw   = controller_output.velocity.disable_axis[5];
    bvr.twist.linear.x     = controller_output.velocity.linear.x;
    bvr.twist.linear.y     = controller_output.velocity.linear.y;
    bvr.twist.linear.z     = controller_output.velocity.linear.z;
    bvr.twist.angular.x    = controller_output.velocity.angular.x;
    bvr.twist.angular.y    = controller_output.velocity.angular.y;
    bvr.twist.angular.z    = controller_output.velocity.angular.z;

    // Publish output
    _pub_wwr.publish(wwr);
    _pub_bvr.publish(bvr);
}


void
Pilot::publishFeedback(const control::Feedback& feedback) {
    // Conversion from internal feedback type to actionlib feedback
    cola2_msgs::SectionFeedback msg;
    msg.desired_surge           = feedback.desired_surge;
    msg.desired_depth           = feedback.desired_depth;
    msg.desired_yaw             = feedback.desired_yaw;
    msg.cross_track_error       = feedback.cross_track_error;
    msg.depth_error             = feedback.depth_error;
    msg.yaw_error               = feedback.yaw_error;
    msg.distance_to_section_end = feedback.distance_to_section_end;
    _section_server->publishFeedback(msg);
}


void
Pilot::publishMarker(double north, double east, double depth) {
    // Publish marker. Marker is published periodically so that RViz always
    // receives it, even if RViz is started after the ActionGoal arrives
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = _node_name;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = north;
    marker.pose.position.y = east;
    marker.pose.position.z = depth;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(1.0);
    marker.frame_locked = false;
    _pub_marker.publish(marker);
}


void
Pilot::getConfig() {
    // Default config here
    _config.rate                = 10;
    _config.section_server_name = "section_server";

    // Load config from param server
    getParam("pilot/rate", _config.rate);
    getParam("pilot/section_server", _config.section_server_name);
}


template<typename T> void
Pilot::getParam(std::string param_name, T& param_var) {
    // Display a message if a parameter is not found in the param server
    if (!ros::param::getCached(param_name, param_var)) {
        ROS_WARN_STREAM(_node_name << ": invalid parameter for " <<
            param_name << " in param server! Using default value of " <<
            param_var);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pilot");
    Pilot pilot;
    ros::spin();
    return 0;
}
