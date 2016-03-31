#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <actionlib/server/simple_action_server.h>
#include <cola2_msgs/WorldSectionReqAction.h>
#include <cola2_msgs/WorldWaypointReqAction.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/WorldWaypointReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <visualization_msgs/Marker.h>
#include "controllers/types.hpp"
#include "controllers/dubins.hpp"
#include "controllers/los_cte.hpp"
#include "controllers/goto.hpp"
#include "controllers/holonomic_goto.hpp"
#include "controllers/anchor.cpp"
#include <cola2_lib/cola2_rosutils/RosUtil.h>
#include <dynamic_reconfigure/server.h>
#include <cola2_control/PilotConfig.h>

#define SECTION_MODE    0
#define WAYPOINT_MODE   1

class Pilot {
public:
    Pilot();

private:
    // Node handle
    ros::NodeHandle _nh;

    // Node name
    std::string _node_name;

    // ROS variables
    ros::Subscriber _sub_nav;
    ros::Publisher _pub_wwr, _pub_bvr, _pub_marker;

    // Actionlib servers
    boost::shared_ptr< actionlib::SimpleActionServer<cola2_msgs::WorldSectionReqAction> > _section_server;
    boost::shared_ptr< actionlib::SimpleActionServer<cola2_msgs::WorldWaypointReqAction> > _waypoint_server;

    // Reconfigure parameters
    dynamic_reconfigure::Server<cola2_control::PilotConfig> _param_server;
    dynamic_reconfigure::Server<cola2_control::PilotConfig>::CallbackType _f;

    // Other vars
    control::State _current_state;

    // Controllers
    DubinsSectionController *_dubins_controller;
    LosCteController *_los_cte_controller;
    GotoController *_goto_controller;
    HolonomicGotoController *_holonomic_goto_controller;
    AnchorController *_anchor_controller;

    // Mutex between section and waypoint controllers
    // TODO: To be done

    // Config
    struct {
        LosCteControllerConfig los_cte_config;
        GotoControllerConfig goto_config;
        HolonomicGotoControllerConfig holonomic_goto_config;
        DubinsSectionControllerConfig dubins_config;
        AnchorControllerConfig anchor_config;
    } _config;

    // Methods
    void navCallback(const auv_msgs::NavSts&);
    void sectionServerCallback(const cola2_msgs::WorldSectionReqGoalConstPtr&);
    void waypointServerCallback(const cola2_msgs::WorldWaypointReqGoalConstPtr&);
    void publishControlCommands(const control::State&, unsigned int);
    void publishFeedback(const control::Feedback&, unsigned int);
    void publishMarker(double, double, double);
    void publishMarkerSections(const control::PointsList);
    void getConfig();
    void setParams(cola2_control::PilotConfig&, uint32_t);
};

Pilot::Pilot()
{
    // Node name
    _node_name = ros::this_node::getName();

    // Get config
    getConfig();

    // Initialize controllers
    // Line of Sight with Cross Tracking Error Controller
    _los_cte_controller = new LosCteController(_config.los_cte_config);
    // Go to waypoint
    _goto_controller = new GotoController(_config.goto_config);
    // Holonomic Goto waypoint
    _holonomic_goto_controller = new HolonomicGotoController(_config.holonomic_goto_config);
    // Dubins controller
    _dubins_controller = new DubinsSectionController(_config.dubins_config);
    // Anchor controller (for keep position in non holonomic vehicles)
    _anchor_controller = new AnchorController(_config.anchor_config);

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
    // SECTION action lib
    _section_server = boost::shared_ptr<actionlib::SimpleActionServer<
        cola2_msgs::WorldSectionReqAction> >(
        new actionlib::SimpleActionServer<cola2_msgs::WorldSectionReqAction>(
        _nh, "world_section_req",
        boost::bind(&Pilot::sectionServerCallback, this, _1), false));
    _section_server->start();

    // WAYPOINT action lib
    _waypoint_server = boost::shared_ptr<actionlib::SimpleActionServer<
        cola2_msgs::WorldWaypointReqAction> >(
        new actionlib::SimpleActionServer<cola2_msgs::WorldWaypointReqAction>(
        _nh, "world_waypoint_req",
        boost::bind(&Pilot::waypointServerCallback, this, _1), false));
    _waypoint_server->start();

    // Init dynamic reconfigure
    _f = boost::bind(&Pilot::setParams, this, _1, _2);
    _param_server.setCallback(_f);

    // Display message
    ROS_INFO_STREAM(_node_name << ": initialized");

    ros::spin();
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
Pilot::waypointServerCallback(const cola2_msgs::WorldWaypointReqGoalConstPtr& data) {
    // TODO: Avoid having a waypoint and a section controller running simultaneously.

    // Conversion from actionlib goal to internal Section type
    control::Waypoint waypoint;
    waypoint.altitude = data->altitude;
    waypoint.altitude_mode = data->altitude_mode;
    waypoint.controller_type = data->controller_type;
    waypoint.disable_axis.x = data->disable_axis.x;
    waypoint.disable_axis.y = data->disable_axis.y;
    waypoint.disable_axis.z = data->disable_axis.z;
    waypoint.disable_axis.roll = data->disable_axis.roll;
    waypoint.disable_axis.pitch = data->disable_axis.pitch;
    waypoint.disable_axis.yaw = data->disable_axis.yaw;
    waypoint.position.north = data->position.north;
    waypoint.position.east = data->position.east;
    waypoint.position.depth = data->position.depth;
    waypoint.orientation.roll = data->orientation.roll;
    waypoint.orientation.pitch = data->orientation.pitch;
    waypoint.orientation.yaw = data->orientation.yaw;
    waypoint.position_tolerance.x = data->position_tolerance.x;
    waypoint.position_tolerance.y = data->position_tolerance.y;
    waypoint.position_tolerance.z = data->position_tolerance.z;
    waypoint.orientation_tolerance.roll = data->orientation_tolerance.roll;
    waypoint.orientation_tolerance.pitch = data->orientation_tolerance.pitch;
    waypoint.orientation_tolerance.yaw = data->orientation_tolerance.yaw;
    waypoint.priority = data->goal.priority;
    waypoint.requester = data->goal.requester;
    waypoint.timeout = data->timeout;

    // Main loop
    double init_time = ros::Time::now().toSec();
    ros::Rate r(10);  // 10Hz
    while (ros::ok()) {
        // Declare some vars
        control::State controller_output;
        control::Feedback feedback;
        cola2_msgs::WorldWaypointReqResult result_msg;
        control::PointsList points;

        // Run controller
        try {
            switch (data->controller_type) {
                case cola2_msgs::WorldWaypointReqGoal::GOTO:
                    ROS_DEBUG_STREAM(_node_name << ": GOTO controller");
                    _goto_controller->compute(_current_state,
                                              waypoint,
                                              controller_output,
                                              feedback,
                                              points);
                    break;
                case cola2_msgs::WorldWaypointReqGoal::HOLONOMIC_GOTO:
                    ROS_DEBUG_STREAM(_node_name << ": HOLONOMIC_GOTO controller");
                    _holonomic_goto_controller->compute(_current_state,
                                                        waypoint,
                                                        controller_output,
                                                        feedback,
                                                        points);
                    break;
                case cola2_msgs::WorldWaypointReqGoal::ANCHOR:
                    ROS_DEBUG_STREAM(_node_name << ": ANCHOR controller");
                    _anchor_controller->compute(_current_state,
                                                 waypoint,
                                                 controller_output,
                                                 feedback,
                                                 points);
                    break;
                default:
                    std::cout << "Controller: " <<  data->controller_type << "\n";
                    throw std::runtime_error("Unknown controller");
            }
        }
        catch (std::exception& e) {
            // Check for failure
            ROS_ERROR_STREAM(_node_name << ": controller failure\n" << e.what());
            result_msg.final_status = cola2_msgs::WorldWaypointReqResult::FAILURE;
            _waypoint_server->setAborted(result_msg);
            break;
        }

        // Publish control commands
        publishControlCommands(controller_output, data->goal.priority);

        // Publish actionlib feedback
        publishFeedback(feedback, WAYPOINT_MODE);

        // Publish marker
        publishMarker(waypoint.position.north,
                      waypoint.position.east,
                      waypoint.position.depth);

        publishMarkerSections(points);

        // Check for success
        if (feedback.success) {
            ROS_INFO_STREAM(_node_name << ": waypoint success");
            result_msg.final_status = cola2_msgs::WorldWaypointReqResult::SUCCESS;
            _waypoint_server->setSucceeded(result_msg);
            break;
        }

        // Check for preempted. This happens upon user request (by preempting
        // or cancelling the goal, or when a new SectionGoal is received
        if (_waypoint_server->isPreemptRequested()) {
            ROS_INFO_STREAM(_node_name << ": waypoint preempted");
            _waypoint_server->setPreempted();
            break;
        }

        // Check for timeout --> If keep position, timeout = 0.0
        if (data->timeout > 0.0) {
            if ((ros::Time::now().toSec() - init_time) > data->timeout) {
                ROS_WARN_STREAM(_node_name << ": waypoint timeout");
                result_msg.final_status = cola2_msgs::WorldWaypointReqResult::TIMEOUT;
                _waypoint_server->setAborted(result_msg);
                break;
            }
        }

        // Sleep
        r.sleep();
    }
}

void
Pilot::sectionServerCallback(const cola2_msgs::WorldSectionReqGoalConstPtr& data) {
    // TODO: Avoid having a waypoint and a section controller running simultaneously.

    // Conversion from actionlib goal to internal Section type
    control::Section section;
    section.initial_position.x      = data->initial_position.x;
    section.initial_position.y      = data->initial_position.y;
    section.initial_position.z      = data->initial_position.z;
    section.initial_yaw             = data->initial_yaw;
    section.initial_surge           = data->initial_surge;
    section.use_initial_yaw         = data->use_initial_yaw;
    section.final_position.x        = data->final_position.x;
    section.final_position.y        = data->final_position.y;
    section.final_position.z        = data->final_position.z;
    section.final_yaw               = data->final_yaw;
    section.final_surge             = data->final_surge;
    section.use_final_yaw           = data->use_final_yaw;
    section.altitude_mode           = data->altitude_mode;
    section.tolerance.x             = data->tolerance.x;
    section.tolerance.y             = data->tolerance.y;
    section.tolerance.z             = data->tolerance.z;

    // Main loop
    double init_time = ros::Time::now().toSec();
    ros::Rate r(10);  // 10Hz
    while (ros::ok()) {
        // Declare some vars
        control::State controller_output;
        control::Feedback feedback;
        cola2_msgs::WorldSectionReqResult result_msg;
        control::PointsList points;

        // Run controller
        try {
            switch (data->controller_type) {
                case cola2_msgs::WorldSectionReqGoal::DUBINS:
                    ROS_DEBUG_STREAM(_node_name << ": DUBINS controller");
                    _dubins_controller->compute(_current_state,
                                                section,
                                                1.0 / 10.0,
                                                controller_output,
                                                feedback,
                                                points);
                    break;
                    case cola2_msgs::WorldSectionReqGoal::LOSCTE:
                        ROS_DEBUG_STREAM(_node_name << ": LOSCTE controller");
                        _los_cte_controller->compute(_current_state,
                                                     section,
                                                     controller_output,
                                                     feedback,
                                                     points);
                        break;
                default:
                    throw std::runtime_error("Unknown controller");
            }
        }
        catch (std::exception& e) {
            // Check for failure
            ROS_ERROR_STREAM(_node_name << ": controller failure\n" << e.what());
            result_msg.final_status = cola2_msgs::WorldSectionReqResult::FAILURE;
            _section_server->setAborted(result_msg);
            break;
        }

        // Publish control commands
        publishControlCommands(controller_output, data->priority);

        // Publish actionlib feedback
        publishFeedback(feedback, SECTION_MODE);

        // Publish marker
        publishMarker(section.final_position.x,
                      section.final_position.y,
                      section.final_position.z);

        publishMarkerSections(points);

        // Check for success
        if (feedback.success) {
            ROS_INFO_STREAM(_node_name << ": section success");
            result_msg.final_status = cola2_msgs::WorldSectionReqResult::SUCCESS;
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
                result_msg.final_status = cola2_msgs::WorldSectionReqResult::TIMEOUT;
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
                              const unsigned int priority) {
    // Get time
    ros::Time now = ros::Time::now();

    // Create ROS msgs for wwr
    auv_msgs::WorldWaypointReq wwr;
    wwr.header.frame_id = _node_name + "_pose";
    wwr.header.stamp = now;
    wwr.goal.priority = priority;
    wwr.goal.requester = _node_name + "_pose";
    wwr.disable_axis.x     = controller_output.pose.disable_axis.x;
    wwr.disable_axis.y     = controller_output.pose.disable_axis.y;
    wwr.disable_axis.z     = controller_output.pose.disable_axis.z;
    wwr.disable_axis.roll  = controller_output.pose.disable_axis.roll;
    wwr.disable_axis.pitch = controller_output.pose.disable_axis.pitch;
    wwr.disable_axis.yaw   = controller_output.pose.disable_axis.yaw;
    wwr.position.north     = controller_output.pose.position.north;
    wwr.position.east      = controller_output.pose.position.east;
    wwr.position.depth     = controller_output.pose.position.depth;
    wwr.orientation.roll   = controller_output.pose.orientation.roll;
    wwr.orientation.pitch  = controller_output.pose.orientation.pitch;
    wwr.orientation.yaw    = controller_output.pose.orientation.yaw;
    wwr.altitude_mode = controller_output.pose.altitude_mode;
    wwr.altitude = controller_output.pose.altitude;

    // Create ROS msgs for bvr
    auv_msgs::BodyVelocityReq bvr;
    bvr.header.frame_id = _node_name + "_velocity";
    bvr.header.stamp = now;
    bvr.goal.priority = priority;
    bvr.goal.requester = _node_name + "_velocity";
    bvr.disable_axis.x     = controller_output.velocity.disable_axis.x;
    bvr.disable_axis.y     = controller_output.velocity.disable_axis.y;
    bvr.disable_axis.z     = controller_output.velocity.disable_axis.z;
    bvr.disable_axis.roll  = controller_output.velocity.disable_axis.roll;
    bvr.disable_axis.pitch = controller_output.velocity.disable_axis.pitch;
    bvr.disable_axis.yaw   = controller_output.velocity.disable_axis.yaw;
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
Pilot::publishFeedback(const control::Feedback& feedback, unsigned int mode=SECTION_MODE) {
    // Conversion from internal feedback type to actionlib feedback
    switch(mode) {
        case SECTION_MODE:
        {
            cola2_msgs::WorldSectionReqFeedback msg;
            msg.desired_surge           = feedback.desired_surge;
            msg.desired_depth           = feedback.desired_depth;
            msg.desired_yaw             = feedback.desired_yaw;
            msg.cross_track_error       = feedback.cross_track_error;
            msg.depth_error             = feedback.depth_error;
            msg.yaw_error               = feedback.yaw_error;
            msg.distance_to_section_end = feedback.distance_to_end;
            _section_server->publishFeedback(msg);
            break;
        }
        case WAYPOINT_MODE:
        {
            cola2_msgs::WorldWaypointReqFeedback msg;
            // TODO: To be completed
            msg.distance_to_waypoint = feedback.distance_to_end;
            _waypoint_server->publishFeedback(msg);
            break;
        }
        default:
            std::cout << "Error, invalid feedback message!\n";
    }
}


void
Pilot::publishMarker(double north, double east, double depth) {
    // Publish marker. Marker is published periodically so that RViz always
    // receives it, even if RViz is started after the ActionGoal arrives
    visualization_msgs::Marker marker;
    marker.header.frame_id      = "world";
    marker.header.stamp         = ros::Time::now();
    marker.ns                   = _node_name;
    marker.type                 = visualization_msgs::Marker::SPHERE;
    marker.action               = visualization_msgs::Marker::ADD;
    marker.pose.position.x      = north;
    marker.pose.position.y      = east;
    marker.pose.position.z      = depth;
    marker.pose.orientation.w   = 1.0;
    marker.pose.orientation.x   = 0.0;
    marker.pose.orientation.y   = 0.0;
    marker.pose.orientation.z   = 0.0;
    marker.scale.x              = 1.0;
    marker.scale.y              = 1.0;
    marker.scale.z              = 1.0;
    marker.color.r              = 1.0;
    marker.color.g              = 0.0;
    marker.color.b              = 0.0;
    marker.color.a              = 0.5;
    marker.lifetime             = ros::Duration(1.0);
    marker.frame_locked         = false;
    _pub_marker.publish(marker);
}

void
Pilot::publishMarkerSections(const control::PointsList points)
{
    // Create visualization marker
    visualization_msgs::Marker marker;
    marker.header.frame_id  = "world";
    marker.header.stamp     = ros::Time::now();
    marker.ns               = _node_name;
    marker.type             = visualization_msgs::Marker::LINE_LIST;
    marker.action           = visualization_msgs::Marker::ADD;

    // Add points to it
    for (unsigned int i = 0; i < points.points_list.size(); i++) {
        geometry_msgs::Point p;
        p.x = points.points_list.at(i).x;
        p.y = points.points_list.at(i).y;
        p.z = points.points_list.at(i).z;
        marker.points.push_back(p);
    }

    marker.scale.x      = 0.35;
    marker.color.r      = 0.8;
    marker.color.g      = 0.8;
    marker.color.b      = 0.0;
    marker.color.a      = 0.5;
    marker.lifetime     = ros::Duration(1.0);
    marker.frame_locked = false;
    _pub_marker.publish(marker);
}

void
Pilot::getConfig() {
    // Load config from param server
    // LOS-CTE controller

    cola2::rosutil::getParam("pilot/los_cte_delta", _config.los_cte_config.delta, 5.0);
    cola2::rosutil::getParam("pilot/los_cte_distance_to_max_velocity", _config.los_cte_config.distance_to_max_velocity, 5.0);
    cola2::rosutil::getParam("pilot/los_cte_max_surge_velocity", _config.los_cte_config.max_surge_velocity, 0.5);
    cola2::rosutil::getParam("pilot/los_cte_min_surge_velocity", _config.los_cte_config.min_surge_velocity, 0.2);
    cola2::rosutil::getParam("pilot/los_cte_min_velocity_ratio", _config.los_cte_config.min_velocity_ratio, 0.1);

    // GOTO controller
    cola2::rosutil::getParam("pilot/goto_max_angle_error", _config.goto_config.max_angle_error, 0.3);
    cola2::rosutil::getParam("pilot/goto_max_surge", _config.goto_config.max_surge, 0.5);
    cola2::rosutil::getParam("pilot/goto_surge_proportional_gain", _config.goto_config.surge_proportional_gain, 0.25);

    // DUBINS controller
    cola2::rosutil::getParam("pilot/dubins_yaw_ki", _config.dubins_config.yaw_ki, 0.003);
    cola2::rosutil::getParam("pilot/dubins_yaw_kp", _config.dubins_config.yaw_kp, 0.09);
    cola2::rosutil::getParam("pilot/dubins_lookahead_sec", _config.dubins_config.lookahead_sec, 4.0);
    cola2::rosutil::getParam("pilot/dubins_acceptance_sec", _config.dubins_config.acceptance_sec, 3.0);

    // ANCHOR controller
    cola2::rosutil::getParam("pilot/anchor_kp", _config.anchor_config.kp, 0.1);
    cola2::rosutil::getParam("pilot/anchor_radius", _config.anchor_config.radius, 1.0);
    cola2::rosutil::getParam("pilot/anchor_min_surge", _config.anchor_config.min_surge, -0.1);
    cola2::rosutil::getParam("pilot/anchor_max_surge", _config.anchor_config.max_surge, 0.3);
    cola2::rosutil::getParam("pilot/anchor_safety_distance", _config.anchor_config.safety_distance, 50.0);
    cola2::rosutil::getParam("pilot/anchor_max_angle_error", _config.anchor_config.max_angle_error, 0.5);
}

void
Pilot::setParams(cola2_control::PilotConfig &config, uint32_t level)
{
    ROS_INFO_STREAM(_node_name << ": new parameters received!\n");
    _config.los_cte_config.delta = config.los_cte_delta;
    _config.los_cte_config.distance_to_max_velocity = config.los_cte_distance_to_max_velocity;
    _config.los_cte_config.max_surge_velocity = config.los_cte_max_surge_velocity;
    _config.los_cte_config.min_surge_velocity = config.los_cte_min_surge_velocity;
    _config.los_cte_config.min_velocity_ratio = config.los_cte_min_velocity_ratio;
    _los_cte_controller->setConfig(_config.los_cte_config);

    _config.goto_config.max_angle_error = config.goto_max_angle_error;
    _config.goto_config.max_surge = config.goto_max_surge;
    _config.goto_config.surge_proportional_gain = config.goto_surge_proportional_gain;
    _goto_controller->setConfig(_config.goto_config);

    _config.dubins_config.yaw_kp = config.dubins_yaw_kp;
    _config.dubins_config.yaw_ki = config.dubins_yaw_ki;
    _config.dubins_config.acceptance_sec = config.dubins_acceptance_sec;
    _config.dubins_config.lookahead_sec = config.dubins_lookahead_sec;
    _dubins_controller->setConfig(_config.dubins_config);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pilot_new");
    Pilot pilot;
    return 0;
}
