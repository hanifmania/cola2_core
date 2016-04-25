#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_msgs/WorldSectionReqAction.h>
#include <cola2_msgs/WorldWaypointReqAction.h>
#include <boost/shared_ptr.hpp>
#include <cola2_msgs/String.h>
#include <cola2_msgs/NewGoto.h>
#include <cola2_msgs/Submerge.h>
#include <cola2_msgs/SetTrajectory.h>
#include <std_srvs/Empty.h>
#include <auv_msgs/GoalDescriptor.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>
#include <ros/console.h>
#include <cola2_lib/cola2_navigation/Ned.h>
#include <auv_msgs/NavSts.h>
#include "controllers/types.hpp"
#include <cola2_lib/cola2_rosutils/RosUtil.h>
#include "controllers/types.hpp"
#include <vector>

typedef struct {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<bool> altitude_mode;
    std::vector<double> yaw;
    std::vector<double> surge;
    std::vector<double> tolerance;
    std::string mode;
    unsigned int timeout;
    std::vector<double> wait;
    bool valid_trajectory;
    bool force_initial_final_waypoints_at_surface;
} Trajectory;

typedef struct {
    double max_distance_to_waypoint;
    control::vector6d tolerance;
} CaptainConfig;

class Captain {
public:
    Captain();

private:
    // Node handle
    ros::NodeHandle _n;

    // Node name
    std::string _name;

    // Class attributes
    bool _is_waypoint_running;
    bool _is_section_running;
    bool _is_trajectory_disabled;
    ros::MultiThreadedSpinner _spinner;
    Trajectory _trajectory;
    Ned *_ned;
    control::Nav _nav;
    CaptainConfig _config;
    bool _is_holonomic_keep_pose_enabled;
    double _min_goto_vel;
    double _min_loscte_vel;

    // Publishers
    ros::Publisher _pub_path;

    // Services
    ros::ServiceServer _enable_goto_srv;
    ros::ServiceServer _disable_goto_srv;
    ros::ServiceServer _submerge_srv;
    ros::ServiceServer _load_trajectory_srv;
    ros::ServiceServer _set_trajectory_srv;
    ros::ServiceServer _enable_trajectory_srv;
    ros::ServiceServer _disable_trajectory_srv;
    ros::ServiceServer _enable_keep_position_holonomic_srv;
    ros::ServiceServer _enable_keep_position_non_holonomic_srv;
    ros::ServiceServer _disable_keep_position_srv;
    ros::Subscriber _2D_nav_goal;

    // Subscriber
    ros::Subscriber _sub_nav;

    // Actionlib client
    boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldSectionReqAction> > _section_client;

    boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldWaypointReqAction> > _waypoint_client;

    // Thread method to wait for Goto
    boost::thread _thread_waypoint;
    void wait_waypoint();

    void test();

    // Config
    struct {
        std::string section_server_name;
    } config;

    // Methods
    bool check_no_request_running();

    void get_config();

    nav_msgs::Path create_path_from_trajectory(Trajectory trajectory);

    double distance_to(const double, const double, const double, const double, const bool);

    // ... services callbacks
    bool enable_goto(cola2_msgs::NewGoto::Request&,
                     cola2_msgs::NewGoto::Response&);

    bool submerge(cola2_msgs::Submerge::Request&,
                  cola2_msgs::Submerge::Response&);

    bool disable_goto(std_srvs::Empty::Request&,
                      std_srvs::Empty::Response&);

    bool load_trajectory(std_srvs::Empty::Request&,
                         std_srvs::Empty::Response&);

    bool set_trajectory(cola2_msgs::SetTrajectory::Request&,
                        cola2_msgs::SetTrajectory::Response&);

    bool enable_trajectory(std_srvs::Empty::Request&,
                           std_srvs::Empty::Response&);

    bool disable_trajectory(std_srvs::Empty::Request&,
                            std_srvs::Empty::Response&);

    bool enable_keep_position_holonomic(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&);

    bool enable_keep_position_non_holonomic(std_srvs::Empty::Request&,
                                            std_srvs::Empty::Response&);

    bool disable_keep_position(std_srvs::Empty::Request&,
                               std_srvs::Empty::Response&);

    void update_nav(const ros::MessageEvent<auv_msgs::NavSts const> & msg);
    void nav_goal(const ros::MessageEvent<geometry_msgs::PoseStamped const> & msg);
};


Captain::Captain():
    _is_waypoint_running(false),
    _is_section_running(false),
    _is_trajectory_disabled(false),
    _spinner(2),
    _is_holonomic_keep_pose_enabled(false)
{
    // Node name
    _name = ros::this_node::getName();

    // Get config
    get_config();
    _trajectory.valid_trajectory = false;

    // Init publishers
    _pub_path = _n.advertise<nav_msgs::Path>("/cola2_control/trajectory_path", 1, true);

    // Actionlib client. Smart pointer is used so that client construction is
    // delayed after configuration is loaded
    ROS_WARN_STREAM(_name << ": Wait for pilot action libs");
    _section_client = boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldSectionReqAction> >(
        new actionlib::SimpleActionClient<cola2_msgs::WorldSectionReqAction>(
        "world_section_req", true));
    _section_client->waitForServer();  // Wait for infinite time

    _waypoint_client = boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldWaypointReqAction> >(
        new actionlib::SimpleActionClient<cola2_msgs::WorldWaypointReqAction>(
        "world_waypoint_req", true));
    _waypoint_client->waitForServer();  // Wait for infinite time

    // Init services
    _enable_goto_srv = _n.advertiseService("/cola2_control/enable_goto", &Captain::enable_goto, this);
    _disable_goto_srv = _n.advertiseService("/cola2_control/disable_goto", &Captain::disable_goto, this);
    _submerge_srv = _n.advertiseService("/cola2_control/submerge", &Captain::submerge, this);

    _load_trajectory_srv = _n.advertiseService("/cola2_control/load_trajectory", &Captain::load_trajectory, this);
    _set_trajectory_srv = _n.advertiseService("/cola2_control/load_trajectory", &Captain::set_trajectory, this);
    _enable_trajectory_srv = _n.advertiseService("/cola2_control/enable_trajectory", &Captain::enable_trajectory, this);
    _disable_trajectory_srv = _n.advertiseService("/cola2_control/disable_trajectory", &Captain::disable_trajectory, this);
    _enable_keep_position_holonomic_srv = _n.advertiseService("/cola2_control/enable_keep_position_g500", &Captain::enable_keep_position_holonomic, this);
    _enable_keep_position_non_holonomic_srv = _n.advertiseService("/cola2_control/enable_keep_position_s2", &Captain::enable_keep_position_non_holonomic, this);
    _disable_keep_position_srv = _n.advertiseService("/cola2_control/disable_keep_position", &Captain::disable_keep_position, this);

    // Subscriber

    _sub_nav = _n.subscribe("/cola2_navigation/nav_sts", 1, &Captain::update_nav, this);
    _2D_nav_goal = _n.subscribe("/move_base_simple/goal", 1, &Captain::nav_goal, this);

    // test();
    _spinner.spin();
}

void
Captain::update_nav(const ros::MessageEvent<auv_msgs::NavSts const> & msg)
{
    _nav.x = msg.getMessage()->position.north;
    _nav.y = msg.getMessage()->position.east;
    _nav.z = msg.getMessage()->position.depth;
    _nav.yaw = msg.getMessage()->orientation.yaw;
    _nav.altitude = msg.getMessage()->altitude;
}


/**
/* nav_goal implements the navigation goal option available in RVIZ to send
/* 2D navigation goals from it.
**/
void
Captain::nav_goal(const ros::MessageEvent<geometry_msgs::PoseStamped const> & msg)
{
    // Disable a previous goto if necessary
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    disable_goto(req, res);

    double x, y;
    // TODO: It is necessary to check TF between msg.header.frame_id and world
    //       and apply a transformation.
    if (msg.getMessage()->header.frame_id == "rviz" || msg.getMessage()->header.frame_id == "world"){
        x = msg.getMessage()->pose.position.x;
        y = msg.getMessage()->pose.position.y;
        if (msg.getMessage()->header.frame_id == "rviz") y = -1.0 * msg.getMessage()->pose.position.y;

        ROS_INFO_STREAM(_name << "Received 2D Nav Goal to " << x << ", " << y);
        cola2_msgs::NewGoto::Request goto_req;
        cola2_msgs::NewGoto::Response goto_res;

        goto_req.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
        goto_req.altitude_mode = false;
        goto_req.blocking = false;
        goto_req.disable_axis.x = false;
        goto_req.disable_axis.y = true;
        goto_req.disable_axis.z = false;
        goto_req.disable_axis.roll = true;
        goto_req.disable_axis.pitch = true;
        goto_req.disable_axis.yaw = false;
        goto_req.position.x = x;
        goto_req.position.y = y;
        goto_req.position.z = 0.0;
        goto_req.position_tolerance.x = 2.0;
        goto_req.position_tolerance.y = 2.0;
        goto_req.position_tolerance.z = 1.0;
        goto_req.orientation_tolerance.yaw = 0.1;
        goto_req.reference = cola2_msgs::NewGoto::Request::REFERENCE_NED;

        enable_goto(goto_req, goto_res);
    }
    else {
        ROS_INFO_STREAM(_name << "Received INVALID 2D Nav Goal from frame " << msg.getMessage()->header.frame_id << ". Change to frame 'rviz or world'");
    }
}

double
Captain::distance_to(const double x, const double y, const double depth, const double altitude, const bool altitude_mode)
{
    double inc_z;
    if (altitude_mode) {
        inc_z = _nav.altitude - altitude;
    }
    else {
        inc_z = depth - _nav.z;
    }
    return sqrt(pow(x - _nav.x, 2) + pow(y - _nav.y, 2) + pow(inc_z, 2));
}

void
Captain::get_config() {
    // Default config here
    config.section_server_name = "section_server";
    double ned_latitude;
    double ned_longitude;
    // Load NED origin
    if (!ros::param::getCached("navigator/ned_latitude", ned_latitude) ||
        !ros::param::getCached("navigator/ned_longitude", ned_longitude)){
      ROS_ASSERT_MSG(false, "NED origin not found in param server");
    }
    _ned = new Ned(ned_latitude, ned_longitude, 0.0);
    cola2::rosutil::getParam("/captain/max_distance_to_waypoint", _config.max_distance_to_waypoint, 300.0);
    std::vector<double> tolerance;
    cola2::rosutil::loadVector("/captain/tolerance", tolerance);
    ROS_ASSERT_MSG(tolerance.size() == 6, "Missing/Invalid /captain/tolerance definition in param server");
    _config.tolerance.x = tolerance.at(0);
    _config.tolerance.y = tolerance.at(1);
    _config.tolerance.z = tolerance.at(2);
    _config.tolerance.roll = tolerance.at(3);
    _config.tolerance.pitch = tolerance.at(4);
    _config.tolerance.yaw = tolerance.at(5);

    // Get max velocity Z from controller params and goto_max_surge or
    // los_cte_max_surge_velocity to estimate GOTO timeout.
    double surge, heave, surge_los;
    cola2::rosutil::getParam("/controller/max_velocity_z", heave, 0.1);
    cola2::rosutil::getParam("pilot/goto_max_surge", surge, 0.1);
    cola2::rosutil::getParam("pilot/los_cte_max_surge_velocity", surge_los, 0.1);
    _min_goto_vel = std::min(heave, surge);
    _min_loscte_vel = std::min(heave, surge_los);
}


bool
Captain::enable_goto(cola2_msgs::NewGoto::Request &req,
                     cola2_msgs::NewGoto::Response &res)
{
    if(check_no_request_running()){
        cola2_msgs::WorldWaypointReqGoal waypoint;
        waypoint.goal.priority = req.priority;
        waypoint.goal.requester = _name;
        waypoint.altitude_mode = req.altitude_mode;
        waypoint.altitude = req.altitude;

        // Check req.reference to transform req.position to appropiate reference frame.
        if(req.reference == cola2_msgs::NewGotoRequest::REFERENCE_NED){
            waypoint.position.north = req.position.x;
            waypoint.position.east = req.position.y;
        }
        else if (req.reference == cola2_msgs::NewGotoRequest::REFERENCE_GLOBAL){
            double north, east, depth;
            _ned->geodetic2Ned(req.position.x, req.position.y, 0.0,
                               north, east, depth);
            waypoint.position.north = north;
            waypoint.position.east = east;
        }
        else {
            ROS_WARN_STREAM(_name << ": Invalid GOTO reference. REFERENCE_VEHICLE not yet implemented.");
        }

        // Check max distance to waypoint
        double distance_to_waypoint = distance_to(waypoint.position.north,
                                                  waypoint.position.east,
                                                  req.position.z,
                                                  req.altitude,
                                                  req.altitude_mode);

        if(distance_to_waypoint > _config.max_distance_to_waypoint) {
            ROS_WARN_STREAM(_name << ": Max distance to waypoint is " <<
                            _config.max_distance_to_waypoint << " requested waypoint is at "
                            << distance_to_waypoint);
            return false;
        }

        waypoint.position.depth = req.position.z;
        waypoint.orientation.yaw = req.yaw;
        waypoint.disable_axis.x = req.disable_axis.x;
        waypoint.disable_axis.y = req.disable_axis.y;
        waypoint.disable_axis.z = req.disable_axis.z;
        waypoint.disable_axis.roll = req.disable_axis.roll;
        waypoint.disable_axis.pitch = req.disable_axis.pitch;
        waypoint.disable_axis.yaw = req.disable_axis.yaw;
        waypoint.position_tolerance.x = req.position_tolerance.x;
        waypoint.position_tolerance.y = req.position_tolerance.y;
        waypoint.position_tolerance.z = req.position_tolerance.z;
        waypoint.orientation_tolerance.yaw = req.orientation_tolerance.yaw;
        waypoint.linear_velocity.x = req.linear_velocity.x;

        if (req.linear_velocity.y != 0.0 || req.linear_velocity.z != 0.0 || req.angular_velocity.yaw != 0.0) {
            ROS_WARN_STREAM(_name << ": GOTO velocity can only be defined in surge. Heave, sway and yaw depend on pose controller.");
        }

        // Choose WorldWaypointReq mode taking into account disable axis & tolerance
        if (req.position_tolerance.x == 0.0 && req.position_tolerance.y == 0.0 && req.position_tolerance.z == 0.0 && !req.disable_axis.x && req.disable_axis.y && !req.disable_axis.yaw) {
            // Non holonomic keep position
            waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::ANCHOR;
        }
        else if (!req.disable_axis.x && req.disable_axis.y && !req.disable_axis.yaw) {
            // X, Z, Yaw Goto (with or wiyhou Z)
            waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::GOTO;
        }
        else if (!req.disable_axis.x && !req.disable_axis.y){
            // Holonomic X, Y, Z, Yaw goto (with or without Z and Yaw)
            waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::HOLONOMIC_GOTO;
        }
        else if (req.disable_axis.x && req.disable_axis.y && !req.disable_axis.z) {
            // Submerge Z, Yaw (with or without Yaw)
            waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::GOTO;
        }

        _is_waypoint_running = true;

        // Compute timeout
        double min_vel = _min_goto_vel;
        if (waypoint.linear_velocity.x != 0.0 and min_vel > waypoint.linear_velocity.x) {
            min_vel = waypoint.linear_velocity.x;
        }
        waypoint.timeout = (2.0 * distance_to_waypoint) / min_vel;

        ROS_INFO_STREAM(_name << ": Send WorldWaypointRequest at " << waypoint.position.north << ", "  << waypoint.position.east << ", " << waypoint.position.depth << ". Timeout = " << waypoint.timeout << "\n");
        _waypoint_client->sendGoal(waypoint);
        res.success = true;
        // If blocking wait for the result
        if(req.blocking){
            wait_waypoint();
        }
        else {
            // If goto is not blocking start a thread that waits for result
            // and then sets _is_waypoint_running to false
            _thread_waypoint = boost::thread(&Captain::wait_waypoint, this);
        }
    }
    else {
        res.success = false;
    }
    return true;
}

bool
Captain::submerge(cola2_msgs::Submerge::Request &req,
                  cola2_msgs::Submerge::Response &res)
{
    cola2_msgs::NewGoto::Request goto_req;
    cola2_msgs::NewGoto::Response goto_res;

    goto_req.priority = auv_msgs::GoalDescriptor::PRIORITY_MANUAL_OVERRIDE + 10;
    goto_req.altitude = req.z;
    goto_req.altitude_mode = req.altitude_mode;
    goto_req.blocking = false;
    goto_req.disable_axis.x = true;
    goto_req.disable_axis.y = true;
    goto_req.disable_axis.z = false;
    goto_req.disable_axis.roll = true;
    goto_req.disable_axis.pitch = true;
    goto_req.disable_axis.yaw = false;
    goto_req.position.z = req.z;
    goto_req.position_tolerance.z = 1.0;
    goto_req.yaw = _nav.yaw;
    goto_req.orientation_tolerance.yaw = 0.1;
    goto_req.reference = cola2_msgs::NewGoto::Request::REFERENCE_NED;

    enable_goto(goto_req, goto_res);
    res.attempted = goto_res.success;
    return true;
}


bool
Captain::disable_goto(std_srvs::Empty::Request&,
                      std_srvs::Empty::Response&)
{
    if(_is_waypoint_running) {
        _waypoint_client->cancelGoal();
        _is_waypoint_running = false;
    }
    return true;
}

bool
Captain::set_trajectory(cola2_msgs::SetTrajectory::Request &req,
                        cola2_msgs::SetTrajectory::Response &res)
{
    // Set a mission file as a service
    bool valid_trajectory = true;

    // Copy data into structure
    Trajectory trajectory;
    trajectory.x = req.x;
    trajectory.y = req.y;
    trajectory.z = req.z;
    trajectory.yaw = req.yaw;
    trajectory.surge = req.surge;
    trajectory.wait = req.wait;
    trajectory.timeout = req.timeout;
    trajectory.mode = req.mode;
    // Special case for std::vector<bool> (from http://wiki.ros.org/msg)
    // bool in C++ is aliased to uint8_t because of array types:
    // std::vector<bool> is in fact a specialized form of vector that is not a
    // container. See http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2160.html
    // for more information.
    // msg: bool[]  -->  c++: std::vector<uint8_t>
    trajectory.altitude_mode.resize(req.altitude_mode.size());
    for (int i = 0; i < req.altitude_mode.size(); i++)
        trajectory.altitude_mode[i] = req.altitude_mode[i];

    // Check sizes
    if (trajectory.x.size() > 1)
    {
        ROS_ERROR("Minimum mission size is 2");
        valid_trajectory = false;
    }
    if ((trajectory.x.size() != trajectory.y.size()) ||
        (trajectory.x.size() != trajectory.z.size()) ||
        (trajectory.x.size() != trajectory.altitude_mode.size()) ||
        (trajectory.x.size() != trajectory.yaw.size()) ||
        (trajectory.x.size() != trajectory.surge.size()) ||
        (trajectory.x.size() != trajectory.wait.size()))
        {
            ROS_ERROR("Different mission array sizes");
            valid_trajectory = false;
        }
    // Check control mode
    if (trajectory.mode != "los_cte" && trajectory.mode != "dubins")
    {
        ROS_ERROR("Invalid control mode");
        valid_trajectory = false;
    }

    // If the trajectory is defined globally, tranform from lat/lon to NED.
    if(!req.is_local)
    {
        ROS_INFO_STREAM(_name << ": global trajectory");
        double north, east, depth;
        for (unsigned int i = 0; i < trajectory.x.size(); i++)
        {
            _ned->geodetic2Ned(trajectory.x.at(i), trajectory.y.at(i), 0.0,
                               north, east, depth);
            trajectory.x.at(i) = north;
            trajectory.y.at(i) = east;
        }
    }
    trajectory.valid_trajectory = valid_trajectory;

    // generate path
    if(valid_trajectory) {
        nav_msgs::Path path = create_path_from_trajectory(trajectory);
        _pub_path.publish(path);
        _trajectory = trajectory;
    }

    res.valid = valid_trajectory;
    return valid_trajectory;
}

bool
Captain::load_trajectory(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res)
{
    // Load mission file from PARAM SERVER.It means that the YAML file must
    // be firts loaded to the param server!
    bool valid_trajectory = true ;
    bool is_trajectory_global = false;
    bool is_trajectory_local = false;
    double ned_latitude;
    double ned_longitude;
    Trajectory trajectory;

    if(cola2::rosutil::loadVector("trajectory/north", trajectory.x) &&
       cola2::rosutil::loadVector("trajectory/east", trajectory.y)){
        ROS_INFO_STREAM(_name << ": loading local trajectory ...");
        is_trajectory_local = true;
    }
    if (cola2::rosutil::loadVector("trajectory/latitude", trajectory.x) &&
        cola2::rosutil::loadVector("trajectory/longitude", trajectory.y)){
        ROS_INFO_STREAM(_name << ": loading global trajectory ...");
        is_trajectory_global = true;
    }
    if(is_trajectory_global && is_trajectory_local) {
        ROS_WARN_STREAM(_name << ": invalid trajectory. Found North/East and Latitude/Longitude waypoints in param server!");
        _trajectory.valid_trajectory = false;
        return false;
    }
    if(!is_trajectory_global && !is_trajectory_local) {
        ROS_INFO_STREAM(_name << ": invalid trajectory");
        _trajectory.valid_trajectory = false;
        return false;
    }
    ROS_ASSERT_MSG(trajectory.x.size() > 1, "Minimum mission size is 2");
    ROS_ASSERT_MSG(trajectory.x.size() == trajectory.y.size(), "Different mission array sizes");

    if(!cola2::rosutil::loadVector("trajectory/z", trajectory.z)) valid_trajectory = false;
    ROS_ASSERT_MSG(trajectory.x.size() == trajectory.z.size(), "Different mission array sizes");

    if(!cola2::rosutil::loadVector("trajectory/altitude_mode", trajectory.altitude_mode)) valid_trajectory = false;
    ROS_ASSERT_MSG(trajectory.x.size() == trajectory.altitude_mode.size(), "Different mission array sizes");

    if (!ros::param::getCached("trajectory/mode", trajectory.mode)) valid_trajectory = false;
    ROS_ASSERT_MSG(trajectory.mode == "los_cte" || trajectory.mode == "dubins", "Invalid trajectory mode");

    if (!ros::param::getCached("trajectory/force_initial_final_waypoints_at_surface", trajectory.force_initial_final_waypoints_at_surface)) {
        trajectory.force_initial_final_waypoints_at_surface = true;
    }

    if(ros::param::has("trajectory/tolerance")) {
        cola2::rosutil::loadVector("trajectory/tolerance", trajectory.tolerance);
        ROS_ASSERT_MSG(trajectory.tolerance.size() == 6, "Invalid tolerance array");
    }

    if(ros::param::has("trajectory/wait")) {
        cola2::rosutil::loadVector("trajectory/wait", trajectory.wait);
        ROS_ASSERT_MSG(trajectory.x.size() == trajectory.wait.size(), "Different mission array sizes");
    }
    else {
        for (unsigned int i = 0; i < trajectory.x.size(); i++) {
            trajectory.wait.push_back(0.0);
        }
    }
    if(ros::param::has("trajectory/yaw")) {
        cola2::rosutil::loadVector("trajectory/yaw", trajectory.yaw);
        ROS_ASSERT_MSG(trajectory.x.size() == trajectory.yaw.size(), "Different mission array sizes");
    }

    if(ros::param::has("trajectory/surge")) {
        cola2::rosutil::loadVector("trajectory/surge", trajectory.surge);
        ROS_ASSERT_MSG(trajectory.x.size() == trajectory.surge.size(), "Different mission array sizes");
    }
    else {
        for (unsigned int i = 0; i < trajectory.x.size(); i++) {
            trajectory.surge.push_back(0.0);
        }
    }

    ROS_ASSERT_MSG(valid_trajectory, "Invalid mission parameter");

    // If the trajectory is defined globally, tranform from lat/lon to NED.
    if(is_trajectory_global){
        double north, east, depth;
        for(unsigned int i = 0; i < trajectory.x.size(); i++){
            _ned->geodetic2Ned(trajectory.x.at(i), trajectory.y.at(i), 0.0,
                               north, east, depth);
            trajectory.x.at(i) = north;
            trajectory.y.at(i) = east;
        }
    }

    trajectory.valid_trajectory = valid_trajectory;
    if(valid_trajectory) {
        nav_msgs::Path path = create_path_from_trajectory(trajectory);
        _pub_path.publish(path);
        _trajectory = trajectory;
    }

    return valid_trajectory;
}


bool
Captain::enable_trajectory(std_srvs::Empty::Request&,
                           std_srvs::Empty::Response&)
{
    if(check_no_request_running() && _trajectory.valid_trajectory){
        _is_trajectory_disabled = false;
        cola2_msgs::NewGoto::Request req;
        cola2_msgs::NewGoto::Response res;
        if (_trajectory.force_initial_final_waypoints_at_surface) {
            // Move to initial waypoint on surface and then submerge to it
            req.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
            req.altitude_mode = false;
            req.blocking = true;
            req.disable_axis.x = false;
            req.disable_axis.y = true;
            req.disable_axis.z = false;
            req.disable_axis.yaw = false;
            req.position.x = _trajectory.x.at(0);
            req.position.y = _trajectory.y.at(0);
            req.position.z = 0.0;
            if (_trajectory.tolerance.size() == 0) {
                req.position_tolerance.x = _config.tolerance.x;
                req.position_tolerance.y = _config.tolerance.y;
                req.position_tolerance.z = _config.tolerance.z;
            }
            else {
                req.position_tolerance.x = _trajectory.tolerance.at(0);
                req.position_tolerance.y = _trajectory.tolerance.at(1);
                req.position_tolerance.z = _trajectory.tolerance.at(2);
            }

            req.linear_velocity.x = _trajectory.surge.at(0);
            req.reference = cola2_msgs::NewGoto::Request::REFERENCE_NED;
            enable_goto(req, res);
            ROS_ASSERT_MSG(res.success, "Impossible to reach initial waypoint");

            if (!_is_trajectory_disabled) {
                // Submerge until initial waypoint
                req.blocking = true;
                req.disable_axis.x = true;
                req.yaw = _nav.yaw;
                req.position.z = _trajectory.z.at(0);
                req.altitude = _trajectory.z.at(0);
                req.altitude_mode = _trajectory.altitude_mode.at(0);
                enable_goto(req, res);
                ROS_ASSERT_MSG(res.success, "Impossible to reach initial waypoint");
            }
        }

        if (!_is_trajectory_disabled) {
            _is_section_running = true;
            // Create section
            cola2_msgs::WorldSectionReqGoal section;
            section.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
            if(_trajectory.mode == "los_cte") {
                section.controller_type = cola2_msgs::WorldSectionReqGoal::LOSCTE;
            }
            else if(_trajectory.mode == "dubins"){
            	section.controller_type = cola2_msgs::WorldSectionReqGoal::DUBINS;
            }
            else {
                ROS_ASSERT_MSG(false, "Invalid trajectory mode!");
            }
            section.disable_z = false;

            if (_trajectory.tolerance.size() == 0) {
                section.tolerance.x = _config.tolerance.x;
                section.tolerance.y = _config.tolerance.y;
                section.tolerance.z = _config.tolerance.z;
            }
            else {
                section.tolerance.x = _trajectory.tolerance.at(0);
                section.tolerance.y = _trajectory.tolerance.at(1);
                section.tolerance.z = _trajectory.tolerance.at(2);
            }

            unsigned int i = 1;
            while (i < _trajectory.x.size() &&  _is_section_running) {
                // For each pair of waypoints:
                // ...initial point of the section

                // Set initial position
                section.initial_position.x = _trajectory.x.at(i-1);
                section.initial_position.y = _trajectory.y.at(i-1);
                section.initial_position.z = _trajectory.z.at(i-1);

                // Set initial yaw
                if (_trajectory.yaw.size() > 0) {
                    section.initial_yaw = _trajectory.yaw.at(i-1);
                    section.use_initial_yaw = true;
                }
                else {
                    section.initial_yaw = 0.0;
                    section.use_initial_yaw = false;
                }

                // Set velocity
                section.initial_surge = _trajectory.surge.at(i-1);

                // ...final point of the section
                section.final_position.x = _trajectory.x.at(i);
                section.final_position.y = _trajectory.y.at(i);
                section.final_position.z = _trajectory.z.at(i);

                // TODO: Pilot is not prepared to use initial and final yaw yet
                // By default, yaw is considered to be the initial one
                //if (_trajectory.yaw.size() > 0) {
                //    section.final_yaw = _trajectory.yaw.at(i);
                //    section.use_final_yaw = true;
                //}
                //else {
                    section.final_yaw = 0.0;
                    section.use_final_yaw = false;
                //}

                // Set final surge
                section.final_surge = _trajectory.surge.at(i);

                // Altitude mode is defined by the initial waypoint
                section.altitude_mode = _trajectory.altitude_mode.at(i-1);
                _section_client->sendGoal(section);

                // Compute timeout
                double distance_to_end_section = distance_to(_trajectory.x.at(i),
                                                             _trajectory.y.at(i),
                                                             _trajectory.z.at(i),
                                                             _trajectory.z.at(i),
                                                             _trajectory.altitude_mode.at(i));
                double min_vel = _min_loscte_vel;
                if (_trajectory.surge.at(i-1) != 0.0 && _trajectory.surge.at(i-1) < _min_loscte_vel) {
                    min_vel = _trajectory.surge.at(i-1);
                }
                double timeout = (2*distance_to_end_section) / min_vel;
                ROS_INFO_STREAM(_name << ": Section timeout = " << timeout << "\n");
                _section_client->waitForResult(ros::Duration(timeout));

                // Move to next waypoint
                i++;
            }
            if(_is_section_running){
                _is_section_running = false;
                // Move to final waypoint on surface if necessary
                if (_trajectory.force_initial_final_waypoints_at_surface) {
                    req.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
                    req.altitude_mode = false;
                    req.blocking = true;
                    req.disable_axis.x = true;
                    req.disable_axis.y = true;
                    req.disable_axis.z = false;
                    req.disable_axis.yaw = true;
                    req.position.x = _trajectory.x.at(_trajectory.x.size()-1);
                    req.position.y = _trajectory.y.at(_trajectory.y.size()-1);
                    req.position.z = 0.0;
                    req.position_tolerance.x = 3.0;
                    req.position_tolerance.y = 3.0;
                    req.position_tolerance.z = 2.0;
                    req.reference = cola2_msgs::NewGoto::Request::REFERENCE_NED;
                    enable_goto(req, res);
                    ROS_ASSERT_MSG(res.success, "Impossible to reach final waypoint");
                }
            }
        }
    }
    else {
        ROS_WARN_STREAM(_name << ": Is trajectory loaded?");
    }
    return true;
}

bool
Captain::disable_trajectory(std_srvs::Empty::Request&,
                            std_srvs::Empty::Response&)
{
    if(_is_waypoint_running) {
        _is_waypoint_running = false;
        _is_trajectory_disabled = true;
        _waypoint_client->cancelGoal();
    }
    else if(_is_section_running) {
        _is_section_running = false;
        _section_client->cancelGoal();
    }
    return true;
}

bool
Captain::enable_keep_position_holonomic(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
    cola2_msgs::NewGoto::Request goto_req;
    cola2_msgs::NewGoto::Response goto_res;

    goto_req.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
    goto_req.altitude_mode = false;
    goto_req.blocking = false;
    goto_req.disable_axis.x = false;
    goto_req.disable_axis.y = false;
    goto_req.disable_axis.z = false;
    goto_req.disable_axis.roll = true;
    goto_req.disable_axis.pitch = true;
    goto_req.disable_axis.yaw = false;
    goto_req.position.x = _nav.x;
    goto_req.position.y = _nav.y;
    goto_req.position.z = _nav.z;
    goto_req.yaw = _nav.yaw;

    // If toloerance is 0.0 position, the waypoint is impossible to reach
    // and therefore, the controller will never finish.
    goto_req.position_tolerance.x = 0.0;
    goto_req.position_tolerance.y = 0.0;
    goto_req.position_tolerance.z = 0.0;
    goto_req.orientation_tolerance.yaw = 0.0;

    ROS_INFO_STREAM(_name << ": Start holonomic keep position at " << _nav.x
                    << ", " << _nav.y << ", " << _nav.z << ", "
                    << _nav.yaw << ".\n");
    _is_holonomic_keep_pose_enabled = true;
    goto_req.reference = cola2_msgs::NewGoto::Request::REFERENCE_NED;
    enable_goto(goto_req, goto_res);

    return true;
}

bool
Captain::enable_keep_position_non_holonomic(std_srvs::Empty::Request&,
                                            std_srvs::Empty::Response&)
{
    cola2_msgs::NewGoto::Request goto_req;
    cola2_msgs::NewGoto::Response goto_res;

    goto_req.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
    goto_req.altitude_mode = false;
    goto_req.blocking = false;
    goto_req.disable_axis.x = false;
    goto_req.disable_axis.y = true;
    goto_req.disable_axis.z = false;
    goto_req.disable_axis.roll = true;
    goto_req.disable_axis.pitch = true;
    goto_req.disable_axis.yaw = false;
    goto_req.position.x = _nav.x;
    goto_req.position.y = _nav.y;
    goto_req.position.z = _nav.z;
    goto_req.yaw = _nav.yaw;

    // If toloerance is 0.0 position, the waypoint is impossible to reach
    // and therefore, the controller will never finish.
    goto_req.position_tolerance.x = 0.0;
    goto_req.position_tolerance.y = 0.0;
    goto_req.position_tolerance.z = 0.0;
    goto_req.orientation_tolerance.yaw = 0.0;

    ROS_INFO_STREAM(_name << ": Start non holonomic keep position at " << _nav.x
                    << ", " << _nav.y << ", " << _nav.z << ", "
                    << _nav.yaw << ".\n");
    _is_holonomic_keep_pose_enabled = true;
    goto_req.reference = cola2_msgs::NewGoto::Request::REFERENCE_NED;
    enable_goto(goto_req, goto_res);

    return true;
}

bool
Captain::disable_keep_position(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
    if (_is_holonomic_keep_pose_enabled) {
        ROS_INFO_STREAM(_name << ": Disable holonomic keep pose.");
        disable_goto(req, res);
        _is_holonomic_keep_pose_enabled = false;
    }

    return true;
}

void
Captain::wait_waypoint()
{
    _waypoint_client->waitForResult();
    _is_waypoint_running = false;
    ROS_INFO_STREAM(_name << ": World Waypoint Request finalized");
}

bool
Captain::check_no_request_running(){
    if(_is_waypoint_running){
        ROS_WARN_STREAM(_name << ": A World Waypoint Request is running!");
        return false;
    }
    else if(_is_section_running){
        ROS_WARN_STREAM(_name << ": A World Section Request is running!");
        return false;
    }
    return true;
}


nav_msgs::Path
Captain::create_path_from_trajectory(Trajectory trajectory)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "/world";

    for(unsigned int i = 0; i < trajectory.x.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        pose.pose.position.x = trajectory.x.at(i);
        pose.pose.position.y = trajectory.y.at(i);
        pose.pose.position.z = trajectory.z.at(i);
        path.poses.push_back(pose);
    }
    return path;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "captain_new");
    Captain captain;
    ros::spin();
    return 0;
}
