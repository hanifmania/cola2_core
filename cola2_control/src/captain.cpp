#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_msgs/WorldSectionReqAction.h>
#include <cola2_msgs/WorldWaypointReqAction.h>
#include <boost/shared_ptr.hpp>
#include <cola2_msgs/String.h>
#include <cola2_msgs/NewGoto.h>
#include <cola2_msgs/Submerge.h>
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

typedef struct {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<bool> altitude_mode;
    std::vector<double> yaw;
    std::vector<double> surge;
    std::string mode;
    unsigned int timeout;
    std::vector<double> wait;
    bool valid_trajectory;
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
    control::vector6d _nav;
    CaptainConfig _config;

    // Publishers
    ros::Publisher _pub_path;

    // Services
    ros::ServiceServer _enable_goto_srv;
    ros::ServiceServer _disable_goto_srv;
    ros::ServiceServer _submerge_srv;
    ros::ServiceServer _load_trajectory_srv;
    ros::ServiceServer _enable_trajectory_srv;
    ros::ServiceServer _disable_trajectory_srv;
    ros::ServiceServer _enable_keep_position_holonomic_srv;
    ros::ServiceServer _enable_keep_position_non_holonomic_srv;
    ros::ServiceServer _disable_keep_position_srv;

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

    void getConfig();

    nav_msgs::Path create_path_from_trajectory(Trajectory trajectory);

    double distance_to(const double, const double);

    // ... services callbacks
    bool enable_goto(cola2_msgs::NewGoto::Request&,
                     cola2_msgs::NewGoto::Response&);

    bool submerge(cola2_msgs::Submerge::Request&,
                  cola2_msgs::Submerge::Response&);

    bool disable_goto(std_srvs::Empty::Request&,
                      std_srvs::Empty::Response&);

    bool load_trajectory(std_srvs::Empty::Request&,
                         std_srvs::Empty::Response&);

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
};


Captain::Captain():
    _is_waypoint_running(false),
    _is_section_running(false),
    _is_trajectory_disabled(false),
    _spinner(2)
{
    // Node name
    _name = ros::this_node::getName();

    // Get config
    getConfig();
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
    _enable_trajectory_srv = _n.advertiseService("/cola2_control/enable_trajectory", &Captain::enable_trajectory, this);
    _disable_trajectory_srv = _n.advertiseService("/cola2_control/disable_trajectory", &Captain::disable_trajectory, this);
    _enable_keep_position_holonomic_srv = _n.advertiseService("/cola2_control/enable_keep_position_g500", &Captain::enable_keep_position_holonomic, this);
    _enable_keep_position_non_holonomic_srv = _n.advertiseService("/cola2_control/enable_keep_position_s2", &Captain::enable_keep_position_non_holonomic, this);
    _disable_keep_position_srv = _n.advertiseService("/cola2_control/disable_keep_position", &Captain::disable_keep_position, this);

    // Subscriber

    _sub_nav = _n.subscribe("/cola2_navigation/nav_sts", 1, &Captain::update_nav, this);

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
}


double
Captain::distance_to(const double x, const double y)
{
    return sqrt(pow(x - _nav.x, 2) + pow(y - _nav.y, 2));
}

void
Captain::getConfig() {
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
}


bool
Captain::enable_goto(cola2_msgs::NewGoto::Request &req,
                     cola2_msgs::NewGoto::Response &res)
{
    if(check_no_request_running()){

        // Check max distance to waypoint
        double distance_to_waypoint = distance_to(req.position.x, req.position.y);
        if(distance_to_waypoint > _config.max_distance_to_waypoint) {
            ROS_WARN_STREAM(_name << ": Max distance to waypoint is " <<
                            _config.max_distance_to_waypoint << " requested waypoint is at "
                            << distance_to(req.position.x, req.position.y));
            return false;
        }
        distance_to_waypoint += fabs(req.position.z - _nav.z);

        _is_waypoint_running = true;
        cola2_msgs::WorldWaypointReqGoal waypoint;
        waypoint.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
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
            ROS_WARN_STREAM(_name << ": Invalid GOTO refenrece. REFERENCE_VEHICLE not yet implemented.");
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

        // Choose WorldWaypointReq mode taking into account disable axis
        if(!req.disable_axis.x && req.disable_axis.y && !req.disable_axis.yaw) {
            // X, Z, Yaw Goto (with or wiyhou Z)
            waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::GOTO;
        }
        else if (!req.disable_axis.x && !req.disable_axis.y){
            // Holonomic X, Y, Z, Yaw goto (with or without Z and Yaw)
            waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::HOLONOMIC_GOTO;
        }
        else if(req.disable_axis.x && req.disable_axis.y && !req.disable_axis.z) {
            // Submerge Z, Yaw (with or without Yaw)
            waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::GOTO;
        }

        // TODO: Check timeout
        waypoint.timeout = distance_to_waypoint * 10.0;

        ROS_INFO_STREAM(_name << ": Send WorldWaypointRequest");
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
Captain::load_trajectory(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res)
{
    // Load mission file from PARAM SERVER.It means that the YAML file must
    // be firts loaded to the param server!
    bool valid_trajectory = true ;
    bool is_trajectory_global = false;
    double ned_latitude;
    double ned_longitude;
    Trajectory trajectory;

    // TODO: Compte! Si es passa d'una mission local a global, s'han de borrar
    //       els camps trajectory/north, trajectory/east, trajectory/latitude, ...
    if(cola2::rosutil::loadVector("trajectory/north", trajectory.x) &&
       cola2::rosutil::loadVector("trajectory/east", trajectory.y)){
        ROS_INFO_STREAM(_name << ": loading local trajectory ...");
    }
    else if (cola2::rosutil::loadVector("trajectory/latitude", trajectory.x) &&
             cola2::rosutil::loadVector("trajectory/longitude", trajectory.y)){
        ROS_INFO_STREAM(_name << ": loading global trajectory ...");
        is_trajectory_global = true;
    }
    else {
        ROS_INFO_STREAM(_name << ": invalid trajectory");
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

    ROS_ASSERT_MSG(valid_trajectory, "Invalid mission parameter");

    // If the trajectory is defined globally, tranform from lat/lon to NED.
    if(is_trajectory_global){
        double north, east, depth;
        for(unsigned int i = 0; i < trajectory.x.size(); i++){
            _ned->geodetic2Ned(trajectory.x.at(i), trajectory.y.at(i), 0.0,
                               north, east, depth);
            trajectory.x.at(i) = north;  // - 15.0; //TODO: treure aixo!!!!
            trajectory.y.at(i) = east;  // - 50.0;
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
        // Move to initial waypoint on surface and then submerge to it
        cola2_msgs::NewGoto::Request req;
        cola2_msgs::NewGoto::Response res;
        req.altitude_mode = false;
        req.blocking = true;
        req.disable_axis.x = false;
        req.disable_axis.y = true;
        req.disable_axis.z = false;
        req.disable_axis.yaw = false;
        req.position.x = _trajectory.x.at(0);
        req.position.y = _trajectory.y.at(0);
        req.position.z = 0.0;
        req.position_tolerance.x = 3.0;
        req.position_tolerance.y = 3.0;
        req.position_tolerance.z = 2.0;
        req.reference = cola2_msgs::NewGoto::Request::REFERENCE_NED;
        enable_goto(req, res);
        ROS_ASSERT_MSG(res.success, "Impossible to reach initial waypoint");
        if (!_is_trajectory_disabled) {
            // Submerge unti initial waypoint
            req.altitude_mode = _trajectory.altitude_mode.at(0);
            req.blocking = true;
            req.disable_axis.x = true;
            req.yaw = _nav.yaw;
            req.position.z = _trajectory.z.at(0);
            enable_goto(req, res);
            ROS_ASSERT_MSG(res.success, "Impossible to reach initial waypoint");
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
                cola2_msgs::WorldSectionReqGoal::DUBINS;
            }
            else {
                ROS_ASSERT_MSG(false, "Invalid trajectory mode!");
            }
            section.disable_z = false;

            section.tolerance.x = _config.tolerance.x;
            section.tolerance.y = _config.tolerance.y;
            section.tolerance.z = _config.tolerance.z;

            unsigned int i = 1;
            while (i < _trajectory.x.size() &&  _is_section_running) {
                // For each pair of waypoints:

                // ...initial point of the section
                section.initial_position.x = _trajectory.x.at(i-1);
                section.initial_position.y = _trajectory.y.at(i-1);
                section.initial_position.z = _trajectory.z.at(i-1);
                if (_trajectory.yaw.size() > 0) {
                    section.initial_yaw = _trajectory.yaw.at(i-1);
                    section.use_initial_yaw = true;
                }
                else {
                    section.initial_yaw = 0.0;
                    section.use_initial_yaw = false;
                }
                if (_trajectory.surge.size() > 0 )
                    section.initial_surge = _trajectory.surge.at(i-1);
                else
                    section.initial_surge = 0;

                // ...final point of the section
                section.final_position.x = _trajectory.x.at(i);
                section.final_position.y = _trajectory.y.at(i);
                section.final_position.z = _trajectory.z.at(i);

                // TODO: Pilot is not prepared to use initial and final yaw yet.
                if (_trajectory.yaw.size() > 0) {
                    section.final_yaw = _trajectory.yaw.at(i);
                    section.use_final_yaw = true;
                }
                else {
                    section.final_yaw = 0.0;
                    section.use_final_yaw = false;
                }
                if (_trajectory.surge.size() > 0 )
                    section.final_surge = _trajectory.surge.at(i);
                else
                    section.final_surge = 0;

                // Altitude mode is defined by the initial waypoint
                section.altitude_mode = _trajectory.altitude_mode.at(i-1);
                _section_client->sendGoal(section);

                // TODO: Check time!
                double section_longitude = sqrt(pow(section.final_position.x - section.initial_position.x, 2) +
                                                pow(section.final_position.y - section.initial_position.y, 2) +
                                                pow(section.final_position.z - section.initial_position.z, 2));

                _section_client->waitForResult(ros::Duration(section_longitude * 6.0));

                // Move to next waypoint
                i++;
            }
            if(_is_section_running){
                _is_section_running = false;
                // Move to final waypoint on surface
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
Captain::enable_keep_position_holonomic(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&)
{
    return false;
}

bool
Captain::enable_keep_position_non_holonomic(std_srvs::Empty::Request&,
                                            std_srvs::Empty::Response&)
{
    return false;
}

bool
Captain::disable_keep_position(std_srvs::Empty::Request&,
                               std_srvs::Empty::Response&)
{
    return false;
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


void
Captain::test()
{
    // Create Waypoint
    cola2_msgs::WorldWaypointReqGoal waypoint;
    waypoint.goal.priority = 10;
    waypoint.goal.requester = "captain";
    waypoint.altitude_mode = false;
    waypoint.position.north = 10.0;
    waypoint.position.east = 10.0;
    waypoint.position.depth = 2.0;
    waypoint.orientation.yaw = 1.57;
    waypoint.disable_axis.x = false;
    waypoint.disable_axis.z = false;
    waypoint.disable_axis.yaw = false;
    waypoint.position_tolerance.x = 3.0;
    waypoint.position_tolerance.y = 3.0;
    waypoint.position_tolerance.z = 1.5;
    waypoint.orientation_tolerance.yaw = 0.1;
    waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::HOLONOMIC_GOTO;
    waypoint.timeout = 100;
    std::cout << "Send waypoint 1\n";
    _waypoint_client->sendGoal(waypoint);
    _waypoint_client->waitForResult(ros::Duration(120.0));

    waypoint.position.north = 0.0;
    waypoint.position.east = 0.0;
    std::cout << "Send waypoint 2\n";
    _waypoint_client->sendGoal(waypoint);
    _waypoint_client->waitForResult(ros::Duration(120.0));

    waypoint.position.north = -10.0;
    waypoint.position.east = -10.0;
    std::cout << "Send waypoint 3\n";
    // _waypoint_client->sendGoal(waypoint);
    // _waypoint_client->waitForResult(ros::Duration(120.0));

    // Create section
    cola2_msgs::WorldSectionReqGoal section;
    section.priority = 30;
    section.initial_surge = 0.3;
    //section.use_initial_yaw = true;
    section.final_surge = 0.3;
    section.controller_type = cola2_msgs::WorldSectionReqGoal::LOSCTE;
    section.use_final_yaw = true;
    section.initial_position.z = 1.0;
    section.final_position.z   = 1.0;
    section.disable_z = false;
    section.tolerance.x = 3.0;
    section.tolerance.y = 3.0;
    section.tolerance.z = 1.5;

    //section.timeout = 10.0;


    // Section 1
    section.initial_position.x = 0;
    section.initial_position.y = 0;
    section.initial_yaw = 0;
    section.final_position.x = 15;
    section.final_position.y = 0;
    section.final_yaw = -1.18925;
    //section.final_z   = 0.5;
    std::cout << "Send Section 1\n";
    _section_client->sendGoal(section);

    //ros::Duration(5.0).sleep();
    //_section_client->cancelGoal();

    _section_client->waitForResult(ros::Duration(120.0));

    // Section 2
    section.initial_position.x = 15.0;
    section.initial_position.y = .0;
    section.initial_yaw = -1.18925;
    section.final_position.x = 15.0;
    section.final_position.y = -15.0;
    section.final_yaw = -1.18925;
    //section.final_z   = 1.0;
    std::cout << "Send Section 2\n";
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));

    //ros::shutdown();

    // Section 3
    section.initial_position.x = 15;
    section.initial_position.y = -15;
    section.initial_yaw = -1.18925;
    section.final_position.x = 0;
    section.final_position.y = -15;
    section.final_yaw = -0.847483;
    //section.final_z   = 1.5;
    std::cout << "Send Section 3\n";
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));

    // Section 4
    section.initial_position.x = 0;
    section.initial_position.y = -15;
    section.initial_yaw = -0.847483;
    section.final_position.x = 0;
    section.final_position.y = 0;
    section.final_yaw = -0.241298;
    //section.final_z   = 2.0;
    std::cout << "Send Section 4\n";
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));

    // Section 5
    section.initial_position.x = 10.4831;
    section.initial_position.y = -15.223;
    section.initial_yaw = -0.241298;
    section.final_position.x = 19.4962;
    section.final_position.y = -17.441;
    section.final_yaw = -0.241298;
    //section.final_z   = 2.0;
    std::cout << "Send Section 5\n";
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(220.0));

    // Section 6
    section.initial_position.x = 19.4962;
    section.initial_position.y = -17.441;
    section.initial_yaw = -0.241298;
    section.final_position.x = 19.6668;
    section.final_position.y = -17.4884;
    section.final_yaw = -0.300332;
    //section.final_z   = 1.5;
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));

    // Section 7
    section.initial_position.x = 19.6668;
    section.initial_position.y = -17.4884;
    section.initial_yaw = -0.300332;
    section.final_position.x = 21.2979;
    section.final_position.y= -18.7242;
    section.final_yaw = -0.996449;
    //section.final_z   = 1.0;
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));

    // Section 8
    section.initial_position.x = 21.2979;
    section.initial_position.y = -18.7242;
    section.initial_yaw = -0.996449;
    section.final_position.x = 24.7907;
    section.final_position.y= -24.1217;
    section.final_yaw = -0.996449;
    //section.final_z   = 0.5;
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));

    // Section 9
    section.initial_position.x = 24.7907;
    section.initial_position.y = -24.1217;
    section.initial_yaw = -0.996449;
    section.final_position.x = 25.1677;
    section.final_position.y= -26.5361;
    section.final_yaw = -1.8354;
    //section.final_z   = 0.0;
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));

    // Section 10
    section.initial_position.x = 25.1677;
    section.initial_position.y = -26.5361;
    section.initial_yaw = -1.8354;
    section.final_position.x = 25;
    section.final_position.y= -27;
    section.final_yaw = -2;
    //section.final_z   = 1.0;
    _section_client->sendGoal(section);
    _section_client->waitForResult(ros::Duration(120.0));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "captain_new");
    Captain captain;
    ros::spin();
    return 0;
}
