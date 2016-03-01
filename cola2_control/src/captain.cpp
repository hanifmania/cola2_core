#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_msgs/WorldSectionReqAction.h>
#include <cola2_msgs/WorldWaypointReqAction.h>
#include <boost/shared_ptr.hpp>
#include <cola2_msgs/String.h>
#include <cola2_msgs/NewGoto.h>
#include <std_srvs/Empty.h>
#include <auv_msgs/GoalDescriptor.h>
#include <boost/thread.hpp>
#include <ros/console.h>
#include "cola2_lib/cola2_navigation/Ned.h"

typedef struct {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<bool> altitude_mode;
    std::string mode;
    unsigned int timeout;
    std::vector<double> wait;
} Trajectory;

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
    ros::MultiThreadedSpinner _spinner;
    Trajectory _trajectory;

    // Services
    ros::ServiceServer _enable_goto_srv;
    ros::ServiceServer _disable_goto_srv;
    ros::ServiceServer _load_trajectory_srv;
    ros::ServiceServer _enable_trajectory_srv;
    ros::ServiceServer _disable_trajectory_srv;
    ros::ServiceServer _enable_keep_position_holonomic_srv;
    ros::ServiceServer _enable_keep_position_non_holonomic_srv;
    ros::ServiceServer _disable_keep_position_srv;

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
    void getConfig();
    template<typename T> void getParam(std::string, T&);

    // ... services callbacks
    bool enable_goto(cola2_msgs::NewGoto::Request&,
                     cola2_msgs::NewGoto::Response&);

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

    template <typename ParamType>
    bool loadVector(const std::string, std::vector<ParamType>&);

};


Captain::Captain():
    _is_waypoint_running(false),
    _spinner(2)
{
    // Node name
    _name = ros::this_node::getName();

    // Get config
    getConfig();

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
    _load_trajectory_srv = _n.advertiseService("/cola2_control/load_trajectory", &Captain::load_trajectory, this);
    _enable_trajectory_srv = _n.advertiseService("/cola2_control/enable_trajectory", &Captain::enable_trajectory, this);
    _disable_trajectory_srv = _n.advertiseService("/cola2_control/disable_trajectory", &Captain::disable_trajectory, this);
    _enable_keep_position_holonomic_srv = _n.advertiseService("/cola2_control/enable_keep_position_holonomic", &Captain::enable_keep_position_holonomic, this);
    _enable_keep_position_non_holonomic_srv = _n.advertiseService("/cola2_control/enable_keep_position_non_holonomic", &Captain::enable_keep_position_non_holonomic, this);
    _disable_keep_position_srv = _n.advertiseService("/cola2_control/disable_keep_position", &Captain::disable_keep_position, this);

    // test();
    _spinner.spin();
}


void
Captain::getConfig() {
    // Default config here
    config.section_server_name = "section_server";
}


template<typename T> void
Captain::getParam(std::string param_name, T& param_var) {
    // Display a message if a parameter is not found in the param server
    if (!ros::param::getCached(param_name, param_var)) {
        ROS_WARN_STREAM(_name << ": invalid parameter for " <<
            param_name << " in param server! Using default value of " <<
            param_var);
    }
}


bool
Captain::enable_goto(cola2_msgs::NewGoto::Request &req,
                     cola2_msgs::NewGoto::Response &res)
{
    if(!_is_waypoint_running){
        _is_waypoint_running = true;
        cola2_msgs::WorldWaypointReqGoal waypoint;
        waypoint.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
        waypoint.goal.requester = _name;
        waypoint.altitude_mode = req.altitude_mode;
        waypoint.altitude = req.altitude;

        // TODO: Check req.reference to transform req.position
        //       to appropiate reference frame.
        waypoint.position.north = req.position.x;
        waypoint.position.east = req.position.y;
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

        // TODO: Compute realistic timeout
        waypoint.timeout = 10000;

        ROS_WARN_STREAM(_name << ": Send WorldWaypointRequest");
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
        ROS_WARN_STREAM(_name << ": Another WorldWaypointRequest is running!");
        res.success = false;
        return false;
    }
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
    bool valid_mission = true ;
    bool _is_trajectory_global = false;
    double ned_latitude;
    double ned_longitude;

    // TODO: Compte! Si es passa d'una mission local a global, s'han de borrar
    //       els camps trajectory/north, trajectory/east, trajectory/latitude, ...
    if(loadVector("trajectory/north", _trajectory.x) &&
       loadVector("trajectory/east", _trajectory.y)){
        ROS_INFO_STREAM(_name << ": loading local trajectory ...");
    }
    else if (loadVector("trajectory/latitude", _trajectory.x) &&
             loadVector("trajectory/longitude", _trajectory.y)){
        ROS_INFO_STREAM(_name << ": loading global trajectory ...");
        _is_trajectory_global = true;
        // Load NED origin
        if (!ros::param::getCached("navigator/ned_latitude", ned_latitude)) valid_mission = false;
        if (!ros::param::getCached("navigator/ned_longitude", ned_longitude)) valid_mission = false;
    }
    else {
        ROS_INFO_STREAM(_name << ": invalid trajectory");
        return false;
    }

    if(!loadVector("trajectory/z", _trajectory.z)) valid_mission = false;
    if(!loadVector("trajectory/altitude_mode", _trajectory.altitude_mode)) valid_mission = false;
    if (!ros::param::getCached("trajectory/mode", _trajectory.mode)) valid_mission = false;
    if (!ros::param::getCached("trajectory/mode", _trajectory.mode)) valid_mission = false;
    if(!loadVector("trajectory/wait", _trajectory.wait)) valid_mission = false;

    ROS_ASSERT_MSG(_trajectory.x.size() == _trajectory.y.size(), "Different mission array sizes");
    ROS_ASSERT_MSG(_trajectory.x.size() == _trajectory.z.size(), "Different mission array sizes");
    ROS_ASSERT_MSG(_trajectory.x.size() == _trajectory.wait.size(), "Different mission array sizes");
    ROS_ASSERT_MSG(_trajectory.x.size() == _trajectory.altitude_mode.size(), "Different mission array sizes");
    ROS_ASSERT_MSG(valid_mission, "Invalid mission parameter");

    // If the trajectory is defined globally, tranform from lat/lon to NED.
    if(_is_trajectory_global){
        Ned ned(ned_latitude, ned_longitude, 0.0);
        double north, east, depth;
        for(unsigned int i = 0; i < _trajectory.x.size(); i++){
            ned.geodetic2Ned(_trajectory.x.at(i), _trajectory.y.at(i), 0.0,
                             north, east, depth);
            _trajectory.x.at(i) = north;
            _trajectory.y.at(i) = east;
        }
    }

    std::cout << "\nx: ";
    for(int i = 0; i < _trajectory.x.size(); i++) std::cout << _trajectory.x.at(i) << " ";
    std::cout << "\ny: ";
    for(int i = 0; i < _trajectory.y.size(); i++) std::cout << _trajectory.y.at(i) << " ";
    std::cout << "\nz: ";
    for(int i = 0; i < _trajectory.z.size(); i++) std::cout << _trajectory.z.at(i) << " ";
    std::cout << "\naltitude_mode: ";
    for(int i = 0; i < _trajectory.altitude_mode.size(); i++) std::cout << _trajectory.altitude_mode.at(i) << " ";
    std::cout << "\nwait: ";
    for(int i = 0; i < _trajectory.wait.size(); i++) std::cout << _trajectory.wait.at(i) << " ";

    std::cout << "\n" << _trajectory.mode << "\n";
    std::cout << _trajectory.timeout << "\n";
    std::cout << "valid_mission: " << valid_mission << "\n";

    return valid_mission;
}

template <typename ParamType>
bool Captain::loadVector(const std::string param_name,
                         std::vector<ParamType> &data)
{
    // Take the vector param and copy to a std::vector<double>
    XmlRpc::XmlRpcValue my_list ;
    if (ros::param::getCached(param_name, my_list)) {
        ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray) ;

        for (int32_t i = 0; i < my_list.size(); ++i) {
            // ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) ;
            data.push_back(static_cast<ParamType>(my_list[i])) ;
        }
    }
    else {
        ROS_FATAL_STREAM(_name << ": invalid parameters for " << param_name << " in param server!");
        return false;
    }
    return true;
}


bool
Captain::enable_trajectory(std_srvs::Empty::Request&,
                           std_srvs::Empty::Response&)
{
    return false;
}

bool
Captain::disable_trajectory(std_srvs::Empty::Request&,
                            std_srvs::Empty::Response&)
{
    return false;
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
    ROS_WARN_STREAM(_name << ": World Waypoint Request finalized");
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
