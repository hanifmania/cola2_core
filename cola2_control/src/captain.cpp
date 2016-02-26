#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_msgs/WorldSectionReqAction.h>
#include <cola2_msgs/WorldWaypointReqAction.h>
#include <boost/shared_ptr.hpp>


class Captain {
public:
    Captain();

private:
    // Node handle
    ros::NodeHandle nh;

    // Node name
    std::string node_name;

    // Actionlib client
    boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldSectionReqAction> > section_client;

    boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldWaypointReqAction> > waypoint_client;

    // Config
    struct {
        std::string section_server_name;
    } config;

    // Methods
    void getConfig();
    template<typename T> void getParam(std::string, T&);
};


Captain::Captain() {
    // Node name
    node_name = ros::this_node::getName();

    // Get config
    getConfig();

    // Actionlib client. Smart pointer is used so that client construction is
    // delayed after configuration is loaded
    std::cout << "Wait for action lib (Pilot)\n";
    section_client = boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldSectionReqAction> >(
        new actionlib::SimpleActionClient<cola2_msgs::WorldSectionReqAction>(
        "world_section_req", true));
    section_client->waitForServer();  // Wait for infinite time

    waypoint_client = boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::WorldWaypointReqAction> >(
        new actionlib::SimpleActionClient<cola2_msgs::WorldWaypointReqAction>(
        "world_waypoint_req", true));
    waypoint_client->waitForServer();  // Wait for infinite time


    // Create Waypoint
    cola2_msgs::WorldWaypointReqGoal waypoint;
    waypoint.goal.priority = 10;
    waypoint.goal.requester = "captain";
    waypoint.altitude_mode = false;
    waypoint.position.north = 10.0;
    waypoint.position.east = 10.0;
    waypoint.position.depth = 2.0;
    waypoint.disable_axis.x = false;
    waypoint.disable_axis.z = false;
    waypoint.position_tolerance.x = 3.0;
    waypoint.position_tolerance.y = 3.0;
    waypoint.position_tolerance.z = 1.5;
    waypoint.controller_type = cola2_msgs::WorldWaypointReqGoal::GOTO;
    waypoint.timeout = 100;
    std::cout << "Send waypoint 1\n";
    // waypoint_client->sendGoal(waypoint);
    // waypoint_client->waitForResult(ros::Duration(120.0));

    waypoint.position.north = 0.0;
    waypoint.position.east = 0.0;
    std::cout << "Send waypoint 2\n";
    // waypoint_client->sendGoal(waypoint);
    // waypoint_client->waitForResult(ros::Duration(120.0));

    waypoint.position.north = -10.0;
    waypoint.position.east = -10.0;
    std::cout << "Send waypoint 3\n";
    // waypoint_client->sendGoal(waypoint);
    // waypoint_client->waitForResult(ros::Duration(120.0));

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
    section_client->sendGoal(section);

    //ros::Duration(5.0).sleep();
    //section_client->cancelGoal();

    section_client->waitForResult(ros::Duration(120.0));

    // Section 2
    section.initial_position.x = 15.0;
    section.initial_position.y = .0;
    section.initial_yaw = -1.18925;
    section.final_position.x = 15.0;
    section.final_position.y = -15.0;
    section.final_yaw = -1.18925;
    //section.final_z   = 1.0;
    std::cout << "Send Section 2\n";
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

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
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 4
    section.initial_position.x = 0;
    section.initial_position.y = -15;
    section.initial_yaw = -0.847483;
    section.final_position.x = 0;
    section.final_position.y = 0;
    section.final_yaw = -0.241298;
    //section.final_z   = 2.0;
    std::cout << "Send Section 4\n";
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 5
    section.initial_position.x = 10.4831;
    section.initial_position.y = -15.223;
    section.initial_yaw = -0.241298;
    section.final_position.x = 19.4962;
    section.final_position.y = -17.441;
    section.final_yaw = -0.241298;
    //section.final_z   = 2.0;
    std::cout << "Send Section 5\n";
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(220.0));

    // Section 6
    section.initial_position.x = 19.4962;
    section.initial_position.y = -17.441;
    section.initial_yaw = -0.241298;
    section.final_position.x = 19.6668;
    section.final_position.y = -17.4884;
    section.final_yaw = -0.300332;
    //section.final_z   = 1.5;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 7
    section.initial_position.x = 19.6668;
    section.initial_position.y = -17.4884;
    section.initial_yaw = -0.300332;
    section.final_position.x = 21.2979;
    section.final_position.y= -18.7242;
    section.final_yaw = -0.996449;
    //section.final_z   = 1.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 8
    section.initial_position.x = 21.2979;
    section.initial_position.y = -18.7242;
    section.initial_yaw = -0.996449;
    section.final_position.x = 24.7907;
    section.final_position.y= -24.1217;
    section.final_yaw = -0.996449;
    //section.final_z   = 0.5;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 9
    section.initial_position.x = 24.7907;
    section.initial_position.y = -24.1217;
    section.initial_yaw = -0.996449;
    section.final_position.x = 25.1677;
    section.final_position.y= -26.5361;
    section.final_yaw = -1.8354;
    //section.final_z   = 0.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 10
    section.initial_position.x = 25.1677;
    section.initial_position.y = -26.5361;
    section.initial_yaw = -1.8354;
    section.final_position.x = 25;
    section.final_position.y= -27;
    section.final_yaw = -2;
    //section.final_z   = 1.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

//    // Section 11
//    section.initial_x = 24.788;
//    section.initial_y = -24.1175;
//    //section.initial_z = 1.0;
//    section.final_position.x = 25;
//    section.final_position.y= -27;
//    section.final_yaw = -2;
//    //section.final_z   = 0.5;
//    section_client->sendGoal(section);
//    section_client->waitForResult(ros::Duration(120.0));

//    // Section 12
//    section.initial_x = 25.0808;
//    section.initial_y =  -26.8056;
//    //section.initial_z = 0.5;
//    section.final_position.x = 25.0;
//    section.final_position.y= -27.0;
//    section.final_yaw = -2.0;
//    //section.final_z   = 0.0;
//    section_client->sendGoal(section);
//    section_client->waitForResult(ros::Duration(120.0));

    ros::shutdown();
}


void
Captain::getConfig() {
    // Default config here
    config.section_server_name = "section_server";

    // Load config from param server
    //getParam("captain/section_server", config.section_server_name);
}


template<typename T> void
Captain::getParam(std::string param_name, T& param_var) {
    // Display a message if a parameter is not found in the param server
    if (!ros::param::getCached(param_name, param_var)) {
        ROS_WARN_STREAM(node_name << ": invalid parameter for " <<
            param_name << " in param server! Using default value of " <<
            param_var);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "captain_new");
    Captain captain;
    ros::spin();
    return 0;
}
