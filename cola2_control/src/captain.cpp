#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_msgs/SectionAction.h>
#include <boost/shared_ptr.hpp>


class Captain {
private:
    // Node handle
    ros::NodeHandle nh;

    // Node name
    std::string node_name;

    // Actionlib client
    boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::SectionAction> > section_client;

    // Config
    struct {
        std::string section_server_name;
    } config;

    // Methods
    void getConfig();
    template<typename T> void getParam(std::string, T&);

public:
    Captain();
};


Captain::Captain() {
    // Node name
    node_name = ros::this_node::getName();

    // Get config
    getConfig();

    // Actionlib client. Smart pointer is used so that client construction is
    // delayed after configuration is loaded
    section_client = boost::shared_ptr<actionlib::SimpleActionClient<
        cola2_msgs::SectionAction> >(
        new actionlib::SimpleActionClient<cola2_msgs::SectionAction>(
        config.section_server_name, true));
    section_client->waitForServer();  // Wait for infinite time

    // Create section
    cola2_msgs::SectionGoal section;
    section.priority = 30;
    section.initial_surge = 0.3;
    //section.use_initial_yaw = true;
    section.final_surge = 0.3;
    section.controller_type = cola2_msgs::SectionGoal::DUBINS;
    section.use_final_yaw = true;
    section.initial_z = 1.0;
    section.final_z   = 1.0;
    //section.timeout = 10.0;

    // Section 1
    section.initial_x = 1.0;
    section.initial_y = 0.0;
    //section.initial_z = 0.0;
    section.final_x = 3.91868;
    section.final_y = -2.30624;
    section.final_yaw = -1.33743;
    //section.final_z   = 0.5;
    section_client->sendGoal(section);

    //ros::Duration(5.0).sleep();
    //section_client->cancelGoal();

    section_client->waitForResult(ros::Duration(120.0));

    // Section 2
    section.initial_x = 3.91868;
    section.initial_y =  -2.30624;
    //section.initial_z = 0.5;
    section.final_x = 6.39008;
    section.final_y = -12.7036;
    section.final_yaw = -1.33743;
    //section.final_z   = 1.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    //ros::shutdown();

    // Section 3
    section.initial_x = 6.39008;
    section.initial_y =  -12.7036;
    //section.initial_z = 1.0;
    section.final_x = 8.32324;
    section.final_y = -14.8433;
    section.final_yaw = -0.334722;
    //section.final_z   = 1.5;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 4
    section.initial_x = 8.32324;
    section.initial_y =  -14.8433;
    //section.initial_z = 1.5;
    section.final_x = 8.36839;
    section.final_y = -14.8586;
    section.final_yaw = -0.318834;
    //section.final_z   = 2.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 5
    section.initial_x = 8.36839;
    section.initial_y =  -14.8586;
    //section.initial_z = 2.0;
    section.final_x = 20.8479;
    section.final_y = -18.978;
    section.final_yaw = -0.318834;
    //section.final_z   = 2.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(220.0));

    // Section 6
    section.initial_x = 20.8479;
    section.initial_y =  -18.978;
    //section.initial_z = 2.0;
    section.final_x = 20.8938;
    section.final_y = -18.9936;
    section.final_yaw = -0.33496;
    //section.final_z   = 1.5;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 7
    section.initial_x = 20.8938;
    section.initial_y =  -18.9936;
    //section.initial_z = 1.5;
    section.final_x = 22.4772;
    section.final_y = -20.2787;
    section.final_yaw = -1.02856;
    //section.final_z   = 1.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 8
    section.initial_x = 22.4772;
    section.initial_y =  -20.2787;
    //section.initial_z = 1.0;
    section.final_x = 24.8418;
    section.final_y = -24.2034;
    section.final_yaw = -1.02856;
    //section.final_z   = 0.5;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 9
    section.initial_x = 24.8418;
    section.initial_y =  -24.2034;
    //section.initial_z = 0.5;
    section.final_x = 25.0808;
    section.final_y = -26.8056;
    section.final_yaw = -1.92982;
    //section.final_z   = 0.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 10
    section.initial_x = 25.0808;
    section.initial_y =  -26.8056;
    //section.initial_z = 1.5;
    section.final_x = 25.0808;
    section.final_y = -26.8056;
    section.final_yaw = -1.92982;
    //section.final_z   = 1.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 11
    section.initial_x = 25.0808;
    section.initial_y =  -26.8056;
    //section.initial_z = 1.0;
    section.final_x = 25.0808;
    section.final_y = -26.8056;
    section.final_yaw = -1.92982;
    //section.final_z   = 0.5;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

    // Section 12
    section.initial_x = 25.0808;
    section.initial_y =  -26.8056;
    //section.initial_z = 0.5;
    section.final_x = 25.0;
    section.final_y = -27.0;
    section.final_yaw = -2.0;
    //section.final_z   = 0.0;
    section_client->sendGoal(section);
    section_client->waitForResult(ros::Duration(120.0));

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
    ros::init(argc, argv, "captain");
    Captain captain;
    ros::spin();
    return 0;
}
