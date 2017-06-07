#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <auv_msgs/WorldWaypointReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <cola2_msgs/Setpoints.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "../cola2_rosutils/DiagnosticHelper.h"
#include "./IAUVController.h"
#include "./Request.h"

class IAUVROSController
{
 public:
  IAUVROSController(const std::string name, const std::string frame_id);

  void initBase(IAUVController * auv_controller_ptr, double period);

  bool enablePoseController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool disablePoseController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool enableVelocityController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool disableVelocityController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool enableThrusterAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool disableThrusterAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool enableFinAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool disableFinAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void checkDiagnostics(const ros::TimerEvent& event);

  void timerCallback(const ros::TimerEvent& event);

  void updateNav(const ros::MessageEvent<auv_msgs::NavSts const> & msg);
  void updateWWR(const ros::MessageEvent<auv_msgs::WorldWaypointReq const> & msg);
  void updateBVR(const ros::MessageEvent<auv_msgs::BodyVelocityReq const> & msg);
  void updateBFR(const ros::MessageEvent<auv_msgs::BodyForceReq const> & msg);

 private:
  void publishThrusterSetpoint(const Eigen::VectorXd setpoint, const ros::Time now);
  void publishFinSetpoint(const Eigen::VectorXd setpoint, const ros::Time now);
  void publishMergedPose(const Request pose, const ros::Time now);
  void publishMergedTwist(const Request twist, const ros::Time now);
  void publishMergedWrench(const Request response, const ros::Time now);

  // Name
  std::string _name;

  // Frame id
  std::string _frame_id;

  // Controller frequency
  double _frequency;

  // Node handle
  ros::NodeHandle _n;

  // Diagnostics
  cola2::rosutils::DiagnosticHelper _diagnostic;

  // Publisher
  ros::Publisher _pub_wrench;
  ros::Publisher _pub_merged_pose;
  ros::Publisher _pub_merged_twist;
  ros::Publisher _pub_thrusters_setpoint;
  ros::Publisher _pub_fins_setpoint;
  ros::Publisher _pub_thrusters_state;

  // Subscriber
  ros::Subscriber _sub_nav_data;
  ros::Subscriber _sub_ww_req;
  ros::Subscriber _sub_bv_req;
  ros::Subscriber _sub_bf_req;
  bool _are_thrusters_killed;

  // Timers
  ros::Timer _timer;
  ros::Timer _check_diagnostics;

  // Services
  ros::ServiceServer _enable_pose_controller_srv;
  ros::ServiceServer _disable_pose_controller_srv;
  ros::ServiceServer _enable_velocity_controller_srv;
  ros::ServiceServer _disable_velocity_controller_srv;
  ros::ServiceServer _enable_thruster_allocator_srv;
  ros::ServiceServer _disable_thruster_allocator_srv;
  ros::ServiceServer _enable_fin_allocator_srv;
  ros::ServiceServer _disable_fin_allocator_srv;

  // AUV controller ptr.
  IAUVController *_auv_controller;

  // Estimated total altitude
  double _last_altitude;
  double _last_altitude_age;
  double _last_depth;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_
