#include "cola2_lib/cola2_control/IAUVROSController.h"

IAUVROSController::IAUVROSController(const std::string name, const std::string frame_id):
  _name(name),
  _frame_id(frame_id),
  _diagnostic(_n, name, "soft"),
  _last_altitude(0.5),
  _last_altitude_age(0.0),
  _last_depth(0.0)
{
}

void IAUVROSController::initBase(IAUVController * auv_controller_ptr, double period)
{
  // Init pointer to AUV controller
  _auv_controller = auv_controller_ptr;

  // Save controller frequency
  _frequency = 1.0/period;

  // Publishers
  _pub_wrench = _n.advertise<auv_msgs::BodyForceReq>("/cola2_control/merged_body_force_req", 1);
  _pub_merged_pose = _n.advertise<auv_msgs::WorldWaypointReq>("/cola2_control/merged_world_waypoint_req", 1);
  _pub_merged_twist = _n.advertise<auv_msgs::BodyVelocityReq>("/cola2_control/merged_body_velocity_req", 1);
  _pub_thrusters_setpoint = _n.advertise<cola2_msgs::Setpoints>("/cola2_control/thrusters_data", 1);
  _pub_fins_setpoint = _n.advertise<cola2_msgs::Setpoints>("/cola2_control/fins_data", 1);


  // Subscribers --> WARNING! The buffer should be at least the size of maximum Request send per iteration/kind
  _sub_nav_data = _n.subscribe("/cola2_navigation/nav_sts", 2, &IAUVROSController::updateNav, this);
  _sub_ww_req = _n.subscribe("/cola2_control/world_waypoint_req", 10, &IAUVROSController::updateWWR, this);
  _sub_bv_req = _n.subscribe("/cola2_control/body_velocity_req", 10, &IAUVROSController::updateBVR, this);
  _sub_bf_req = _n.subscribe("/cola2_control/body_force_req", 10, &IAUVROSController::updateBFR, this);

  _are_thrusters_killed = false;

  // Services
  _enable_pose_controller_srv = _n.advertiseService("/cola2_control/enable_pose_controller",
                                                    &IAUVROSController::enablePoseController,
                                                    this);
  _disable_pose_controller_srv = _n.advertiseService("/cola2_control/disable_pose_controller",
                                                     &IAUVROSController::disablePoseController,
                                                     this);
  _enable_velocity_controller_srv = _n.advertiseService("/cola2_control/enable_velocity_controller",
                                                        &IAUVROSController::enableVelocityController,
                                                        this);
  _disable_velocity_controller_srv = _n.advertiseService("/cola2_control/disable_velocity_controller",
                                                         &IAUVROSController::disableVelocityController,
                                                         this);
  _enable_thruster_allocator_srv = _n.advertiseService("/cola2_control/enable_thrusters",
                                                       &IAUVROSController::enableThrusterAllocator,
                                                       this);
  _disable_thruster_allocator_srv = _n.advertiseService("/cola2_control/disable_thrusters",
                                                        &IAUVROSController::disableThrusterAllocator,
                                                        this);
  _enable_fin_allocator_srv = _n.advertiseService("/cola2_control/enable_fins",
                                                  &IAUVROSController::enableFinAllocator,
                                                  this);
  _disable_fin_allocator_srv = _n.advertiseService("/cola2_control/disable_fins",
                                                   &IAUVROSController::disableFinAllocator,
                                                   this);

  // Timers
  _timer = _n.createTimer(ros::Duration(period), &IAUVROSController::timerCallback, this);
  _check_diagnostics = _n.createTimer(ros::Duration(1.0), &IAUVROSController::checkDiagnostics, this);
}

bool IAUVROSController::enablePoseController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Enable pose controller\n";
  _auv_controller->setPoseController(true);
  return true;
}

bool IAUVROSController::disablePoseController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Disable pose controller\n";
  _auv_controller->setPoseController(false);
  return true;
}

bool IAUVROSController::enableVelocityController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Enable velocity controller\n";
  _auv_controller->setVelocityController(true);
  return true;
}

bool IAUVROSController::disableVelocityController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Disable velocity controller\n";
  _auv_controller->setVelocityController(false);
  return true;
}

bool IAUVROSController::enableThrusterAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Enable thruster allocator";
  _auv_controller->setThrusterAllocator(true);
  _are_thrusters_killed = false;

  return true;
}

bool IAUVROSController::disableThrusterAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Disable thruster allocator";

  // Send last setpoint to zero
  Eigen::VectorXd setpoint(_auv_controller->getNumberofThrusters());
  for (unsigned int i = 0; i < _auv_controller->getNumberofThrusters(); i++)
  {
    setpoint(i) = 0.0;
  }
  publishThrusterSetpoint(setpoint, ros::Time::now());

  // Disable thrusters
  _auv_controller->setThrusterAllocator(false);

  return true;
}

bool IAUVROSController::enableFinAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Enable fin allocator";
  _auv_controller->setFinAllocator(true);
  return true;
}

bool IAUVROSController::disableFinAllocator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "[Controller] Disable fin allocator";
  _auv_controller->setFinAllocator(false);
  return true;
}

void IAUVROSController::checkDiagnostics(const ros::TimerEvent& event)
{
  // for diagnostic purposes
  if (fabs(_diagnostic.getCurrentFreq() - _frequency) > 1.0)
  {
    _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::WARN);
    if (_diagnostic.getTimesInWarning() > 10)
    {
      _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::ERROR);
    }
  }
  else
  {
    _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::OK);
  }

  if (!_auv_controller->_is_pose_controller_enable)
  {
    _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::WARN, "pose controller disabled");
  }
  if (!_auv_controller->_is_velocity_controller_enable)
  {
    _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::WARN, "velocity controller disabled");
  }
  if (!_auv_controller->_is_thruster_allocator_enable)
  {
    _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::WARN, "thruster allocator disabled");
  }
}

void IAUVROSController::timerCallback(const ros::TimerEvent& event)
{
  // Get current time
  ros::Time now = ros::Time::now();

  // Iterate controller
  _auv_controller->iteration(now.toSec());

  // Compute thruster setpoints
  _auv_controller->computeThrusterAllocator();

  // Check period for diagnostics
  _diagnostic.check_frequency();

  // Publish data
  publishMergedPose(_auv_controller->getMergedPose(), now);
  publishMergedTwist(_auv_controller->getMergedTwist(), now);
  publishMergedWrench(_auv_controller->getMergedWrench(), now);

  // Publish thruster setpoint if enabled
  if (_auv_controller->isThrusterAllocatorEnable())
  {
    Eigen::VectorXd setpoint = _auv_controller->getThrusterSetpoints();
    publishThrusterSetpoint(setpoint, now);
  }

  // Publish fins setpoint. If disabled the setpoint is set to an appropiate angle
  publishFinSetpoint(_auv_controller->getFinSetpoints(), now);
}

void IAUVROSController::updateNav(const ros::MessageEvent<auv_msgs::NavSts const> & msg)
{
  // Update pose feedback
  std::vector< double > pose_feedback;
  pose_feedback.push_back(msg.getMessage()->position.north);
  pose_feedback.push_back(msg.getMessage()->position.east);
  pose_feedback.push_back(msg.getMessage()->position.depth);
  pose_feedback.push_back(msg.getMessage()->orientation.roll);
  pose_feedback.push_back(msg.getMessage()->orientation.pitch);
  pose_feedback.push_back(msg.getMessage()->orientation.yaw);
  _auv_controller->updatePoseFeedback(pose_feedback);

  // Update twist feedback
  std::vector< double > twist_feedback;
  twist_feedback.push_back(msg.getMessage()->body_velocity.x);
  twist_feedback.push_back(msg.getMessage()->body_velocity.y);
  twist_feedback.push_back(msg.getMessage()->body_velocity.z);
  twist_feedback.push_back(msg.getMessage()->orientation_rate.roll);
  twist_feedback.push_back(msg.getMessage()->orientation_rate.pitch);
  twist_feedback.push_back(msg.getMessage()->orientation_rate.yaw);
  _auv_controller->updateTwistFeedback(twist_feedback);

  // Stores last altitude. If altitude is invalid, during 5 seconds estimate it wrt last altitude and delta depth.
  // If more than 5 seconds put it a 0.5.
  if (msg.getMessage()->altitude > 0.0)
  {
    _last_altitude = msg.getMessage()->altitude;
    _last_altitude_age = msg.getMessage()->header.stamp.toSec();
    _last_depth = msg.getMessage()->position.depth;
  }
  else
  {
    if ((ros::Time::now().toSec() - _last_altitude_age)  > 5.0)
    {
      _last_altitude = 0.5;
    }
    else
    {
      _last_altitude = _last_altitude - (msg.getMessage()->position.depth - _last_depth);
      _last_depth = msg.getMessage()->position.depth;
    }
  }
}

void IAUVROSController::updateWWR(const ros::MessageEvent<auv_msgs::WorldWaypointReq const> & msg)
{
  // Init request
  Request req(msg.getMessage()->goal.requester,
              msg.getMessage()->header.stamp.toSec(),
              msg.getMessage()->goal.priority,
              6);

  // Set disable axis
  std::vector< bool > disabled_axis;
  disabled_axis.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis.push_back(msg.getMessage()->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector<double> values;
  values.push_back(msg.getMessage()->position.north);
  values.push_back(msg.getMessage()->position.east);

  // If desired Z is in altitude transform it into depth
  if (msg.getMessage()->altitude_mode)
  {
    double altitude_to_depth = (_last_depth + _last_altitude) - msg.getMessage()->altitude;
    if (altitude_to_depth < 0.0) altitude_to_depth = 0.0;
    values.push_back(altitude_to_depth);
  }
  else
  {
    values.push_back(msg.getMessage()->position.depth);
  }

  values.push_back(msg.getMessage()->orientation.roll);
  values.push_back(msg.getMessage()->orientation.pitch);
  values.push_back(msg.getMessage()->orientation.yaw);
  req.setValues(values);

  // Add request to controller ptr.
  _auv_controller->updatePoseRequest(req);
}

void IAUVROSController::updateBVR(const ros::MessageEvent<auv_msgs::BodyVelocityReq const> & msg)
{
  // Init request
  Request req(msg.getMessage()->goal.requester,
              msg.getMessage()->header.stamp.toSec(),
              msg.getMessage()->goal.priority,
              6);

  // Set disable axis
  std::vector< bool > disabled_axis;
  disabled_axis.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis.push_back(msg.getMessage()->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector< double > values;
  values.push_back(msg.getMessage()->twist.linear.x);
  values.push_back(msg.getMessage()->twist.linear.y);
  values.push_back(msg.getMessage()->twist.linear.z);
  values.push_back(msg.getMessage()->twist.angular.x);
  values.push_back(msg.getMessage()->twist.angular.y);
  values.push_back(msg.getMessage()->twist.angular.z);
  req.setValues(values);

  // Add request to controller ptr.
  _auv_controller->updateTwistRequest(req);
}

void IAUVROSController::updateBFR(const ros::MessageEvent<auv_msgs::BodyForceReq const> & msg)
{
  // Init request
  Request req(msg.getMessage()->goal.requester,
              msg.getMessage()->header.stamp.toSec(),
              msg.getMessage()->goal.priority,
              6);

  // Set disable axis
  std::vector< bool > disabled_axis;
  disabled_axis.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis.push_back(msg.getMessage()->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector< double > values;
  values.push_back(msg.getMessage()->wrench.force.x);
  values.push_back(msg.getMessage()->wrench.force.y);
  values.push_back(msg.getMessage()->wrench.force.z);
  values.push_back(msg.getMessage()->wrench.torque.x);
  values.push_back(msg.getMessage()->wrench.torque.y);
  values.push_back(msg.getMessage()->wrench.torque.z);
  req.setValues(values);

  // Add request to controller ptr.
  _auv_controller->updateWrenchRequest(req);
}

void IAUVROSController::publishThrusterSetpoint(const Eigen::VectorXd setpoint, const ros::Time now)
{
  // Cretae ROS thruster setpoint msg
  cola2_msgs::Setpoints output;

  // Fill header
  output.header.frame_id = _frame_id;
  output.header.stamp = now;

  for (unsigned int i = 0; i < setpoint.size(); i++)
  {
    output.setpoints.push_back(setpoint[i]);
  }

  // Publish message
  _pub_thrusters_setpoint.publish(output);
}

void IAUVROSController::publishFinSetpoint(const Eigen::VectorXd setpoint, const ros::Time now)
{
  // Cretae ROS fin setpoint msg
  cola2_msgs::Setpoints output;

  // Fill header
  output.header.frame_id = _frame_id;
  output.header.stamp = now;

  for (unsigned int i = 0; i < setpoint.size(); i++)
  {
    output.setpoints.push_back(setpoint[i]);
  }

  // Publish message
  _pub_fins_setpoint.publish(output);
}

void IAUVROSController::publishMergedPose(const Request pose, const ros::Time now)
{
  // Create ROS output
  auv_msgs::WorldWaypointReq output;

  // Fill header
  output.header.frame_id = _frame_id;
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = pose.getPriority();
  output.goal.requester = pose.getRequester();

  // Fill disable axis
  std::vector< bool > disable_axis = pose.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector< double > values = pose.getValues();
  assert(values.size() == 6);
  output.position.north = values.at(0);
  output.position.east = values.at(1);
  output.position.depth = values.at(2);
  output.orientation.roll = values.at(3);
  output.orientation.pitch = values.at(4);
  output.orientation.yaw = values.at(5);

  // Publish output
  _pub_merged_pose.publish(output);
}

void IAUVROSController::publishMergedTwist(const Request twist, const ros::Time now)
{
  // Create ROS output
  auv_msgs::BodyVelocityReq output;

  // Fill header
  output.header.frame_id = _frame_id;
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = twist.getPriority();
  output.goal.requester = twist.getRequester();

  // Fill disable axis
  std::vector< bool > disable_axis = twist.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector< double > values = twist.getValues();
  assert(values.size() == 6);
  output.twist.linear.x = values.at(0);
  output.twist.linear.y = values.at(1);
  output.twist.linear.z = values.at(2);
  output.twist.angular.x = values.at(3);
  output.twist.angular.y = values.at(4);
  output.twist.angular.z = values.at(5);

  // Publish output
  _pub_merged_twist.publish(output);
}

void IAUVROSController::publishMergedWrench(const Request response, const ros::Time now)
{
  // Create ROS output
  auv_msgs::BodyForceReq output;

  // Fill header
  output.header.frame_id = _frame_id;
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = response.getPriority();
  output.goal.requester = response.getRequester();

  // Fill disable axis
  std::vector<bool> disable_axis = response.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector<double> values = response.getValues();
  assert(values.size() == 6);
  output.wrench.force.x = values.at(0);
  output.wrench.force.y = values.at(1);
  output.wrench.force.z = values.at(2);
  output.wrench.torque.x = values.at(3);
  output.wrench.torque.y = values.at(4);
  output.wrench.torque.z = values.at(5);

  // Publish output
  _pub_wrench.publish(output);
}
