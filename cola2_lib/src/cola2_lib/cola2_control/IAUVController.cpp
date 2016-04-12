#include "cola2_lib/cola2_control/IAUVController.h"

IAUVController::IAUVController(double period, int n_dof, int n_thrusters, int n_fins):
  _is_pose_controller_enable(true),
  _is_velocity_controller_enable(true),
  _is_thruster_allocator_enable(true),
  _is_fin_allocator_enable(true),
  _n_dof(n_dof),
  _pose_merge("pose_merge", "pose", period * 3),
  _pose_feedback(n_dof, 0.0),
  _max_velocity(n_dof, 0.2),
  _twist_merge("twist_merge", "twist", period * 3),
  _twist_feedback(n_dof, 0.0),
  _max_wrench(n_dof, 100.0),
  _wrench_merge("wrench_merge", "wrench", period * 3),
  _thruster_setpoints(n_thrusters),
  _fin_setpoints(n_fins)
{
}

void IAUVController::updatePoseRequest(Request req)
{
  _pose_merge.addRequest(req);
}

void IAUVController::updatePoseFeedback(std::vector< double > feedback)
{
  _pose_feedback = feedback;
}

void IAUVController::updateTwistRequest(Request req)
{
  _twist_merge.addRequest(req);
}

void IAUVController::updateTwistFeedback(std::vector< double > feedback)
{
  _twist_feedback = feedback;
}

void IAUVController::updateWrenchRequest(Request req)
{
  _wrench_merge.addRequest(req);
}

void IAUVController::addPolyParamToVector(std::vector< double > values,
                                          std::vector< std::map< std::string, double > >& params_vector)
{
  assert(values.size() == 3);
  std::map< std::string, double > param;
  param.insert(std::pair< std::string, double > ("n_dof", 3.0));
  param.insert(std::pair< std::string, double > ("0", values.at(0)));
  param.insert(std::pair< std::string, double > ("1", values.at(1)));
  param.insert(std::pair< std::string, double > ("2", values.at(2)));
  params_vector.push_back(param);
}

void IAUVController::addPIDParamToVector(std::vector< std::string > keys, std::vector< double > values,
                                         std::vector< std::map< std::string, double > >& params_vector)
{
  assert(keys.size() == values.size());
  std::map< std::string, double > param;
  for (unsigned int i = 0; i < keys.size(); i++)
  {
    param.insert(std::pair< std::string, double > (keys.at(i), values.at(i)));
  }
  // Add derivative_term_from_feedback
  param.insert(std::pair< std::string, double > ("derivative_term_from_feedback", 1.0));
  params_vector.push_back(param);
}

void IAUVController::setMaxWrench(std::vector<double> max_wrench)
{
  assert(max_wrench.size() == _max_wrench.size());
  std::copy(max_wrench.begin(), max_wrench.end(), _max_wrench.begin());
}

void IAUVController::setMaxVelocity(std::vector<double> max_velocity)
{
  assert(max_velocity.size() == _max_velocity.size());
  std::copy(max_velocity.begin(), max_velocity.end(), _max_velocity.begin());
}

Request IAUVController::getMergedPose() const
{
  return _merged_pose;
}

Request IAUVController::getMergedTwist() const
{
  return _merged_twist;
}

Request IAUVController::getMergedWrench() const
{
  return _merged_wrench;
}

void IAUVController::setMergedWrench(const Request wrench)
{
  _merged_wrench = wrench;
}

Eigen::VectorXd IAUVController::getThrusterSetpoints() const
{
  return _thruster_setpoints;
}

Eigen::VectorXd IAUVController::getFinSetpoints() const
{
  return _fin_setpoints;
}

void IAUVController::setPoseController(const bool is_enabled)
{
  _is_pose_controller_enable = is_enabled;
}

void IAUVController::setVelocityController(const bool is_enabled)
{
  _is_velocity_controller_enable = is_enabled;
}

void IAUVController::setThrusterAllocator(const bool is_enabled)
{
  _is_thruster_allocator_enable = is_enabled;
}

bool IAUVController::isThrusterAllocatorEnable() const
{
  return _is_thruster_allocator_enable;
}

void IAUVController::setFinAllocator(const bool is_enabled)
{
  _is_fin_allocator_enable = is_enabled;
}

bool IAUVController::isFinAllocatorEnable() const
{
  return _is_fin_allocator_enable;
}
