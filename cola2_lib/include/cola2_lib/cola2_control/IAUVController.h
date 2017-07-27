
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVCONTROLLER_H_

#include <Eigen/Dense>

#include <algorithm>
#include <map>
#include <utility>
#include <string>
#include <vector>

#include "./Merge.h"
#include "./Request.h"

class IAUVController
{
 public:
  IAUVController(double period, int n_dof, int n_thrusters, int n_fins = 0);

  void updatePoseRequest(Request req);
  void updatePoseFeedback(std::vector< double > feedback);
  void updateTwistRequest(Request req);
  void updateTwistFeedback(std::vector< double > feedback);
  void updateWrenchRequest(Request req);

  void addPolyParamToVector(std::vector< double > values,
                             std::vector< std::map< std::string, double > >& params_vector);
  void addPIDParamToVector(std::vector< std::string > keys, std::vector< double > values,
                            std::vector< std::map< std::string, double > >& params_vector);

  void setMaxWrench(std::vector<double> max_wrench);
  void setMaxVelocity(std::vector<double> max_velocity);

  Request getMergedPose() const;
  Request getMergedTwist() const;
  Request getMergedWrench() const;

  void setMergedWrench(const Request wrench);

  Eigen::VectorXd getThrusterSetpoints() const;
  Eigen::VectorXd getFinSetpoints() const;

  virtual void iteration(double current_time) = 0;
  virtual void reset() = 0;
  virtual void computeThrusterAllocator() = 0;
  virtual unsigned int getNumberofThrusters() const = 0;

  void setPoseController(const bool is_enabled);
  void setVelocityController(const bool is_enabled);
  void setThrusterAllocator(const bool is_enabled);
  bool getThrusterAllocator() const;

  void setFinAllocator(const bool is_enabled);

  bool isThrusterAllocatorEnable() const;
  bool isFinAllocatorEnable() const;

  // Are controllers enabled
  bool _is_pose_controller_enable;
  bool _is_velocity_controller_enable;
  bool _is_thruster_allocator_enable;
  bool _is_fin_allocator_enable;

 protected:
  // DoF
  unsigned int _n_dof;

  // Pose
  Merge _pose_merge;
  std::vector< double > _pose_feedback;
  std::vector< double > _max_velocity;

  // Twist
  Merge _twist_merge;
  std::vector< double > _twist_feedback;
  std::vector< double > _max_wrench;

  // Wrench
  Merge _wrench_merge;

  // Intermediate requests
  Request _merged_pose;
  Request _merged_twist;
  Request _merged_wrench;
  Eigen::VectorXd _thruster_setpoints;
  Eigen::VectorXd _fin_setpoints;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVCONTROLLER_H_
