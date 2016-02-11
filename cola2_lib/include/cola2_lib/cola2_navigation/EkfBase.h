#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFBASE_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFBASE_H_

// Don't panic!!! This file is for the COLA2 project.
// However, as all the included files that it is going to need
// are currently developed in this other project, I've started
// to work with it here.

#include <Eigen/Dense>

#include <iostream>
#include <map>
#include <utility>
#include <string>
#include <vector>

#define MAX_COV 0.0025

class EkfBase
{
 public:
  EkfBase(const unsigned int state_vector_size, const Eigen::VectorXd q_var);

  void initEkf(const Eigen::VectorXd state, const Eigen::VectorXd p_var);
  bool makePrediction(const double now, Eigen::VectorXd u);
  bool applyUpdate(const Eigen::VectorXd z,
                   const Eigen::MatrixXd r,
                   const Eigen::MatrixXd h,
                   const Eigen::MatrixXd v,
                   const double mahalanobis_distance_threshold);

  void showStateVector();
  Eigen::VectorXd getStateVector();
  Eigen::MatrixXd getCovarianceMatrix();

  void setLastPredictionTime(const double time);
  void setTransformation(const std::string sensor_id,
                         const Eigen::Quaterniond rotation_from_origin,
                         const Eigen::Vector3d translation_from_origin);

 protected:
  // *****************************************
  //          Utility functions
  // *****************************************
  double mahalanobisDistance(const Eigen::VectorXd z,
                             const Eigen::MatrixXd r,
                             const Eigen::MatrixXd h);

  void checkIntegrity();
  bool isValidCandidate(const std::vector< Eigen::VectorXd >& candidates);
  void updatePrediction();

  unsigned int _state_vector_size;
  double _last_prediction;
  Eigen::MatrixXd _Q;
  bool _is_ekf_init;
  Eigen::MatrixXd _x;
  Eigen::MatrixXd _P;
  Eigen::MatrixXd _x_;
  Eigen::MatrixXd _P_;
  std::map< std::string, std::pair< Eigen::Quaterniond, Eigen::Vector3d  > > _transformations;
  unsigned int _filter_updates;

 private:
  // *****************************************
  //            Methods that must be
  //      implemented in derived calsses
  // *****************************************
  virtual void normalizeInnovation(Eigen::VectorXd& innovation) const = 0;
  virtual void normalizeState() = 0;
  virtual Eigen::MatrixXd computeA(const double t, const Eigen::VectorXd u) const = 0;
  virtual Eigen::VectorXd computeU() const = 0;
  virtual Eigen::MatrixXd computeW(const double t, const Eigen::VectorXd u) const = 0;
  virtual Eigen::VectorXd f(const Eigen::VectorXd x_1, const double t, const Eigen::VectorXd u) const = 0;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFBASE_H_
