#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFSLAMAUV_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFSLAMAUV_H_

// Don't panic!!! This file is for the COLA2 project.
// However, as all the included files that it is going to need
// are currently developed in this other project, I've started
// to work with it here.

#include <iostream>
#include <map>
#include <utility>
#include <string>
#include <vector>

#include "./EkfBase.h"
#include "./transformations.h"
#include "./nav_utils.h"

class EkfSlamAuv: public EkfBase
{
 public:
  EkfSlamAuv(const unsigned int state_vector_size, const Eigen::VectorXd q_var);

  void setImuInput(const std::string sensor_id,
                   const double time_stamp,
                   const Eigen::Quaterniond orientation,
                   const Eigen::Matrix3d orientation_cov,
                   const Eigen::Vector3d angular_velocity,
                   const Eigen::Matrix3d angular_velocity_cov,
                   const double declination = 0.0);

  unsigned int getNumberOfLandmarks() const;

  std::string getLandmarkId(const unsigned int id);

  int getLandmarkPosition(const std::string& id);


  double getLandmarkLastUpdate(unsigned int landmark_id);

  void getImuData(Eigen::Quaterniond& orientation,
                  Eigen::Matrix3d& orientation_cov,
                  Eigen::Vector3d& angular_velocity,
                  Eigen::Matrix3d& angular_velocity_cov);


  void positionUpdate(const std::string sensor_id,
                      const double time_stamp,
                      const Eigen::Vector3d position,
                      const Eigen::Matrix3d position_cov);

  void velocityUpdate(const std::string sensor_id,
                      const double time_stamp,
                      const Eigen::Vector3d velocity,
                      const Eigen::Matrix3d velocity_cov);

  unsigned int landmarkUpdate(const std::string& sensor_id,
                              const std::string& landmark_id,
                              const double& time_stamp,
                              const Eigen::Vector3d& landmark_measured_position,
                              const Eigen::Quaterniond& landmark_measured_orientation,
                              const Eigen::MatrixXd& landmark_cov);

  unsigned int rangeUpdate(const std::string& landmark_id,
                           const double& time_stamp,
                           const double& landmark_measured_range,
                           const Eigen::MatrixXd& range_cov);

  void addLandmark(const Eigen::VectorXd& landmark,
                   const Eigen::MatrixXd& landmark_cov,
                   const std::string landmark_id);

  void createLandmarkMeasure(const Eigen::Vector3d landmark_position,
                             const Eigen::Vector3d landmark_orientation_RPY,
                             const unsigned int landmark_id,
                             const Eigen::Matrix3d rot,
                             const Eigen::MatrixXd covariance,
                             Eigen::VectorXd& z,
                             Eigen::MatrixXd& r,
                             Eigen::MatrixXd& h,
                             Eigen::MatrixXd& v);

  void createVelocityMeasure(const Eigen::Vector3d velocity,
                             const Eigen::Matrix3d velocity_cov,
                             Eigen::VectorXd& z,
                             Eigen::MatrixXd& r,
                             Eigen::MatrixXd& h,
                             Eigen::MatrixXd& v);

  void createPositionMeasure(const Eigen::Vector3d position,
                             const Eigen::Matrix3d position_cov,
                             Eigen::VectorXd& z,
                             Eigen::MatrixXd& r,
                             Eigen::MatrixXd& h,
                             Eigen::MatrixXd& v);

  // Implement virtual methods defined in EkfBase class
  void normalizeInnovation(Eigen::VectorXd& innovation) const;
  void normalizeState();
  Eigen::VectorXd computeU() const;
  Eigen::MatrixXd computeA(const double t, const Eigen::VectorXd u) const;
  Eigen::MatrixXd computeW(const double t, const Eigen::VectorXd u) const;
  Eigen::VectorXd f(const Eigen::VectorXd x_1, const double t, const Eigen::VectorXd u) const;

  Eigen::Quaterniond _auv_orientation;
  Eigen::Matrix3d _auv_orientation_cov;
  Eigen::Vector3d _auv_angular_velocity;
  Eigen::Matrix3d _auv_angular_velocity_cov;
  bool _is_imu_init;
  unsigned int _number_of_landmarks;
  std::map< unsigned int, double > _landmark_last_update;
  std::map< std::string, unsigned int > _mapped_lamdmarks;
  std::map< unsigned int, std::string > _id_to_mapped_lamdmark;
  std::map< std::string, std::vector< Eigen::VectorXd > > _candidate_landmarks;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFSLAMAUV_H_
