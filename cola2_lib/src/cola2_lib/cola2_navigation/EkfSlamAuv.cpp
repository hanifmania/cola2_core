
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/cola2_navigation/EkfSlamAuv.h"

EkfSlamAuv::EkfSlamAuv(const unsigned int state_vector_size, const Eigen::VectorXd q_var):
  EkfBase(state_vector_size, q_var),
  _is_imu_init(false),
  _number_of_landmarks(0),
  _auv_orientation(1.0, 0.0, 0.0, 0.0)
{
  _auv_orientation_cov      = Eigen::Matrix3d::Zero();  // TODO: initializer list? How?
  _auv_angular_velocity     = Eigen::Vector3d::Zero();
  _auv_angular_velocity_cov = Eigen::Matrix3d::Zero();
}

void EkfSlamAuv::setImuInput(const std::string sensor_id,
                             const double time_stamp,
                             const Eigen::Quaterniond orientation,
                             const Eigen::Matrix3d orientation_cov,
                             const Eigen::Vector3d angular_velocity,
                             const Eigen::Matrix3d angular_velocity_cov,
                             const double declination)
{
  // ---- TODO: Apply declination. Check if it is correct! ----------
  // std::cout << "orientation: " << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ", " << orientation.w() << "\n";
  Eigen::Vector3d angle = getRPY(orientation.toRotationMatrix());
  // std::cout << "Yaw: " << angle[0] << ", declination: " << rad2Deg( declination ) << "\n";
  angle[2] = normalizeAngle(angle[2] - declination);
  // std::cout << "Yaw with declination: " << angle[2] << "\n";
  Eigen::Quaterniond orientation_dev = euler2Quaternion(angle[0], angle[1], angle[2]);
  // std::cout << "orientation with declination: " << orientation_dev.x() << ", " << orientation_dev.y() << ", " << orientation_dev.z() << ", " << orientation_dev.w() << "\n";
  // ------------------------------------------------------------------

  // TODO: WARNING! COVARIANCE INFORMATION IS NOT ROTATED AS THE TF INDICATES!!!
  if (_transformations.find(sensor_id) != _transformations.end())
  {
    // Save imu data
    _auv_orientation = transformations::orientation(orientation_dev, _transformations[sensor_id].first);
    _auv_orientation_cov = orientation_cov;
    _auv_angular_velocity =  transformations::angularVelocity(angular_velocity, _transformations[sensor_id].first);
    _auv_angular_velocity_cov = angular_velocity_cov;
    _is_imu_init = true;
  }
  else
  {
    std::cout << "EkfSlamAuv warning: IMU data stored without appropriate TF\n";
    // Save imu data
    _auv_orientation = orientation_dev;
    _auv_orientation_cov = orientation_cov;
    _auv_angular_velocity =  angular_velocity;
    _auv_angular_velocity_cov = angular_velocity_cov;
    _is_imu_init = true;
  }

  if (makePrediction(time_stamp, computeU()))
  {
    updatePrediction();
    // showStateVector();
  }
}

unsigned int EkfSlamAuv::getNumberOfLandmarks() const
{
  return _number_of_landmarks;
}

std::string EkfSlamAuv::getLandmarkId(const unsigned int id)
{
  return _id_to_mapped_lamdmark[ id ];
}

int EkfSlamAuv::getLandmarkPosition(const std::string& id)
{
    if (_mapped_lamdmarks.find(id) != _mapped_lamdmarks.end()) {
        return _mapped_lamdmarks[id];
    }
    return -1;
}

double EkfSlamAuv::getLandmarkLastUpdate(unsigned int landmark_id)
{
  return _landmark_last_update[landmark_id];
}

void EkfSlamAuv::getImuData(Eigen::Quaterniond& orientation,
                            Eigen::Matrix3d& orientation_cov,
                            Eigen::Vector3d& angular_velocity,
                            Eigen::Matrix3d& angular_velocity_cov)
{
  orientation = _auv_orientation;
  orientation_cov = _auv_orientation_cov;
  angular_velocity = _auv_angular_velocity;
  angular_velocity_cov = _auv_angular_velocity_cov;
}


void EkfSlamAuv::positionUpdate(const std::string sensor_id,
                                const double time_stamp,
                                const Eigen::Vector3d position,
                                const Eigen::Matrix3d position_cov)
{
  // std::cout << "position update\n";
  if (_is_imu_init && makePrediction( time_stamp, computeU()))
  {
    Eigen::Vector3d position_tmp;
    Eigen::Matrix3d position_cov_tmp;

    if (_transformations.find(sensor_id) != _transformations.end())
    {
      // Transform position to vehicle frame
      position_tmp = transformations::position(position,
                                               _auv_orientation,
                                               _transformations[sensor_id].second);
      // std::cout << "position_tmp:\n" << position_tmp << "\n";

      // Transform covariance to vehicle frame
      position_cov_tmp = transformations::positionCovariance(position_cov,
                                                             _auv_orientation_cov,
                                                             _auv_orientation,
                                                             _transformations[sensor_id].second);
      // std::cout << "position_cov_tmp:\n" << position_cov_tmp << "\n";
    }
    else
    {
      position_tmp = position;
      position_cov_tmp = position_cov;
    }

    // Create position measure matrices
    Eigen::VectorXd z;
    Eigen::MatrixXd r, h, v;
    createPositionMeasure(position_tmp, position_cov_tmp,
                          z, r, h, v);

    // std::cout << "z:\n" << z << "\n";
    // std::cout << "r:\n" << r << "\n";
    // std::cout << "h:\n" << h << "\n";
    // std::cout << "v:\n" << v << "\n";

    // Apply update
    if (!applyUpdate( z, r, h, v, 30.0))
    {
      // If the update fails update at leat the prediction
      std::cout << "Position " << sensor_id << " Update fails!\n";
      updatePrediction();
    }
  }
  else
  {
    std::cout << "Make prediction " << sensor_id << " fail, update not applied!\n";
  }
}

void EkfSlamAuv::velocityUpdate(const std::string sensor_id,
                                const double time_stamp,
                                const Eigen::Vector3d velocity,
                                const Eigen::Matrix3d velocity_cov)
{
  Eigen::VectorXd u = computeU();
  if (_is_imu_init && makePrediction( time_stamp, u))
  {
    Eigen::Vector3d velocity_tmp;
    Eigen::Matrix3d velocity_cov_tmp;

    if (_transformations.find(sensor_id) != _transformations.end())
    {
      // Transform position to vehicle frame
      velocity_tmp = transformations::linearVelocity(velocity,
                                                     _auv_angular_velocity,
                                                     _transformations[ sensor_id ].first,
                                                     _transformations[ sensor_id ].second);

      // Transform covariance to vehicle frame
      velocity_cov_tmp = transformations::linearVelocityCov(velocity_cov,
                                                            _auv_angular_velocity_cov,
                                                            _transformations[sensor_id].first,
                                                            _transformations[sensor_id].second);
    }
    else
    {
      velocity_tmp = velocity;
      velocity_cov_tmp = velocity_cov;
    }

    // Create velocity measure matrices
    Eigen::VectorXd z;
    Eigen::MatrixXd r, h, v;
    createVelocityMeasure(velocity_tmp, velocity_cov_tmp,
                          z, r, h, v);

    // std::cout << "z:\n" << z << "\n";
    // std::cout << "r:\n" << r << "\n";
    // std::cout << "h:\n" << h << "\n";
    // std::cout << "v:\n" << v << "\n";

    // Apply update
    if (!applyUpdate(z, r, h, v, 16.0))
    {
      // If the update fails update at leat the prediction
      updatePrediction();
    }
  }
}

unsigned int EkfSlamAuv::landmarkUpdate(const std::string& sensor_id,
                                        const std::string& landmark_id,
                                        const double& time_stamp,
                                        const Eigen::Vector3d& landmark_measured_position,
                                        const Eigen::Quaterniond& landmark_measured_orientation,
                                        const Eigen::MatrixXd& landmark_cov)
{
  if (_is_imu_init && makePrediction( time_stamp, computeU()))
  {
    // std::cout << "landmark: " << landmark_id << std::endl;
    Eigen::Vector3d landmark_position_tmp;
    Eigen::Quaterniond landmark_orientation_tmp;
    Eigen::MatrixXd landmark_cov_tmp;

    if (_transformations.find(sensor_id) != _transformations.end())
    {
      // std::cout << "Transform landmark from sensor frame to vehicle frame.\n";
      // Transform position to vehicle frame
      landmark_position_tmp = transformations::landmarkPosition(landmark_measured_position,
                                                                _transformations[sensor_id].first,
                                                                _transformations[sensor_id].second);

      // Transform orientation to vehicle frame
      landmark_orientation_tmp = transformations::landmarkOrientation(landmark_measured_orientation,
                                                                      _transformations[sensor_id].first);

      // TODO: Transform covariance to vehicle frame not properly done!!!
      Eigen::Matrix3d tmp;
      tmp = landmark_cov.block(0, 0, 3, 3);
      tmp = transformations::rotatedCovariance(tmp, _transformations[sensor_id].first);
      landmark_cov_tmp = landmark_cov;
      landmark_cov_tmp.block(0, 0, 3, 3) = tmp;
    }
    else
    {
      // std::cout << "Landmark already in vehicle frame.\n";
      landmark_position_tmp = landmark_measured_position;
      landmark_orientation_tmp = landmark_measured_orientation;
      landmark_cov_tmp = landmark_cov;
    }

    // Compose landmark orienation with vehicle orientation to obtain
    // the landmark orientation in the inertial frame (world frame)
    // std::cout << "Compute Angle (RPY) in I frame...\n";
    Eigen::Vector3d angle = getRPY(_auv_orientation.toRotationMatrix() * landmark_orientation_tmp.toRotationMatrix());
    // std::cout << "Angle (YPR) in I frame:\n" << angle << "\n";

    // If the landmark has not been mapped
    if (_mapped_lamdmarks.find(landmark_id) == _mapped_lamdmarks.end())
    {
      // std::cout << "Landmark " << landmark_id << " not already mapped.\n";
      // Create a candidate
      // ...transform landmark position from vehicle frame to inertial frame (world)
      landmark_position_tmp = transformations::landmarkPosition(landmark_position_tmp ,
                                                                _auv_orientation,
                                                                Eigen::Vector3d(_x(0), _x(1), _x(2)));
      Eigen::VectorXd candidate = Eigen::MatrixXd::Zero(6, 1);
      candidate << landmark_position_tmp(0), landmark_position_tmp(1), landmark_position_tmp(2), angle(0), angle(1), angle(2);
      std::cout << "candidate (xyz rpy):\n" << candidate << "\n";
      // If candidate has been seen before
      if (_candidate_landmarks.find(landmark_id) != _candidate_landmarks.end())
      {
        std::cout << "previously seen candidate.\n";
        // Add candidate
        _candidate_landmarks[landmark_id].push_back(candidate);

        // Check candidate
        if (isValidCandidate(_candidate_landmarks[landmark_id]))
        {
          std::cout << "validated candidate.\n";
          // _mapped_lamdmarks[landmark_id] = _number_of_landmarks;
          // _id_to_mapped_lamdmark[_number_of_landmarks] = landmark_id;
          _landmark_last_update[_number_of_landmarks] = time_stamp;
          addLandmark(candidate, landmark_cov_tmp, landmark_id);
          return 0;  // just validated
        }
        else
        {
          return 1;  // not validated yet
        }
      }
      else
      {
        // Add first candidate
        std::cout << "First time that the candidate is seen\n";
        std::vector< Eigen::VectorXd > tmp;
        tmp.push_back(candidate);
        _candidate_landmarks.insert(std::pair< std::string, std::vector<Eigen::VectorXd> >(landmark_id, tmp));
        return 2;  // first sight
      }
    }

    // If the landmark is already mapped ...
    // ... create landmark matrices to apply the update
    Eigen::VectorXd z;
    Eigen::MatrixXd r, h, v;
    createLandmarkMeasure(landmark_position_tmp,
                          angle,
                          _mapped_lamdmarks[landmark_id],
                          _auv_orientation.toRotationMatrix().transpose(),
                          landmark_cov_tmp,
                          z, r, h, v);

    // std::cout << "z:\n" << z << "\n";
    // std::cout << "r:\n" << r << "\n";
    // std::cout << "h:\n" << h << "\n";
    // std::cout << "v:\n" << v << "\n";

    // Save last update time for this landmark
    _landmark_last_update[_mapped_lamdmarks[landmark_id]] = time_stamp;

    // Apply update
    if (!applyUpdate( z, r, h, v, 25.0))
    {
      // If the update fails update at leat the prediction
      updatePrediction();
      return 4;  // prediction only
    }
    return 3;  // update

  }
  return 5;  // no imu / no prediction
}




unsigned int EkfSlamAuv::rangeUpdate(const std::string& landmark_id,
                                     const double& time_stamp,
                                     const double& landmark_measured_range,
                                     const Eigen::MatrixXd& range_cov)
{
  // If the landmark is already mapped do a prediction.
  if (_mapped_lamdmarks.find(landmark_id) != _mapped_lamdmarks.end() &&
      makePrediction(time_stamp, computeU())) {

    // If the landmark is already mapped ...
    unsigned int landmark_number = _mapped_lamdmarks[landmark_id];
    std::cout << "landmark: " << landmark_id << " (" << landmark_number << "), range: " << landmark_measured_range << std::endl;

    // ... create landmark matrices to apply the update
    Eigen::VectorXd z;
    Eigen::MatrixXd v;

    Eigen::Vector3d diff, auv_position, landmark_position;
    Eigen::VectorXd innovation = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd h = Eigen::VectorXd::Zero(1);

    auv_position(0) = _x_(0, 0);
    auv_position(1) = _x_(1, 0);
    auv_position(2) = _x_(2, 0);
    landmark_position(0) = _x_(6 + 6*landmark_number);
    landmark_position(1) = _x_(6 + 6*landmark_number + 1);
    landmark_position(2) = _x_(6 + 6*landmark_number + 2);
    diff = landmark_position - auv_position;
    h(0) = diff.norm();
    innovation(0) = landmark_measured_range - h(0);

    // Linearization of h() into H
    double inv = 1.0 / h(0);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6 + 6*getNumberOfLandmarks());
    H(0, 0) = -diff(0) * inv;
    H(0, 1) = -diff(1) * inv;
    H(0, 2) = -diff(2) * inv;
    H(0, 6 + 6*landmark_number) = -H(0, 0);
    H(0, 6 + 6*landmark_number + 1) = -H(0, 1);
    H(0, 6 + 6*landmark_number + 2) = -H(0, 2);

    // std::cout << "_x_: " << _x_ << std::endl;
    // std::cout << "auv: " << auv_position << std::endl;
    // std::cout << "landmark: " << landmark_position << std::endl;
    // std::cout << "diff: " << diff << std::endl;
    // std::cout << "innovation: " << innovation << std::endl;
    // std::cout << "H: " << H << std::endl;

    // Save last update time for this landmark
    _landmark_last_update[_mapped_lamdmarks[landmark_id]] = time_stamp;

    // Apply update
    if (!applyNonLinearUpdate(innovation,
                              range_cov,
                              H,
                              Eigen::MatrixXd::Identity(1, 1),
                              25.0)) {
      // If the update fails update at least the prediction
      updatePrediction();
      return 4;  // prediction only
    }
    return 3;  // update
  }
  return 1;  // no validated landmark
}

void EkfSlamAuv::resetLandmarks() {
    // Remove all landmarks from the filter
    _mapped_lamdmarks.clear();
    _id_to_mapped_lamdmark.clear();
    _number_of_landmarks = 0;
    Eigen::MatrixXd new_P;
    new_P = Eigen::MatrixXd::Identity(6, 6);
    new_P = _P.block(0, 0, 6, 6);
    _P = new_P;
    std::cout << "New P:\n" << new_P << "\n\n";
    Eigen::MatrixXd new_x;
    new_x = Eigen::MatrixXd::Zero(6, 1);
    new_x = _x.block(0, 0, 6, 1);
    _x = new_x;
    std::cout << "New x:\n" << new_x << "\n\n";
}

void
EkfSlamAuv::addLandmark(const Eigen::VectorXd& landmark,
                        const Eigen::MatrixXd& landmark_cov,
                        const std::string landmark_id)
{
  // Map landmark name
  _mapped_lamdmarks[landmark_id] = _number_of_landmarks;
  _id_to_mapped_lamdmark[_number_of_landmarks] = landmark_id;

  // Increase P matrix
  Eigen::MatrixXd new_P;
  new_P = Eigen::MatrixXd::Identity(_x.size() + 6, _x.size() + 6);
  new_P.block(0, 0, _x.size(), _x.size()) = _P;

  // Correlations landmark position and vehicle position
  new_P.block(6 + 6*_number_of_landmarks, 0, 3, 3) = _P.block(0, 0, 3, 3);
  new_P.block(0, 6 + 6*_number_of_landmarks, 3, 3) = _P.block(0, 0, 3, 3);

  Eigen::Matrix3d m = _auv_orientation.toRotationMatrix() * landmark_cov.block(0, 0, 3, 3) *
      _auv_orientation.toRotationMatrix().transpose() + _P.block(0, 0, 3, 3);
  Eigen::Matrix3d m2 = _auv_orientation.toRotationMatrix() * landmark_cov.block(3, 3, 3, 3) *
      _auv_orientation.toRotationMatrix().transpose();
  new_P.block(6 + 6*_number_of_landmarks, 6 + 6*_number_of_landmarks, 3, 3) = m;
  new_P.block(9 + 6*_number_of_landmarks, 9 + 6*_number_of_landmarks, 3, 3) = m2;

  std::cout << "Old P:\n" << _P << "\n\n";
  std::cout << "New P:\n" << new_P << "\n\n";
  _P = new_P;

  // Increase state vector
  Eigen::VectorXd new_x;
  new_x = Eigen::MatrixXd::Zero(_x.size() + 6, 1);
  new_x.block(0, 0, _x.size(), 1) = _x;
  new_x.block(_x.size(), 0, 6, 1) = landmark;

  std::cout << "Old x:\n" << _x << "\n\n";
  std::cout << "New x:\n" << new_x << "\n\n";
  _x = new_x;

  _number_of_landmarks++;
}

void EkfSlamAuv::createLandmarkMeasure(const Eigen::Vector3d landmark_position,
                                       const Eigen::Vector3d landmark_orientation_RPY,
                                       const unsigned int landmark_id,
                                       const Eigen::Matrix3d rot,
                                       const Eigen::MatrixXd covariance,
                                       Eigen::VectorXd& z,
                                       Eigen::MatrixXd& r,
                                       Eigen::MatrixXd& h,
                                       Eigen::MatrixXd& v)
{
  // Define measurement
  z.resize(6, 1);
  z(0) = landmark_position(0);
  z(1) = landmark_position(1);
  z(2) = landmark_position(2);
  z(3) = landmark_orientation_RPY(0);
  z(4) = landmark_orientation_RPY(1);
  z(5) = landmark_orientation_RPY(2);

  h.resize(6, 6 + 6*_number_of_landmarks);
  h = Eigen::MatrixXd::Zero(6, 6 + 6*_number_of_landmarks);
  if (landmark_id < _number_of_landmarks)
  {
    h.block(0, 0, 3, 3) = -1*rot;
    h.block(0, 6+landmark_id*6, 3, 3) = rot;
    h.block(3, 9+landmark_id*6, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  }
  else
  {
    std::cerr << "Invalid landmark id " << landmark_id << "\n";
  }

  // If the landmark detector does not fill the orientation covariance
  r = covariance;
  if ( r(3, 3) == 0 && r(4, 4) == 0 && r(5, 5) == 0 )
  {
    r(3, 3) = r(0, 0);
    r(4, 4) = r(0, 0);
    r(5, 5) = r(0, 0);
  }
  v = Eigen::MatrixXd::Identity(6, 6);
}

void EkfSlamAuv::createVelocityMeasure(const Eigen::Vector3d velocity,
                                       const Eigen::Matrix3d velocity_cov,
                                       Eigen::VectorXd& z,
                                       Eigen::MatrixXd& r,
                                       Eigen::MatrixXd& h,
                                       Eigen::MatrixXd& v)
{
  z.resize(3);
  z = velocity;

  r.resize(3, 3);
  r = velocity_cov;

  h.resize(3, 6 + 6*_number_of_landmarks);
  h = Eigen::MatrixXd::Zero(3, 6 + 6*_number_of_landmarks);
  h(0, 3) = 1.0;
  h(1, 4) = 1.0;
  h(2, 5) = 1.0;

  v.resize(3, 3);
  v = Eigen::MatrixXd::Identity(3, 3);
}

void EkfSlamAuv::createPositionMeasure(const Eigen::Vector3d position,
                                       const Eigen::Matrix3d position_cov,
                                       Eigen::VectorXd& z,
                                       Eigen::MatrixXd& r,
                                       Eigen::MatrixXd& h,
                                       Eigen::MatrixXd& v)
{
  // Check size (GPS = 2, depth_sensor = 1, USBL = 3, ...)
  unsigned int total_size = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    if (position_cov(i, i) < 999)
    {
      total_size++;
    }
  }

  // Resize as necessary
  z.resize(total_size);
  z = Eigen::MatrixXd::Zero(total_size, 1);
  r.resize(total_size, total_size);
  h.resize(total_size, 6 + 6*_number_of_landmarks);
  h = Eigen::MatrixXd::Zero(total_size, 6 + 6*_number_of_landmarks);
  v.resize(total_size, total_size);
  v = Eigen::MatrixXd::Identity(total_size, total_size);
  // Fill the matrices/vectors
  unsigned int j = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    if (position_cov(i, i) < 999)
    {
      z(j) = position(i);
      r(j, j) = position_cov(i, i);
      h(j, i) = 1.0;
      j++;
    }
  }

  // r matrix is filled only with the diagonal
  // to avoid that:
  if (total_size == 3)
  {
    r = position_cov;
  }
  else if (total_size == 2)
  {
    if (position_cov(0, 0) < 999 && position_cov(1, 1) < 999)
    {
      r = position_cov.block(0, 0, 2, 2);
    }
    else if (position_cov(0, 0) < 999 && position_cov(2, 2) < 999)
    {
      r(0, 0) = position_cov(0, 0); r(0, 1) = position_cov(0, 2);
      r(1, 0) = position_cov(2, 0); r(1, 1) = position_cov(2, 2);
    }
    else if (position_cov(1, 1) < 999 && position_cov(2, 2) < 999)
    {
      r = position_cov.block(1, 1, 2, 2);
    }
  }
}

// Implement virtual methods defined in EkfBase class
void EkfSlamAuv::normalizeInnovation(Eigen::VectorXd& innovation) const
{
  if (innovation.cols() == 6)
  {
    // If landmark update normalize angles in innovation
    innovation(3) = normalizeAngle(innovation(3));
    innovation(4) = normalizeAngle(innovation(4));
    innovation(5) = normalizeAngle(innovation(5));
  }
}

void EkfSlamAuv::normalizeState()
{
  // Normalize landmark angles after an update
  for (unsigned int i = 0; i < static_cast<unsigned int>((_x.size() / 6) - 1); i++)
  {
    _x(9 + i*6) = normalizeAngle(_x(9 + i*6));
    _x(10 + i*6) = normalizeAngle(_x(10 + i*6));
    _x(11 + i*6) = normalizeAngle(_x(11 + i*6));
  }
}

Eigen::VectorXd EkfSlamAuv::computeU() const
{
  Eigen::Vector3d tmp = getRPY(_auv_orientation.toRotationMatrix());
  Eigen::Vector3d u(tmp(0), tmp(1), tmp(2));
  return u;
}

Eigen::MatrixXd EkfSlamAuv::computeA(const double t, const Eigen::VectorXd u) const
{
  assert(u.size() == 3);

  // A is the jacobian matrix of f(x)
  double roll = u(0);
  double pitch = u(1);
  double yaw = u(2);

  unsigned int a_size = 6 + 6 * _number_of_landmarks;
  Eigen::MatrixXd A(a_size, a_size);
  A = Eigen::MatrixXd::Identity(a_size, a_size);
  A(0, 3) = cos(pitch)*cos(yaw)*t;
  A(0, 4) = -cos(roll)*sin(yaw)*t + sin(roll)*sin(pitch)*cos(yaw)*t;
  A(0, 5) = sin(roll)*sin(yaw)*t + cos(roll)*sin(pitch)*cos(yaw)*t;

  A(1, 3) = cos(pitch)*sin(yaw)*t;
  A(1, 4) = cos(roll)*cos(yaw)*t + sin(roll)*sin(pitch)*sin(yaw)*t;
  A(1, 5) = -sin(roll)*cos(yaw)*t + cos(roll)*sin(pitch)*sin(yaw)*t;

  A(2, 3) = -sin(pitch)*t;
  A(2, 4) = sin(roll)*cos(pitch)*t;
  A(2, 5) = cos(roll)*cos(pitch)*t;

  return A;
}

Eigen::MatrixXd EkfSlamAuv::computeW(const double t, const Eigen::VectorXd u) const
{
  // The noise in the system is a term added to the acceleration:
  // e.g. x[0] = x1 + cos(pitch)*cos(yaw)*(vx1*t +  Eax) *t^2/2)-..
  // then, dEax/dt of x[0] = cos(pitch)*cos(yaw)*t^2/2

  assert(u.size() == 3);

  double roll = u(0);
  double pitch = u(1);
  double yaw = u(2);
  double t2 = (t*t)/2;

  unsigned int w_size = 6 + 6 * _number_of_landmarks;
  Eigen::MatrixXd W(w_size, 3);
  W = Eigen::MatrixXd::Zero(w_size, 3);
  W(0, 0) = cos(pitch)*cos(yaw)*t2;
  W(0, 1) = -cos(roll)*sin(yaw)*t2 + sin(roll)*sin(pitch)*cos(yaw)*t2;
  W(0, 2) = sin(roll)*sin(yaw)*t2 + cos(roll)*sin(pitch)*cos(yaw)*t2;

  W(1, 0) = cos(pitch)*sin(yaw)*t2;
  W(1, 1) = cos(roll)*cos(yaw)*t2 + sin(roll)*sin(pitch)*sin(yaw)*t2;
  W(1, 2) = -sin(roll)*cos(yaw)*t2 + cos(roll)*sin(pitch)*sin(yaw)*t2;

  W(2, 0) = -sin(pitch)*t2;
  W(2, 1) = sin(roll)*cos(pitch)*t2;
  W(2, 2) = cos(roll)*cos(pitch)*t2;

  W(3, 0) = t;
  W(4, 1) = t;
  W(5, 2) = t;

  // Add some noise to landmakrs orientation
  // otherwise they will not move at all
  for (unsigned int i = 0; i < _number_of_landmarks; i++)
  {
    W(9 + i * 6, 0)  = 0.005;
    W(10 + i * 6, 1) = 0.005;
    W(11 + i * 6, 2) = 0.005;
  }
  return W;
}


Eigen::VectorXd EkfSlamAuv::f(const Eigen::VectorXd x_1, const double t, const Eigen::VectorXd u) const
{
  // The model takes as state 3D position (x, y, z) and linear
  //  velocity (vx, vy, vz). The input is the orientation
  //  (roll, pitch yaw) and the linear accelerations (ax, ay, az). """

  assert(u.size() == 3);
  double roll = u(0);
  double pitch = u(1);
  double yaw = u(2);
  double x1 = x_1(0);
  double y1 = x_1(1);
  double z1 = x_1(2);
  double vx1 = x_1(3);
  double vy1 = x_1(4);
  double vz1 = x_1(5);

  Eigen::VectorXd x(6+6*_number_of_landmarks);
  x = x_1;

  // Compute Prediction Model with constant velocity
  x(0) = x1 + cos(pitch)*cos(yaw)*(vx1*t) - cos(roll)*sin(yaw)*(vy1*t) + sin(roll)*sin(pitch)*cos(yaw)*(vy1*t) +
      sin(roll)*sin(yaw)*(vz1*t) + cos(roll)*sin(pitch)*cos(yaw)*(vz1*t);
  x(1) = y1 + cos(pitch)*sin(yaw)*(vx1*t) + cos(roll)*cos(yaw)*(vy1*t) + sin(roll)*sin(pitch)*sin(yaw)*(vy1*t) -
      sin(roll)*cos(yaw)*(vz1*t) + cos(roll)*sin(pitch)*sin(yaw)*(vz1*t);
  x(2) = z1 - sin(pitch)*(vx1*t) + sin(roll)*cos(pitch)*(vy1*t) + cos(roll)*cos(pitch)*(vz1*t);
  x(3) = vx1;
  x(4) = vy1;
  x(5) = vz1;
  return x;
}










/*
double EKF_SLAM::rangeUpdate(double z, double sigma, double th, bool reallydo) {
    // Rest of values not initialized
    if (!_imu_init || !_range_init) {
        // Return
        std::cout << "cannot range update!!" << std::endl;
        return -1.0;
    } else {
        // Debug
        if (DEBUG) {
            std::cout << std::endl << "Range update with " << z << " " << sigma << "  th: " << th << std::endl;
        }
        // Range to beacon from robot position
        // h(x) = sqrt( (xr-xb)^2 + (yr-yb)^2 + (zr-zb)^2 ) + wz
        Eigen::Vector3d diff = _x.tail(3) - _x.head(3);
        Eigen::VectorXd h = Eigen::VectorXd::Zero(1);
        h(0) = diff.norm();

        // Measurement
        Eigen::VectorXd r = Eigen::VectorXd::Zero(1);
        r(0) = z;

        // Linearization of h() into H
        double inv = 1.0 / h(0);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 9);
        H(0, 0) = -diff(0) * inv;
        H(0, 1) = -diff(1) * inv;
        H(0, 2) = -diff(2) * inv;
        H(0, 6) = -H(0, 0);
        H(0, 7) = -H(0, 1);
        H(0, 8) = -H(0, 2);

        // R matrix
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1);
        R(0,0) = sigma;

        // Update
        double y;
        if (reallydo) {
            y = make_update(h, r, H, R, th);
        } else {
            y = test_update(h, r, H, R, th);
        }
        // Return innovation for weighting
        return y;
    }
} */
