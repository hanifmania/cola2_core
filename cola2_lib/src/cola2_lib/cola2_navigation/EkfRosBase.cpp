#include "cola2_lib/cola2_navigation/EkfRosBase.h"

EkfRosBase::EkfRosBase(const std::string name, bool debug):
  _name(name),
  _diagnostic(_n, name, "soft"),
  _debug(debug),
  _altitude(0.5),
  _tf_listener()
{
  // Publishers
  _pub_odom = _n.advertise<nav_msgs::Odometry>("/pose_ekf_slam/odometry", 1);
  _pub_map = _n.advertise<cola2_msgs::Map>("/pose_ekf_slam/map", 1);
  _pub_nav_sts = _n.advertise< auv_msgs::NavSts >("/cola2_navigation/nav_sts", 1);
  _pub_landmarks = _n.advertise<visualization_msgs::MarkerArray>("/pose_ekf_slam/landmarks", 1);
  _pub_gps_ned = _n.advertise<geometry_msgs::PoseStamped>("/cola2_navigation/gps_ned", 1);
  _pub_usbl_ned = _n.advertise<geometry_msgs::PointStamped>("/cola2_navigation/usbl_ned", 1);

  // _pub_covariance_marker = _n.advertise<visualization_msgs::Marker>("/pose_ekf_slam/covariance_marker", 1);
  _pub_altitude_range = _n.advertise<sensor_msgs::Range>("/cola2_perception/altitude", 1);

  // Init timer
  _timer = _n.createTimer(ros::Duration(1.0), &EkfRosBase::checkDiagnostics, this);
}

void EkfRosBase::checkDiagnostics(const ros::TimerEvent& e)
{
  _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  _diagnostic.add("test", "ok");
  _diagnostic.publish();
  std::cout << "Publish diagnostics\n";
}

bool EkfRosBase::getTfs()
{
  for (std::vector<std::string>::iterator it = _sensor_frames_id.begin(); it != _sensor_frames_id.end(); it++)
  {
    tf::StampedTransform transform;
    std::cout << "wait for " << *it << " to " << _vehicle_frame_id << " TF.\n";
    try
    {
      _listener.waitForTransform(_vehicle_frame_id, *it, ros::Time(0), ros::Duration(5.0));
      _listener.lookupTransform(_vehicle_frame_id, *it, ros::Time(0), transform);
      _ekf_slam_auv->setTransformation(*it,
                                       Eigen::Quaterniond(transform.getRotation().getW(),
                                                          transform.getRotation().getX(),
                                                          transform.getRotation().getY(),
                                                          transform.getRotation().getZ()),
                                       Eigen::Vector3d(transform.getOrigin().getX(),
                                                       transform.getOrigin().getY(),
                                                       transform.getOrigin().getZ()));
    }
    catch (tf::TransformException ex)
    {
      std::cerr << "[Navigator] TF for " << *it << " not found in TF tree!\n";
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
  }
  return true;
}

void EkfRosBase::publish(ros::Time stamp)
{
  Eigen::Quaterniond orientation;
  Eigen::Vector3d angular_velocity;
  Eigen::Matrix3d orientation_cov, angular_velocity_cov;
  _ekf_slam_auv->getImuData(orientation,
                            orientation_cov,
                            angular_velocity,
                            angular_velocity_cov);

  // Publish Odometry message
  nav_msgs::Odometry odom;
  odom.header.frame_id = _world_frame_id;
  odom.header.stamp = stamp;
  odom.pose.pose.position.x = _ekf_slam_auv->getStateVector()(0);
  odom.pose.pose.position.y = _ekf_slam_auv->getStateVector()(1);
  odom.pose.pose.position.z = _ekf_slam_auv->getStateVector()(2);
  odom.pose.pose.orientation.w = orientation.w();
  odom.pose.pose.orientation.x = orientation.x();
  odom.pose.pose.orientation.y = orientation.y();
  odom.pose.pose.orientation.z = orientation.z();
  odom.twist.twist.linear.x = _ekf_slam_auv->getStateVector()(3);
  odom.twist.twist.linear.y = _ekf_slam_auv->getStateVector()(4);
  odom.twist.twist.linear.z = _ekf_slam_auv->getStateVector()(5);
  odom.twist.twist.angular.x = angular_velocity(0);
  odom.twist.twist.angular.y = angular_velocity(1);
  odom.twist.twist.angular.z = angular_velocity(2);

  Eigen::MatrixXd pose_cov(6, 6);
  pose_cov.block(0, 0, 3, 3) = _ekf_slam_auv->getCovarianceMatrix().block(0, 0, 3, 3);
  pose_cov.block(3, 3, 3, 3) = orientation_cov;
  Eigen::MatrixXd twist_cov(6, 6);
  twist_cov.block(0, 0, 3, 3) = _ekf_slam_auv->getCovarianceMatrix().block(3, 3, 3, 3);
  twist_cov.block(3, 3, 3, 3) = angular_velocity_cov;

  // TODO: Avoid double FOR!!!
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < 6; j++)
    {
      odom.pose.covariance.at(i*6+j) = pose_cov(i, j);
      odom.twist.covariance.at(i*6+j) = twist_cov(i, j);
    }
  }
  _pub_odom.publish(odom);

  // Publish Nav Status
  auv_msgs::NavSts nav_sts;
  nav_sts.header.frame_id = _vehicle_frame_id;
  nav_sts.header.stamp = stamp;

  nav_sts.altitude = _altitude;

  nav_sts.body_velocity.x = _ekf_slam_auv->getStateVector()(3);
  nav_sts.body_velocity.y = _ekf_slam_auv->getStateVector()(4);
  nav_sts.body_velocity.z = _ekf_slam_auv->getStateVector()(5);

  double lat, lon, height;
  _ned->ned2Geodetic(_ekf_slam_auv->getStateVector()(0),
                     _ekf_slam_auv->getStateVector()(1),
                     _ekf_slam_auv->getStateVector()(2),
                     lat, lon, height);
  nav_sts.global_position.latitude = lat;
  nav_sts.global_position.longitude = lon;

  Eigen::Vector3d euler = getRPY(orientation.toRotationMatrix());
  nav_sts.orientation.roll = euler(0);
  nav_sts.orientation.pitch = euler(1);
  nav_sts.orientation.yaw = euler(2);

  nav_sts.orientation_rate.roll = angular_velocity(0);
  nav_sts.orientation_rate.pitch = angular_velocity(1);
  nav_sts.orientation_rate.yaw = angular_velocity(2);

  nav_sts.origin.latitude = _init_latitude;
  nav_sts.origin.longitude = _init_longitude;

  nav_sts.position.north = _ekf_slam_auv->getStateVector()(0);
  nav_sts.position.east = _ekf_slam_auv->getStateVector()(1);
  nav_sts.position.depth = _ekf_slam_auv->getStateVector()(2);

  nav_sts.position_variance.north = _ekf_slam_auv->getCovarianceMatrix()(0, 0);
  nav_sts.position_variance.east = _ekf_slam_auv->getCovarianceMatrix()(1, 1);
  nav_sts.position_variance.depth = _ekf_slam_auv->getCovarianceMatrix()(2, 2);
  nav_sts.orientation_variance.roll = orientation_cov(0, 0);
  nav_sts.orientation_variance.pitch = orientation_cov(1, 1);
  nav_sts.orientation_variance.yaw = orientation_cov(2, 2);

  _pub_nav_sts.publish(nav_sts);

  // Publish _vehicle_ TF
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                  odom.pose.pose.position.y,
                                  odom.pose.pose.position.z));
  transform.setRotation(tf::Quaternion(odom.pose.pose.orientation.x,
                                       odom.pose.pose.orientation.y,
                                       odom.pose.pose.orientation.z,
                                       odom.pose.pose.orientation.w));
  _br.sendTransform(tf::StampedTransform(transform,
                                         stamp,
                                         _world_frame_id,
                                         _vehicle_frame_id));

  // Publish altitude range
  sensor_msgs::Range range;
  range.header.frame_id = "/altitude_sensor";
  range.header.stamp = stamp;
  range.max_range = 60.0;
  range.min_range = 0.3;
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = 0.05;
  if (_altitude > 0.0)
    range.range = _altitude;
  else
    range.range = 0.0;

  _pub_altitude_range.publish(range);

  // Publish altitude TF
  tf::Transform transform_altitude;
  transform_altitude.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  transform_altitude.setRotation(tf::Quaternion(0.0, -0.7071, 0.0, 0.7071));  // -90º in Y axis rotation
  _br.sendTransform(tf::StampedTransform(transform_altitude,
                                         stamp,
                                         _vehicle_frame_id,
                                         "/altitude_sensor"));

  // Publish list of landmarks (Map)
  cola2_msgs::Map map;
  map.header.frame_id = "world";
  map.header.stamp = stamp;

  // Check if there are landmarks in the state vector
  for (unsigned int i = 0; i < _ekf_slam_auv->getNumberOfLandmarks(); i++)
  {
    cola2_msgs::Landmark landmark;
    landmark.landmark_id = _ekf_slam_auv->getLandmarkId(i);
    landmark.last_update.fromSec(_ekf_slam_auv->getLandmarkLastUpdate(i));
    landmark.pose.pose.position.x = _ekf_slam_auv->getStateVector()(6 + 6*i);
    landmark.pose.pose.position.y = _ekf_slam_auv->getStateVector()(7 + 6*i);
    landmark.pose.pose.position.z = _ekf_slam_auv->getStateVector()(8 + 6*i);
    Eigen::Quaterniond quat = euler2Quaternion(_ekf_slam_auv->getStateVector()(9 + 6*i),
                                               _ekf_slam_auv->getStateVector()(10 + 6*i),
                                               _ekf_slam_auv->getStateVector()(11 + 6*i));
    landmark.pose.pose.orientation.w = quat.w();
    landmark.pose.pose.orientation.x = quat.x();
    landmark.pose.pose.orientation.y = quat.y();
    landmark.pose.pose.orientation.z = quat.z();

    Eigen::MatrixXd landmark_cov(6, 6);
    landmark_cov = _ekf_slam_auv->getCovarianceMatrix().block(_ekf_slam_auv->getNumberOfLandmarks()*6,
                                                              _ekf_slam_auv->getNumberOfLandmarks()*6,
                                                              6, 6);
    // TODO: Avoid double FOR!!!
    for (unsigned int i = 0; i < 6; i++)
    {
      for (unsigned int j = 0; j < 6; j++)
      {
        landmark.pose.covariance.at(i*6 + j) = landmark_cov(i, j);
      }
    }
    map.landmark.push_back(landmark);
  }
  _pub_map.publish(map);

  // Publish Marker array for landmarks
  visualization_msgs::MarkerArray landmarks_array;
  for (unsigned int i = 0; i < _ekf_slam_auv->getNumberOfLandmarks(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = _world_frame_id;
    marker.header.stamp = stamp;
    marker.ns = _ekf_slam_auv->getLandmarkId(i);
    marker.id = 0;
    marker.type = marker.CUBE;
    marker.action = marker.ADD;
    marker.pose.position.x = _ekf_slam_auv->getStateVector()(6 + 6*i);
    marker.pose.position.y = _ekf_slam_auv->getStateVector()(7 + 6*i);
    marker.pose.position.z = _ekf_slam_auv->getStateVector()(8 + 6*i);

    Eigen::Quaterniond quat = euler2Quaternion(_ekf_slam_auv->getStateVector()(9 + 6*i),
                                               _ekf_slam_auv->getStateVector()(10 + 6*i),
                                               _ekf_slam_auv->getStateVector()(11 + 6*i));
    marker.pose.orientation.w = quat.w();
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();

    marker.scale.x = 0.8;
    marker.scale.y = 0.4;
    marker.scale.z = 0.2;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 1.0;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(2.0);
    marker.frame_locked = false;
    landmarks_array.markers.push_back(marker);
  }
  _pub_landmarks.publish(landmarks_array);

  // Save last 10s of positions for the USBL delayed data
  _last_positions.push_back(Eigen::Vector3d(stamp.toSec(),
                                            _ekf_slam_auv->getStateVector()(0),
                                            _ekf_slam_auv->getStateVector()(1)));
  bool found = false;
  while (!found)
  {
    if (_last_positions.size() > 0 && (stamp.toSec() - _last_positions.at(0)[0]) > 10.0)
    {
      _last_positions.erase(_last_positions.begin());
    }
    else
    {
      found = true;
    }
  }
}

void EkfRosBase::checkFrameId(std::string frame_id)
{
  // Check if we already have the transform in memory
  if (std::find(_frame_ids.begin(), _frame_ids.end(), frame_id) == _frame_ids.end())
  {
    // Check the tf listener
    bool found(false);
    ros::Time time;
    tf::StampedTransform transf;
    try
    {
      _tf_listener.waitForTransform(_vehicle_frame_id, frame_id, time, ros::Duration(1.0));
      _tf_listener.lookupTransform(_vehicle_frame_id, frame_id, time, transf);
      found = true;
      Eigen::Quaterniond quat;
      Eigen::Vector3d trans;
      tf::quaternionTFToEigen(transf.getRotation(), quat);
      tf::vectorTFToEigen(transf.getOrigin(), trans);
      _ekf_slam_auv->setTransformation(frame_id, quat, trans);
      _frame_ids.push_back(frame_id);
      ROS_INFO("Registered transform '%s' from tf", frame_id.c_str());
    }
    catch (std::exception e)
    {
      // Nothing to do...
      ROS_WARN("tf error: frame robot to %s not found", frame_id.c_str());
    }

    // Load from params
    if (!found)
    {
      std::vector< double > tf_data;
      cola2::rosutil::loadParam(frame_id+"/tf", tf_data);
      if (tf_data.size() == 6)
      {
        ROS_WARN("LOADED TF FROM PARAM SERVER --> %s", frame_id.c_str());
        _ekf_slam_auv->setTransformation(frame_id,
                                         euler2Quaternion(deg2Rad(tf_data.at(3)), deg2Rad(tf_data.at(4)), deg2Rad(tf_data.at(5))),
                                         Eigen::Vector3d(tf_data.at(0), tf_data.at(1), tf_data.at(2)));
        _frame_ids.push_back(frame_id);
      }
      else
      {
        ROS_ERROR("ERROR! TF %s NOT FOUND", frame_id.c_str());
      }
    }
  }
}

Eigen::Vector3d EkfRosBase::getPositionIncrementFrom(double time)
{
  unsigned int i = 0;
  while (i < _last_positions.size())
  {
    if (_last_positions.at(i)[0] > time)
    {
      return Eigen::Vector3d(_last_positions.at(_last_positions.size() - 1)[0] - _last_positions.at(i)[0],
          _last_positions.at(_last_positions.size() - 1)[1] - _last_positions.at(i)[1],
          _last_positions.at(_last_positions.size() - 1)[2] - _last_positions.at(i)[2]);
    }
    else
    {
      i++;
    }
  }
  // If not position found return a negative time
  return Eigen::Vector3d(-1.0, 0.0, 0.0);
}
