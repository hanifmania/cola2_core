#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFROSBASE_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFROSBASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <cola2_msgs/Map.h>
#include <auv_msgs/NavSts.h>

#include <string>
#include <vector>

#include "./EkfSlamAuv.h"
#include "./Ned.h"
#include "../cola2_rosutils/DiagnosticHelper.h"
#include "../cola2_rosutils/RosUtil.h"

class EkfRosBase
{
 public:
  EkfRosBase(const std::string name, bool debug = true);

  void checkDiagnostics(const ros::TimerEvent& e);

  bool getTfs();

  void publish(ros::Time stamp);

  void checkFrameId(std::string frame_id);

  Eigen::Vector3d getPositionIncrementFrom(double time);

 protected:
  // EKF ptr
  EkfSlamAuv *_ekf_slam_auv;

  // Name
  std::string _name;

  // Node handle
  ros::NodeHandle _n;

  // Diagnostics
  cola2::rosutils::DiagnosticHelper _diagnostic;

  // Publisher
  ros::Publisher _pub_odom;
  ros::Publisher _pub_map;
  ros::Publisher _pub_nav_sts;
  ros::Publisher _pub_gps_ned;

  // .. for visualization purposes only
  ros::Publisher _pub_landmarks;
  ros::Publisher _pub_covariance_marker;
  ros::Publisher _pub_altitude_range;

  // NED ptr
  Ned *_ned;
  double _init_latitude;
  double _init_longitude;

  // Enable debug publications
  bool _debug;

  // Other
  std::string _vehicle_frame_id;
  std::string _world_frame_id;
  std::vector<std::string> _sensor_frames_id;
  tf::TransformBroadcaster _br;
  tf::TransformListener _listener;
  double _altitude;
  ros::Timer _timer;
  std::vector< std::string > _frame_ids;

  // Tf
  tf::TransformListener _tf_listener;

  // for delayed USBL
  std::vector<Eigen::Vector3d> _last_positions;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_EKFROSBASE_H_
