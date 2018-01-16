
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/cola2_navigation/nav_utils.h"

Eigen::Vector3d getRPY(const Eigen::Matrix3d& rotation)
{
  int solution_number = 1;
  Eigen::Vector3d rpy;

  struct Euler
  {
    double yaw;
    double pitch;
    double roll;
  };

  Euler euler_out;
  Euler euler_out2;  // second solution
  // get the pointer to the raw data

  // Check that pitch is not at a singularity
  // Check that pitch is not at a singularity
  if (fabs(rotation(2, 0)) >= 1)
  {
    euler_out.yaw = 0;
    euler_out2.yaw = 0;

    // From difference of angles formula
    if (rotation(2, 0) < 0)  // gimbal locked down
    {
      double delta = atan2(rotation(0, 1), rotation(0, 2));
      euler_out.pitch = M_PI/2.0;
      euler_out2.pitch = M_PI/2.0;
      euler_out.roll = delta;
      euler_out2.roll = delta;
    }
    else  // gimbal locked up
    {
      double delta = atan2(-rotation(0, 1), -rotation(0, 2));
      euler_out.pitch = -M_PI / 2.0;
      euler_out2.pitch = -M_PI / 2.0;
      euler_out.roll = delta;
      euler_out2.roll = delta;
    }
  }
  else
  {
    euler_out.pitch = -asin(rotation(2, 0));
    euler_out2.pitch = M_PI - euler_out.pitch;

    euler_out.roll = atan2(rotation(2, 1)/cos(euler_out.pitch), rotation(2, 2)/cos(euler_out.pitch));
    euler_out2.roll = atan2(rotation(2, 1)/cos(euler_out2.pitch), rotation(2, 2)/cos(euler_out2.pitch));

    euler_out.yaw = atan2(rotation(1, 0)/cos(euler_out.pitch), rotation(0, 0)/cos(euler_out.pitch));
    euler_out2.yaw = atan2(rotation(1, 0)/cos(euler_out2.pitch), rotation(0, 0)/cos(euler_out2.pitch));
  }
  if (solution_number == 1)
  {
    rpy(0) = euler_out.roll;
    rpy(1) = euler_out.pitch;
    rpy(2) = euler_out.yaw;
  }
  else
  {
    rpy(0) = euler_out2.roll;
    rpy(1) = euler_out2.pitch;
    rpy(2) = euler_out2.yaw;
  }

  return rpy;
}

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
}

double dms2Deg(const double degree_minutes, const char hemisphere)
{
  assert(hemisphere == 'N' || hemisphere == 'S' || hemisphere == 'E' || hemisphere == 'W');

  unsigned int degrees = static_cast<unsigned int>(degree_minutes / 100);
  double minutes = degree_minutes - (degrees * 100);

  if (hemisphere == 'E' || hemisphere == 'N')
  {
    return degrees + (minutes / 60.0);
  }
  else if (hemisphere == 'W' || hemisphere == 'S')
  {
    return - (degrees + (minutes / 60));
  }
  return 0.0;
}

double dms2DegInt(const double degree_minutes, const int hemisphere)
{
  switch (hemisphere)
  {
  case 0:
    return dms2Deg(degree_minutes, static_cast<char>('N'));
  case 1:
    return dms2Deg(degree_minutes, static_cast<char>('S'));
  case 2:
    return dms2Deg(degree_minutes, static_cast<char>('W'));
  case 3:
    return dms2Deg(degree_minutes, static_cast<char>('E'));
  default:
    // std::cerr << "Invalid hemisphere: " << hemisphere << "\n";
    return 0.0;
  }
}

double rad2Deg(const double radians)
{
  return (radians / M_PI) * 180.0;
}

double deg2Rad(const double degrees)
{
  return (degrees / 180.0) * M_PI;
}

double normalizeAngle(const double angle)
{
  return (angle + (2.0* M_PI * floor((M_PI-angle) / (2.0*M_PI))));
}
