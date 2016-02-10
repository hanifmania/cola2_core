#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_NAV_UTILS_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_NAV_UTILS_H_

#include <eigen3/Eigen/Geometry>
#include <math.h>

#include <iostream>

Eigen::Vector3d getRPY(const Eigen::Matrix3d& rotation);

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);

double dms2Deg(const double degree_minutes, const char hemisphere);

double dms2DegInt(const double degree_minutes, const int hemisphere);

double rad2Deg(const double radians);

double deg2Rad(const double degrees);

double normalizeAngle(const double angle);

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_NAV_UTILS_H_
