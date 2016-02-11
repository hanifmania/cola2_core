#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_TRANSFORMATIONS_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_TRANSFORMATIONS_H_

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <iostream>

#include "./nav_utils.h"

namespace transformations
{
/*
 * Given a position measured at distance translation_from_origin
 * with the body orientated as body_orientation, computes the
 * measured position in the origen frame
 */
Eigen::Vector3d position(const Eigen::Vector3d measured_position,
                         const Eigen::Quaterniond body_orientation,
                         const Eigen::Vector3d translation_from_origin);

/*
 * Given the position of a landmark (landmark_pose) measured from a
 * sensor placed at distance (translation_from_origin) and rotation (rotation_from_origen)
 * from the origin, computes the landmark position wrt the origin
 */
Eigen::Vector3d landmarkPosition(const Eigen::Vector3d measured_landmark_position,
                                 const Eigen::Quaterniond rotation_from_origen,
                                 const Eigen::Vector3d translation_from_origin);

/*
 * Given an orientation (measured_orientation) measured from a device
 * attached to the origin with a rotation (rotation_from_origen), computes
 * the orientation of the origin frame.
 */
Eigen::Quaterniond orientation(const Eigen::Quaterniond measured_orientation,
                               const Eigen::Quaterniond rotation_from_origen);

/*
 * Given a measured landmark orientation (measured_landmark_orientation) measured from a device
 * attached to a the origin with a rotation (rotation_from_origen), computes
 * the landmark orientation from the origin.
 */
Eigen::Quaterniond landmarkOrientation(const Eigen::Quaterniond measured_landmark_orientation,
                                       const Eigen::Quaterniond rotation_from_origen);

/*
 * Given a measured linear velocity (measured_linear_velocity) displaced
 * (translation_from_origen) and rotated (rotation_from_origen) from the origen,
 * computes the linear velocity in the origin.
 */
Eigen::Vector3d linearVelocity(const Eigen::Vector3d measured_linear_velocity,
                               const Eigen::Vector3d origin_angular_velocity,
                               const Eigen::Quaterniond rotation_from_origen,
                               const Eigen::Vector3d translation_from_origen);

/*
 * Given a measured angular velocity (measured_angular_velocity) measured a
 * (rotation_from_origen) from the origin, computes the angular velocity in the origin
 */
Eigen::Vector3d angularVelocity(const Eigen::Vector3d measured_angular_velocity,
                                const Eigen::Quaterniond rotation_from_origen);

/*
 * Computes the covariance a measured position after being transformed to
 * the origin frame.
 */
Eigen::Matrix3d positionCovariance(const Eigen::Matrix3d measured_position_covariance,
                                   const Eigen::Matrix3d origen_orientation_covariance,
                                   const Eigen::Quaterniond origen_orientation,
                                   const Eigen::Vector3d translation_from_origin);

/*
 * Computes the covariance a measured linear velocity after being transformed to
 * the origin frame.
 */
Eigen::Matrix3d linearVelocityCov(const Eigen::Matrix3d measured_linear_velocity_covariance,
                                  const Eigen::Matrix3d origin_angular_velocity_covariance,
                                  const Eigen::Quaterniond rotation_from_origen,
                                  const Eigen::Vector3d translation_from_origen);

/*
 * Applies a rotation to a covariance.
 */
Eigen::Matrix3d rotatedCovariance(const Eigen::Matrix3d covariance,
                                  const Eigen::Quaterniond rotation);

}  // namespace transformations

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_NAVIGATION_TRANSFORMATIONS_H_
