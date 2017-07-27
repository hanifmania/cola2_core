
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/cola2_navigation/transformations.h"

namespace transformations
{

Eigen::Vector3d position(const Eigen::Vector3d measured_position,
                         const Eigen::Quaterniond body_orientation,
                         const Eigen::Vector3d translation_from_origin)
{
  // std::cout << "Rotation Matrix: " << body_orientation.toRotationMatrix() << "\n";
  return measured_position - (body_orientation.toRotationMatrix() * translation_from_origin);
}

Eigen::Vector3d landmarkPosition(const Eigen::Vector3d measured_landmark_position,
                                 const Eigen::Quaterniond rotation_from_origen,
                                 const Eigen::Vector3d translation_from_origin)
{
  // To create the TF multiply a translation by a quaternion (rotation)
  Eigen::Translation< double, 3 > translation(translation_from_origin);
  Eigen::Transform< double, 3, Eigen::Projective > T = translation * rotation_from_origen;

  // Create an homogeneous vector and multiply it by the TF
  Eigen::Vector4d h = measured_landmark_position.homogeneous();
  h = T * h;

  // Return an unnormalized vector
  return h.hnormalized();
}

Eigen::Quaterniond orientation(const Eigen::Quaterniond measured_orientation,
                               const Eigen::Quaterniond rotation_from_origen )
{
  Eigen::Matrix3d orientation_data = measured_orientation.toRotationMatrix();
  Eigen::Matrix3d R = rotation_from_origen.toRotationMatrix();
  Eigen::Matrix3d ret = orientation_data * R.inverse();

  return Eigen::Quaterniond( ret );
}

Eigen::Quaterniond landmarkOrientation(const Eigen::Quaterniond measured_landmark_orientation,
                                       const Eigen::Quaterniond rotation_from_origen)
{
  return Eigen::Quaterniond(rotation_from_origen.toRotationMatrix() *
                            measured_landmark_orientation.toRotationMatrix());
}

Eigen::Vector3d linearVelocity(const Eigen::Vector3d measured_linear_velocity,
                               const Eigen::Vector3d origin_angular_velocity,
                               const Eigen::Quaterniond rotation_from_origen,
                               const Eigen::Vector3d translation_from_origen)
{
  // Align measured velocity with origin frame
  Eigen::Matrix3d R = rotation_from_origen.toRotationMatrix();
  Eigen::Vector3d v_r = R * measured_linear_velocity;

  // Once the velocity is rotates to the origin frame it is necessary to add
  // the linear velocity that appears when combining origin frame angular velocity
  // and displacement between sensor and origin frame
  return v_r - origin_angular_velocity.cross(translation_from_origen);
}

Eigen::Vector3d angularVelocity(const Eigen::Vector3d measured_angular_velocity,
                                const Eigen::Quaterniond rotation_from_origen)
{
  return rotation_from_origen.toRotationMatrix() * measured_angular_velocity;
}

Eigen::Matrix3d positionCovariance(const Eigen::Matrix3d measured_position_covariance,
                                   const Eigen::Matrix3d origen_orientation_covariance,
                                   const Eigen::Quaterniond origen_orientation,
                                   const Eigen::Vector3d translation_from_origin)
{
  Eigen::Vector3d angle = getRPY(origen_orientation.toRotationMatrix());

  double r = angle(0);
  double p = angle(1);
  double y = angle(2);

  double dx = translation_from_origin(0);
  double dy = translation_from_origin(1);
  double dz = translation_from_origin(2);

  double cr = cos(r);
  double sr = sin(r);
  double cp = cos(p);
  double sp = sin(p);
  double cy = cos(y);
  double sy = sin(y);

  //    RPY rotation matrix
  //    rpy = array([cy*cp,  cy*sp*sr - sy*cr,   cy*sp*cr + sy*sr,
  //                 sy*cp,  sy*sp*sr + cy*cr,   sy*sp*cr - cy*sr,
  //                 -sp,    cp*sr,              cp*cr]).reshape(3,3)

  //    Z = measured_pose - RPY*trans
  //    Zx = Mx - cy*cp*dx + (cy*sp*sr - sy*cr)*dy + (cy*sp*cr + sy*sr)*dz
  //    Zy = My - sy*cp*dx + (sy*sp*sr + cy*cr)*dy + (sy*sp*cr - cy*sr)*dz
  //    Zz = Mz - (-sp*dx) + cp*sr*dy + cp*cr*dz

  double d_r_x = (cy*sp*cr + sy*sr)*dy + (-cy*sp*sr + sy*cr)*dz;
  double d_r_y = (sy*sp*cr - cy*sr)*dy + (-sy*sp*sr - cy*cr)*dz;
  double d_r_z = -cp*cr*dy + cp*sr*dz;
  double d_p_x = -cy*sp*dx + cy*cp*sr*dy + cy*cp*cr*dz;
  double d_p_y = -sy*sp*dx + sy*cp*sr*dy + sy*cp*cr*dz;
  double d_p_z = -cp*dx - sp*sr*dy - sp*cr*dz;
  double d_y_x = -sy*cp*dx + (-sy*sp*sr - cy*cr)*dy + (-sy*sp*cr + cy*sr)*dz;
  double d_y_y = cy*cp*dx + (cy*sp*sr - sy*cr)*dy + (cy*sp*cr + sy*sr)*dz;
  double d_y_z = 0;

  Eigen::Matrix3d rpy_cov;
  rpy_cov << d_r_x, d_p_x, d_y_x,
      d_r_y, d_p_y, d_y_y,
      d_r_z, d_p_z, d_y_z;

  return (measured_position_covariance + rpy_cov * origen_orientation_covariance * rpy_cov.transpose());
}

Eigen::Matrix3d linearVelocityCov(const Eigen::Matrix3d measured_linear_velocity_covariance,
                                  const Eigen::Matrix3d origin_angular_velocity_covariance,
                                  const Eigen::Quaterniond rotation_from_origen,
                                  const Eigen::Vector3d translation_from_origen)
{
  //    Z = RPY*Measured_v - cross(w, trans)
  //
  //    Take first RPY*M_v
  //    Z'vx = cy*cp*vx + (cy*sp*sr - sy*cr)*vy + (cy*sp*cr + sy*sr)*vz
  //    Z'vy = sy*cp*vx + (sy*sp*sr + cy*cr)*vy + (sy*sp*cr - cy*sr)*vz
  //    Z'vz = -sp*vx + cp*sr*vy + cp*cr*vz
  //    This is a linera operation because the covariance is in M_v then:

  Eigen::Matrix3d RPY_x_M_v = rotation_from_origen.toRotationMatrix() *
      measured_linear_velocity_covariance *
      rotation_from_origen.toRotationMatrix().transpose();

  //    Take second cross(W, T)
  //    x = wy*tz - wz*ty
  //    y = - wx*tz + wz*tx
  //    z = wx*ty - wy*tx
  //    This is also linear

  double d_wx_x = 0;
  double d_wx_y = -translation_from_origen(2);
  double d_wx_z = translation_from_origen(1);
  double d_wy_x = translation_from_origen(2);
  double d_wy_y = 0;
  double d_wy_z = -translation_from_origen(0);
  double d_wz_x = -translation_from_origen(1);
  double d_wz_y = translation_from_origen(0);
  double d_wz_z = 0;

  Eigen::Matrix3d w_cov;
  w_cov << d_wx_x, d_wy_x, d_wz_x,
      d_wx_y, d_wy_y, d_wz_y,
      d_wx_z, d_wy_z, d_wz_z;

  return RPY_x_M_v + w_cov * origin_angular_velocity_covariance * w_cov.transpose();
}

Eigen::Matrix3d rotatedCovariance(const Eigen::Matrix3d covariance,
                                  const Eigen::Quaterniond rotation)
{
  //    Z = RPY*W
  //    Zwx = cy*cp*wx + (cy*sp*sr - sy*cr)*wy + (cy*sp*cr + sy*sr)*wz
  //    Zwy = sy*cp*wx + (sy*sp*sr + cy*cr)*wy + (sy*sp*cr - cy*sr)*wz
  //    Zwz = -sp*wx + cp*sr*wy + cp*cr*wz
  //   As the covariance is in the W the function is linear

  return rotation.toRotationMatrix() * covariance * rotation.toRotationMatrix().transpose();
}

}  // namespace transformations
