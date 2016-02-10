#include "cola2_lib/cola2_control/Model.h"

Model::Model()
{
  Mu = 126.5000;
  Mv = 273.8433;
  Mw = 213.0535;
  Mp = 55.0000;
  Mr = 67.8951;
  zB = -0.02;
  W  = 532.5500;
  B  = 550.1106;
  Xuu = -31.8086;
  Yvv = -222.8960;
  Zww = -263.4225;
  Kpp = -0.0000;
  Mqq = -40.5265;
  Nrr = -40.5265;
  Xu = -2.0000;
  Yv = -23.0000;
  Zw = -0.0000;
  Kp = -10.0000;
  Mq = -0.0000;
  // Mq = 67.8951;
  Nr = -0.0000;
}

std::vector<double> Model::compute(const std::vector<double> twist_feedback, const std::vector<double> pose_feedback)
{
  float u = twist_feedback.at(0);
  float v = twist_feedback.at(1);
  float w = twist_feedback.at(2);
  float p = twist_feedback.at(3);
  float q = twist_feedback.at(4);
  float r = twist_feedback.at(5);
  float _phi = pose_feedback.at(3);
  float _theta = pose_feedback.at(4);

  std::vector< double > ret;
  float X_model = Mw*w*q-Mv*v*r+(W-B)*sin(_theta)-Xu*u-Xuu*u*std::abs(u);
  ret.push_back(X_model);
  float Y_model = -Mw*w*p+Mu*u*r-(W-B)*cos(_theta)*sin(_phi)-Yv*v-Yvv*v*std::abs(v);
  ret.push_back(Y_model);
  float Z_model = Mv*v*p-Mu*u*q-(W-B)*cos(_theta)*cos(_phi)-Zw*w-Zww*w*std::abs(w);
  ret.push_back(Z_model);
  ret.push_back(0.0);
  float M_model = -zB*B*sin(_theta);
  ret.push_back(M_model);
  float N_model = Mv*v*u-Mu*u*v+Mq*q*p-Mp*p*q-Nr*r-Nrr*r*std::abs(r);
  ret.push_back(N_model);

  return ret;
}
