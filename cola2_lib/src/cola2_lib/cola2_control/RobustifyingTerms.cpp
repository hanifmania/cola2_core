#include "cola2_lib/cola2_control/RobustifyingTerms.h"

RT::RT()
{
  nu = 0.01;
  nv = 0.01;
  nw = 0.01;
  nq = 0.01;
  nr = 0.01;
}

std::vector<double> RT::compute(const std::vector<double> feedback, Request setpoint)
{
  float du_des = 0.0;
  float dv_des = 0.0;
  float dw_des = 0.0;
  float dp_des = 0.0;
  float dq_des = 0.0;
  float dr_des = 0.0;

  float u = feedback.at(0);
  float v = feedback.at(1);
  float w = feedback.at(2);
  float p = feedback.at(3);
  float q = feedback.at(4);
  float r = feedback.at(5);

  float e_u = u - setpoint.getValues().at(0);
  float e_v = v - setpoint.getValues().at(1);
  float e_w = w - setpoint.getValues().at(2);
  float e_r = r - setpoint.getValues().at(5);

  std::vector< double > ret;
  float X = - nu*e_u*(pow((v*r), 2)+pow((w*q), 2)+pow((p*r), 2)+1.0+pow(u, 2)+pow(u, 4)+pow(du_des, 2)+pow(dq_des, 2));
  if (!setpoint.getDisabledAxis().at(0))
  {
    ret.push_back(X);
  }
  else
  {
    ret.push_back(0.0);
  }

  float Y = - nv*e_v*(pow((u*r), 2)+pow((w*p), 2)+pow((q*r), 2)+1.0+pow(v, 2)+pow(v, 4)+pow(dv_des, 2)+pow(dp_des, 2));
  if (!setpoint.getDisabledAxis().at(1))
  {
    ret.push_back(Y);
  }
  else
  {
    ret.push_back(0.0);
  }

  float Z = - nw*e_w*(pow((u*q), 2)+pow((v*p), 2)+pow(p, 4)+pow(q, 4)+1.0+pow(w, 2)+pow(w, 4)+pow(dw_des, 2));
  if (!setpoint.getDisabledAxis().at(2))
  {
    ret.push_back(Z);
  }
  else
  {
    ret.push_back(0.0);
  }

  ret.push_back(0.0);
  ret.push_back(0.0);

  float N = - nr*e_r*(pow(r, 2)+pow(r, 4)+pow(dr_des, 2));
  if (!setpoint.getDisabledAxis().at(5))
  {
    ret.push_back(N);
  }
  else
  {
    ret.push_back(0.0);
  }

  return ret;
}
