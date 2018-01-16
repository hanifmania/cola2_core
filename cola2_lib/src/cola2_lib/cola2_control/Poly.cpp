
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/cola2_control/Poly.h"

Poly::Poly(std::string name):
  IController(name),
  _setpoint_coefs()
{
}

void Poly::reset()
{
}

double Poly::compute(double time_in_sec, double setpoint, double feedback)
{
  double tau = 0.0;
  bool negative_setpoint = false;
  if (setpoint < 0) negative_setpoint = true;

  for (unsigned int i = 0; i < _setpoint_coefs.size(); i++)
  {
    tau = tau + pow(static_cast<double>(fabs(setpoint)), static_cast<double>(i)) * _setpoint_coefs.at(i);
  }
  if (negative_setpoint) tau = -1.0 * tau;

  // Return non-saturated tau
  return tau;
}

bool Poly::setParameters(std::map<std::string, double> params)
{
  std::cout << "Set params for " << _name << ": " << static_cast<int>(params["1"]) << "\n";
  _setpoint_coefs.clear();
  try
  {
    unsigned int n_dof = static_cast<int>(params["n_dof"]);
    // std::cout << "poly n_dof: " << n_dof << "\n";
    for (unsigned int i = 0; i < n_dof; i++)
    {
      std::ostringstream s;
      s << i;
      const std::string i_as_string(s.str());
      _setpoint_coefs.push_back(static_cast<double>(params[i_as_string]));
      // std::cout << "add param: " << double( params[i_as_string] ) << "\n";
    }
  }
  catch (...)
  {
    return false;
  }
  // std::cout << "Poly initialized!\n";
  return true;
}
