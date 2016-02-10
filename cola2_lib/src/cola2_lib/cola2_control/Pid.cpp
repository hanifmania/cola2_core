#include "cola2_lib/cola2_control/Pid.h"

Pid::Pid(std::string name):
  IController(name),
  _kp(0),
  _ti(0),
  _td(0),
  _i_limit(0),
  _fff(0),
  _time_old(0),
  _feedback_old(0),
  _error_old(0),
  _eik_old(0),
  _derivative_term_from_feedback(true),
  _edotk_old(0)
{}

void Pid::reset()
{
  _eik_old = 0.0;
  _time_old = 0;
}

double Pid::compute(double time_in_sec, double setpoint, double feedback)
{
  // std::cout << "Compute " << _name << ", time: " << time_in_sec << ", setpoint: " << setpoint << ", feedback: "
  // << feedback << std::endl;

  // Check time
  if (_time_old == 0)
  {
    // Fist time or just reset
    _time_old = time_in_sec;
    _feedback_old = feedback;
    _error_old = setpoint - feedback;
    _edotk_old = 0.0;
    return 0.0;
  }

  double dt = time_in_sec - _time_old;
  if (dt > 0.2)
  {
    // To much time without controlling this DoF, reset it!
    std::cout << "ERROR wit PID " << _name << " dt = " << dt << "\n";
    reset();
    return 0.0;
  }

  if (dt < 0.01) dt = 0.01;
  _time_old = time_in_sec;
  // std::cout << "PID " << _name << " dt: " << dt << std::endl;

  // Compute error, derivative of feedback and integral part
  double ek = setpoint - feedback;
  // std::cout << "PID "  << _name << " ek: " << ek << std::endl;

  double edotk = 0.0;
  if (_derivative_term_from_feedback)
  {
    edotk = (feedback - _feedback_old) / dt;
  }
  else
  {
    edotk = (ek - _error_old) / dt;
  }

  // std::cout << "edotk: " << edotk << "\n";
  // std::cout << "tD: " << _td << "\n";
  // std::cout << "part derivativa: " << _kp * _td * edotk << "\n";

  // edotk = (edotk + _edotk_old) / 2.0;
  // std::cout << "edotk filtered: " << edotk << "\n";

  double eik = _eik_old + (ek * dt);

  // Compute the integral part if ti > 0
  double integral_part = 0.0;
  double tau = 0;
  if (_ti > 0.0)
  {
    // Integral part
    integral_part = (_kp / _ti) * eik;

    // Saturate integral part
    // (anti-windup condition, integral part not higher than a value)
    integral_part = _saturateValue(integral_part, _i_limit);

    // Restore eik
    if (_kp > 0.0)
    {
      // Avoid division by zero
      eik = integral_part * _ti / _kp;
    }
    else
    {
      eik = 0.0;
    }
    // Compute tau
    tau = _kp * (ek - _td * edotk) + integral_part + _fff;
  }
  else
  {
    // Compute tau without integral part
    tau = _kp * (ek - _td * edotk) + _fff;
  }
  // std::cout << "PID " << _name << " tau: " << tau << std::endl;

  // Store for the next time
  _feedback_old = feedback;
  _error_old = ek;
  _eik_old = eik;
  _edotk_old = edotk;

  // Return saturate tau
  return _saturateValue(tau, 1.0);
}

bool Pid::setParameters(std::map<std::string, double> params)
{
  std::cout << "Set params for " << _name << " as: " << params["kp"] << ", " << params["ti"] << ", " << params["td"]
            << ", " << params["derivative_term_from_feedback"] << "\n";

  try
  {
    _kp = params["kp"];
    _ti = params["ti"];
    _td = params["td"];
    _i_limit = params["i_limit"];
    _fff = params["fff"];
    _derivative_term_from_feedback = static_cast<bool>(params["derivative_term_from_feedback"]);
  }
  catch (...)
  {
    std::cout << "PID " << _name << " setting parameters ERROR! \n";
    return false;
  }
  return true;
}
