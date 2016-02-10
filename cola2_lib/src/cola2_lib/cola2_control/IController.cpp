#include "cola2_lib/cola2_control/IController.h"

IController::IController(std::string name):
  _name(name)
{}

void IController::reset()
{}

double IController::compute(double time_in_sec, double setpoint, double feedback)
{
  return 0.0;
}

bool IController::setParameters(std::map< std::string, double > params)
{
  return false;
}

double _saturateValue(double value, double limit)
{
  if (value > limit) value = limit;
  if (value < -limit) value = -limit;
  return value;
}

double _normalizeAngle(const double angle)
{
  return (angle + (2.0 * __PI__ * floor((__PI__ - angle) / (2.0 * __PI__))));
}
