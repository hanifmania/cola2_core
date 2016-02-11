#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ICONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ICONTROLLER_H_

#include <math.h>

#include <iostream>
#include <map>
#include <string>

#define __PI__ 3.141592653

class IController
{
 public:
  IController(std::string name);

  virtual void reset();
  virtual double compute(double time_in_sec, double setpoint, double feedback);
  virtual bool setParameters(std::map< std::string, double > params);

 protected:
  std::string _name;
};

double _saturateValue(double value, double limit);
double _normalizeAngle(const double angle);

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ICONTROLLER_H_
