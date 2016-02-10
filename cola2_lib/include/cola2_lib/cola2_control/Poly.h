#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_POLY_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_POLY_H_

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "./IController.h"

class Poly: public IController
{
 public:
    Poly(std::string name);

    void reset();
    double compute(double time_in_sec, double setpoint, double feedback);
    bool setParameters(std::map<std::string, double> params);

 private:
    std::vector<double> _setpoint_coefs;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_POLY_H_
