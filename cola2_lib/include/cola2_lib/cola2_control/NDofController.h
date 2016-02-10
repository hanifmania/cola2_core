#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_

#include <assert.h>
#include <map>
#include <string>
#include <vector>

#include "./IController.h"
#include "./Request.h"

class NDofController
{
 public:
  NDofController(const unsigned int n_dof = 6);

  void addController(IController *controller);

  void setControllerParams(std::vector< std::map<std::string, double> > params);

  void reset();

  std::vector<double> compute(double time_in_sec, Request req, std::vector<double> feedback);

 private:
  std::vector<IController*> _controllers;
  unsigned int _n_dof;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_
