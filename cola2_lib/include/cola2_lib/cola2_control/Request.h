
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_REQUEST_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_REQUEST_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

class Request
{
 public:
  Request(std::string requester = "", const double stamp = 0.0,
          const unsigned int priority = 0, const unsigned int n_dof = 6);
  Request(const Request& other);

  void combineRequest(const Request& req, std::string mode);
  void combineSamePriorityRequest(const Request& req, std::string mode);
  void combineLowerPriorityRequest(const Request& lower);
  void combineRequest(const Request& req);

  bool matchDisabledAxis(std::vector<bool> axis);

  void setDisabledAxis(std::vector<bool> values);
  std::vector<bool> getDisabledAxis() const;

  void setValues(std::vector< double > values);
  std::vector< double > getValues() const;

  void setRequester(std::string name);
  std::string getRequester() const;

  double getStamp();

  unsigned int getPriority() const;
  void setPriority(const unsigned int priority);

  /*
     * If at least one axis is not disabled return false, otherwise true.
     */
  bool isAllDisabled() const;

  bool operator<(const Request& rhs) const;
  bool operator==(const Request& rhs) const;
  friend std::ostream& operator<<(std::ostream &out, Request r);

 private:
  std::string _requester;
  double _stamp;
  std::vector<bool> _disabled_axis;
  std::vector<double> _values;
  unsigned int _priority;
  unsigned int _n_dof;
};

//std::ostream& operator<<(std::ostream& out, Request r);

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_REQUEST_H_
