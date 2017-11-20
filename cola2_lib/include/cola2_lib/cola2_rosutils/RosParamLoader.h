
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*
 * RosParamLoader.h
 *
 *  Created on: Jan 24, 2013
 *      Author: Enric Galceran
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_ROSPARAMLOADER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_ROSPARAMLOADER_H_

#include <ros/ros.h>

#include <list>
#include <map>
#include <stdexcept>
#include <string>

namespace cola2 {
namespace rosutils {

class RosParamLoader
{
 public:
  RosParamLoader();

  void loadParams() throw(std::runtime_error);

  void mapParam(const std::string& name, int* value) throw(std::runtime_error);
  void mapParam(const std::string& name, double* value) throw(std::runtime_error);
  void mapParam(const std::string& name, std::string* value) throw(std::runtime_error);
  void mapParam(const std::string& name, bool* value) throw(std::runtime_error);

 private:
  typedef std::map<std::string, int*> IntegersMap;
  typedef std::map<std::string, double*> RealsMap;
  typedef std::map<std::string, std::string*> StringsMap;
  typedef std::map<std::string, bool*> BooleansMap;

  IntegersMap integers_;
  RealsMap reals_;
  StringsMap strings_;
  BooleansMap booleans_;

  std::list<std::string> itemNames_;

  template <typename ValueType, typename ContainerType>
  void mapParam(const std::string& name, ValueType value, ContainerType& container) throw(std::runtime_error);

  template <typename ParamType>
  void loadParam(const std::string& name, ParamType& value);
};

}  // namespace rosutils
}  // namespace cola2


#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_ROSPARAMLOADER_H_

