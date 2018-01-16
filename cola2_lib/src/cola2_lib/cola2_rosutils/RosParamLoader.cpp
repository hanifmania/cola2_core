
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

#include "cola2_lib/cola2_rosutils/RosParamLoader.h"

namespace cola2 {
namespace rosutils {

RosParamLoader::RosParamLoader():
  integers_(),
  reals_(),
  strings_(),
  booleans_(),
  itemNames_()
{
}

void RosParamLoader::loadParams()
throw(std::runtime_error)
{
  for (IntegersMap::const_iterator i = integers_.begin(); i != integers_.end(); ++i)
  {
    loadParam(i->first, *(i->second));
  }
  for (RealsMap::const_iterator i = reals_.begin(); i != reals_.end(); ++i)
  {
    loadParam(i->first, *(i->second));
  }
  for (StringsMap::const_iterator i = strings_.begin(); i != strings_.end(); ++i )
  {
    loadParam(i->first, *(i->second));
  }
  for (BooleansMap::const_iterator i = booleans_.begin(); i != booleans_.end(); ++i)
  {
    loadParam(i->first, *(i->second));
  }
}

void RosParamLoader::mapParam(const std::string& name, int* value)
throw(std::runtime_error)
{
  mapParam(name, value, integers_);
}

void RosParamLoader::mapParam(const std::string& name, double* value)
throw(std::runtime_error)
{
  mapParam(name, value, reals_);
}

void RosParamLoader::mapParam(const std::string& name, std::string* value)
throw(std::runtime_error)
{
  mapParam(name, value, strings_);
}

void RosParamLoader::mapParam(const std::string& name, bool* value)
throw(std::runtime_error)
{
  mapParam(name, value, booleans_);
}

template <typename ValueType, typename ContainerType>
void RosParamLoader::mapParam(const std::string& name, ValueType value, ContainerType& container)
throw(std::runtime_error)
{
  if (std::find( itemNames_.begin(), itemNames_.end(), name ) != itemNames_.end())
    throw std::runtime_error("Name already used: " + name);
  itemNames_.push_back(name);
  container.insert(std::make_pair(name, value));
}

template <typename ParamType>
void RosParamLoader::loadParam(const std::string& name, ParamType& value)
{
  if (ros::param::get(name, value))
    ROS_INFO("Got param: %s", name.c_str());
  else
    ROS_FATAL("Couldn't get '%s' param.", name.c_str());
}

}  // namespace rosutils
}  // namespace cola2
