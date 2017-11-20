
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*
 * diagnostic_helper.cpp
 *
 * Created on: March 20, 2013
 *     Author: Narcis Palomeras
 */

#include "cola2_lib/cola2_rosutils/DiagnosticHelper.h"

namespace cola2 {
namespace rosutils {

DiagnosticHelper::DiagnosticHelper(ros::NodeHandle& n, const std::string name, const std::string hardware_id):
  _counter(0),
  _current_freq(0.0),
  _times_in_warning(0)
{
  _diagnostic_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  _diagnostic.name = name;
  _diagnostic.hardware_id = hardware_id;
  _last_check_freq = ros::Time().now().toSec();
}

void DiagnosticHelper::setLevel(const int level, const std::string message)
{
  _diagnostic.level = level;
  if (message.compare("none") == 0)
  {
    if (level == diagnostic_msgs::DiagnosticStatus::OK)
      _diagnostic.message = "Ok";
    else if (level == diagnostic_msgs::DiagnosticStatus::WARN)
      _diagnostic.message = "Warning";
    else
      _diagnostic.message = "Error";
  }
  else
  {
    _diagnostic.message = message;
  }
  // Publish diagnostic message
  publish();
}

void DiagnosticHelper::add(const std::string key, const int value)
{
  std::stringstream ss;
  ss << value;
  add(key, ss.str());
}

void DiagnosticHelper::add(const std::string key, const double value)
{
  std::stringstream ss;
  ss << value;
  add(key, ss.str());
}

void DiagnosticHelper::add(const std::string key, const std::string value)
{
  // Create KeyValue type
  diagnostic_msgs::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;

  // Search for key
  std::vector<diagnostic_msgs::KeyValue>::iterator it = _diagnostic.values.begin();
  bool found = false;
  while (it != _diagnostic.values.end() && !found)
  {
    if (it->key.compare(key) == 0)
      found = true;
    else
      it++;
  }

  // If key already found in 'values' delete it
  if (it != _diagnostic.values.end())
  {
    _diagnostic.values.erase(it);
  }

  // Add Key Value
  _diagnostic.values.push_back(key_value);
}

void DiagnosticHelper::del(const std::string key)
{
  // Search for key
  std::vector<diagnostic_msgs::KeyValue>::iterator it = _diagnostic.values.begin();
  bool found = false;
  while (it != _diagnostic.values.end() && !found)
  {
    if (it->key.compare(key) == 0)
      found = true;
    it++;
  }

  // If key already found in 'values' delete it
  if (it != _diagnostic.values.end())
  {
    _diagnostic.values.erase(it);
  }
}

void DiagnosticHelper::publish()
{
  // Compute frequency
  if (_counter > 0)
  {
    if (ros::Time().now().toSec() - _last_check_freq > 3.0)
    {
      _current_freq =  _counter / (ros::Time().now().toSec() - _last_check_freq);
      add("frequency", _current_freq);
      _counter = 0;
      _last_check_freq = ros::Time().now().toSec();
    }
  }

  // Published as a warning
  if (_diagnostic.level != diagnostic_msgs::DiagnosticStatus::OK)
  {
    _times_in_warning++;
  }
  else
  {
    _times_in_warning = 0;
  }

  // Publish diagnostic
  diagnostic_msgs::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = ros::Time::now();
  diagnostic_array.header.frame_id = _diagnostic.name;
  diagnostic_array.status.push_back(_diagnostic);
  _diagnostic_pub.publish(diagnostic_array);
}

void DiagnosticHelper::check_frequency()
{
  /*  This method computes the frequency in which this method is called. */
  _counter++;
}

double DiagnosticHelper::getCurrentFreq()
{
  return _current_freq;
}

unsigned int DiagnosticHelper::getTimesInWarning()
{
  return _times_in_warning;
}

}  // namespace rosutils
}  // namespace cola2
