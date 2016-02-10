/*
 * diagnostic_helper.cpp
 *
 * Created on: March 20, 2013
 *     Author: Narcis Palomeras
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_DIAGNOSTICHELPER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_DIAGNOSTICHELPER_H_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <string>
#include <vector>

namespace cola2 {
namespace rosutils {

class DiagnosticHelper
{
 public:
  DiagnosticHelper(ros::NodeHandle& n, const std::string name, const std::string hardware_id);

  void setLevel(const int level, const std::string message = "none");

  void add(const std::string key, const int value);
  void add(const std::string key, const double value);
  void add(const std::string key, const std::string value);

  void del(const std::string key);

  void publish();

  void check_frequency();
  double getCurrentFreq();
  unsigned int getTimesInWarning();

 private:
  // Diagnostic msg
  diagnostic_msgs::DiagnosticStatus _diagnostic;

  // ROS Publisher
  ros::Publisher _diagnostic_pub;

  // Check frequency
  unsigned int _counter;
  double _last_check_freq;
  double _current_freq;

  // Check time in warning
  unsigned int _times_in_warning;
};  // DiagnosticHelper

}  // namespace rosutils
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_DIAGNOSTICHELPER_H_
