/*
 * cola2_ros_util.h
 *
 *  Created on: 20/3/2013
 *      Author: Narcis Palomeras
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_ROSUTIL_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_ROSUTIL_H_

#include <ros/ros.h>
#include <cola2_msgs/DigitalOutput.h>

#include <string>
#include <vector>

#include "../cola2_io/SerialPort.h"
#include "./DiagnosticHelper.h"

namespace cola2 {
namespace rosutil {

struct SpConfig
{
  // ------- Serial Port Config -------
  std::string sp_path;
  int sp_baud_rate;
  int sp_char_size;
  int sp_stop_bits;
  std::string sp_parity;
  std::string sp_flow_control;
  int sp_timeout;
};

void loadParam(std::string param_name, bool& value);
void loadParam(std::string param_name, int& value);
void loadParam(std::string param_name, double& value);
void loadParam(std::string param_name, std::string& value);
void loadParam(std::string param_name, std::vector<double>& vector);
void loadParam(std::string param_name, std::vector<std::string>& vector);

bool initSensor(const std::string name, cola2::rosutils::DiagnosticHelper& diagnostic, ros::NodeHandle& n,
                int digital_output, float time_out = 0.0);

bool setUpSerialPort(cola2::io::SerialPort& serial_port, const cola2::rosutil::SpConfig& sp_config,
                     cola2::rosutils::DiagnosticHelper& diagnostic);

}  // namespace rosutil
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_ROSUTILS_ROSUTIL_H_

