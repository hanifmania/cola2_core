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

template<typename T>
void getParam(const std::string param_name, T& param_var, T default_value) {
    // Display a message if a parameter is not found in the param server
    if (!ros::param::getCached(param_name, param_var)) {
        ROS_WARN_STREAM("Value for parameter " <<
            param_name << " not found in param server! Using default value " <<
            default_value);
            param_var = default_value;
    }
}

template <typename ParamType>
bool loadVector(const std::string param_name,
                std::vector<ParamType> &data)
{
    // Take the vector param and copy to a std::vector<double>
    XmlRpc::XmlRpcValue my_list ;
    if (ros::param::getCached(param_name, my_list)) {
        ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray) ;

        for (int32_t i = 0; i < my_list.size(); ++i) {
            // ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) ;
            data.push_back(static_cast<ParamType>(my_list[i])) ;
        }
    }
    else {
        ROS_FATAL_STREAM("Invalid parameters for " << param_name << " in param server!");
        return false;
    }
    return true;
}

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
