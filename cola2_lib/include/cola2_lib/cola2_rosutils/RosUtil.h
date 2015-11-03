/*
 * cola2_ros_util.h
 *
 *  Created on: 20/3/2013
 *      Author: Narcis Palomeras
 */



#ifndef ROSUTIL_H_
#define ROSUTIL_H_

#include "ros/ros.h"
#include <cola2_lib/cola2_io/SerialPort.h>
#include "cola2_lib/cola2_rosutils/DiagnosticHelper.h"
#include "cola2_safety/DigitalOutput.h"

namespace cola2{
	namespace rosutil{

struct SpConfig {
	// ------- Serial Port Config -------
	std::string sp_path ;
	int sp_baud_rate ;
	int sp_char_size ;
	int sp_stop_bits ;
	std::string sp_parity ;
	std::string sp_flow_control ;
	int sp_timeout ;
};


void
loadParam( std::string param_name, bool& value ) {
    if (!ros::param::getCached(param_name, value)) {
        ROS_FATAL( "Invalid parameters for bool %s in param server!", param_name.c_str() ) ;
        ros::shutdown();
    }
}

void
loadParam( std::string param_name, int& value ) {
    if (!ros::param::getCached(param_name, value)) {
        ROS_FATAL( "Invalid parameters for int %s in param server!", param_name.c_str() ) ;
        ros::shutdown();
    }
}

void
loadParam( std::string param_name, double& value ) {
    if (!ros::param::getCached(param_name, value)) {
        ROS_FATAL( "Invalid parameters for double %s in param server!", param_name.c_str() ) ;
        ros::shutdown();
    }
}

void
loadParam( std::string param_name, std::string& value ) {
    if (!ros::param::getCached(param_name, value)) {
        ROS_FATAL( "Invalid parameters for string %s in param server!", param_name.c_str() ) ;
        ros::shutdown();
    }
}

void
loadParam( std::string param_name, std::vector<double>& vector ) {
    XmlRpc::XmlRpcValue my_list;
    if ( ros::param::getCached( param_name, my_list ) ) {
        ROS_ASSERT( my_list.getType() == XmlRpc::XmlRpcValue::TypeArray );
        for ( int32_t i = 0; i < my_list.size(); ++i ) {
            ROS_ASSERT( my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble );
            vector.push_back( static_cast<double>( my_list[i] ) );
        }
    }
    else {
        ROS_FATAL( "Invalid parameters for vector %s in param server!\nShutdown node.", param_name.c_str() );
        ros::shutdown();
    }
}

void
loadParam( std::string param_name, std::vector<std::string>& vector ) {
    XmlRpc::XmlRpcValue my_list;
    if ( ros::param::getCached( param_name, my_list ) ) {
        ROS_ASSERT( my_list.getType() == XmlRpc::XmlRpcValue::TypeArray );
        for ( int32_t i = 0; i < my_list.size(); ++i ) {
            ROS_ASSERT( my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString );
            vector.push_back( static_cast<std::string>( my_list[i] ) );
        }
    }
    else {
        ROS_FATAL( "Invalid parameters for string vector %s in param server!\nShutdown node.", param_name.c_str() );
        ros::shutdown();
    }
}

bool
initSensor(const std::string name,
	   cola2::rosutils::DiagnosticHelper& diagnostic,
	   ros::NodeHandle& n,
	   int digital_output,
	   float time_out = 0.0) 
{
	//Set up diagnostics
	diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::WARN, "Initializing...");
	diagnostic.publish();
	
	//Call service to enable valeport digital output
	if(digital_output >= 0){
		ros::Duration(5.0).sleep();
		ROS_INFO("[%s]: Waiting for service digital_out", name.c_str());
		if(ros::service::waitForService("digital_output", 5)){
			ros::ServiceClient client = n.serviceClient<cola2_safety::DigitalOutput>("digital_output");
			cola2_safety::DigitalOutput srv;
			srv.request.digital_out = digital_output;
			srv.request.value = true;
			ROS_INFO("[%s]: Call service digital_out", name.c_str());
			if (!client.call(srv)) {
				ROS_ERROR("[%s] Failed to call service digital_output", name.c_str());
				diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::ERROR, "Impossible to enable digital_output");
				diagnostic.publish();
				return false;
			}
		}
		else {
			ROS_ERROR("[%s] Failed to call service digital_output", name.c_str());
			diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::ERROR, "Impossible to enable digital_output");
			diagnostic.publish();
			return false;
		}
	}

	if(time_out > 0){
		//Wait sensor to boot
		ROS_INFO("[%s]: Wait until sensor starts", name.c_str());
		ros::Duration(time_out).sleep();
	}
	return true;
}


bool setUpSerialPort(cola2::io::SerialPort& serial_port,
		     const cola2::rosutil::SpConfig& sp_config,
		     cola2::rosutils::DiagnosticHelper& diagnostic)
{
	try {
		serial_port.open(sp_config.sp_path);
		serial_port.setBaudRate(cola2::io::SerialPort::baudRateFromInteger(sp_config.sp_baud_rate));
		serial_port.setCharSize(cola2::io::SerialPort::charSizeFromInteger(sp_config.sp_char_size));
		serial_port.setNumOfStopBits(cola2::io::SerialPort::numOfStopBitsFromInteger(sp_config.sp_stop_bits));
		serial_port.setParity(cola2::io::SerialPort::parityFromString(sp_config.sp_parity));
		serial_port.setFlowControl(cola2::io::SerialPort::flowControlFromString(sp_config.sp_flow_control));
	}
	catch( std::exception& e) {
		diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::ERROR, "Error setting up the serial port!");
		diagnostic.publish();
		ROS_ERROR("Error setting up the serial port!");
		return false;
	}
	return true;
}

}; //rosutil
}; //cola2

#endif // ROSUTIL_H_

