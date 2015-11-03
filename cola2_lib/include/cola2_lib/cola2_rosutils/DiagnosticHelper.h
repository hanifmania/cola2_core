/*
 * diagnostic_helper.cpp
 * 
 * Created on: March 20, 2013
 *     Author: Narcis Palomeras
 */

#ifndef DIAGNOSTIC_HELPER_H_
#define DIAGNOSTIC_HELPER_H_

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"

namespace cola2 {
    namespace rosutils {
    
class DiagnosticHelper{
public:
	DiagnosticHelper(ros::NodeHandle& n, 
			 const std::string name, 
             const std::string hardware_id):
        _counter( 0 ),
        _current_freq( 0.0 ),
        _times_in_warning( 0 )
	{
		_diagnostic_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
		_diagnostic.name = name;
		_diagnostic.hardware_id = hardware_id;
        _last_check_freq = ros::Time().now().toSec();
	}
	
	
	void 
	setLevel(const int level, 
		 const std::string message = "none")
	{
		_diagnostic.level = level;
		if(message.compare("none") == 0){
			if(level == diagnostic_msgs::DiagnosticStatus::OK)
				_diagnostic.message = "Ok";
			else if (level == diagnostic_msgs::DiagnosticStatus::WARN)
				_diagnostic.message = "Warning";
			else 
				_diagnostic.message = "Error";
		}
		else {
			_diagnostic.message = message;
		}
		
		// Publish diagnostic message
		publish();
		
	}	

	
	void 
	add(const std::string key,
	    const int value)
	{
		std::stringstream ss;
		ss << value;
		add(key, ss.str());
	}
	
	
	void 
	add(const std::string key,
	    const double value)
	{
		std::stringstream ss;
		ss << value;
		add(key, ss.str());
	}
	
	
	void 
	add(const std::string key,
	    const std::string value)
	{
		// Create KeyValue type
		diagnostic_msgs::KeyValue key_value;
		key_value.key = key;
		key_value.value = value;
				
		// Search for key
		std::vector<diagnostic_msgs::KeyValue>::iterator it = _diagnostic.values.begin();
		bool found = false; 
		while(it != _diagnostic.values.end() && !found){
			if(it->key.compare(key) == 0)
				found = true;
			else
				it++;
		}
		
		// If key already found in 'values' delete it
		if(it != _diagnostic.values.end()){
			_diagnostic.values.erase(it);
		}
		
		// Add Key Value
		_diagnostic.values.push_back(key_value);
	}


	void
	del(const std::string key)
	{
		// Search for key
		std::vector<diagnostic_msgs::KeyValue>::iterator it = _diagnostic.values.begin();
		bool found = false; 
		while(it != _diagnostic.values.end() && !found){
			if(it->key.compare(key) == 0)
				found = true;
			it++;
		}
		
		// If key already found in 'values' delete it
		if(it != _diagnostic.values.end()){
			_diagnostic.values.erase(it);
		}
	}


	void
	publish()
	{
        // Compute frequency
        if( _counter > 0 ){
            if( ros::Time().now().toSec() - _last_check_freq > 3.0 ) {
                _current_freq =  _counter / ( ros::Time().now().toSec() - _last_check_freq );
                add( "frequency", _current_freq);
                _counter = 0;
                _last_check_freq = ros::Time().now().toSec();
            }
        }

        //Published as a warning
        if( _diagnostic.level != diagnostic_msgs::DiagnosticStatus::OK ) {
            _times_in_warning++;
        }
        else {
            _times_in_warning = 0;
        }

		//publish diagnostic
		diagnostic_msgs::DiagnosticArray diagnostic_array;
		diagnostic_array.header.stamp = ros::Time::now() ;
		diagnostic_array.header.frame_id = _diagnostic.name;
		diagnostic_array.status.push_back(_diagnostic);
		_diagnostic_pub.publish(diagnostic_array);
	}

    void
    check_frequency()
    {
        /*  This method computes the frequency in which this method is called. */
        _counter++;
    }

    double
    getCurrentFreq()
    {
        return _current_freq;
    }

    unsigned int
    getTimesInWarning()
    {
        return _times_in_warning;
    }


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

}; //DiagnosticHelper
}; // namespace ros
}; // namespace cola2

#endif /*DIAGNOSTIC_HELPER_H_*/

/*
int
main(int argc, char **argv)
{
	// Init ROS node
	ros::init(argc, argv, "test_diagnostic_helper", ros::init_options::NoSigintHandler);
	
	// Node handle
	ros::NodeHandle n;

	// Create DiagnosticHelper instance
	DiagnosticHelper diagnostic_helper = DiagnosticHelper(n, "test_diagnostic_helper", "none");

	// Set level and message
	diagnostic_helper.setLevel(diagnostic_msgs::DiagnosticStatus::OK);

	// Add information pairs
	diagnostic_helper.add("test_1", "1");
	diagnostic_helper.add("test_2", "2");


	//Publish
	for(int i = 0; i < 5; i++) {
		diagnostic_helper.publish();
		ros::Duration(3.0).sleep();
		diagnostic_helper.add("test_1", "3");
	}

}
*/

