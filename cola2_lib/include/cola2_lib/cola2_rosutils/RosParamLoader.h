/*
 * RosParamLoader.h
 *
 *  Created on: Jan 24, 2013
 *      Author: Enric Galceran
 */

#ifndef ROS_PARAM_LOADER_H_
#define ROS_PARAM_LOADER_H_

#include <string>
#include <stdexcept>
#include <map>
#include "ros/ros.h"

namespace cola2 {
    namespace rosutils {

class RosParamLoader {

public:

	RosParamLoader():
		integers_(),
		reals_(),
		strings_(),
		booleans_(),
		//vectors_(),
		itemNames_()
	{ }



	~RosParamLoader()
	{ }



	void
	loadParams()
		throw ( std::runtime_error )
	{
		for ( IntegersMap::const_iterator i = integers_.begin(); i != integers_.end(); ++i ) {
			loadParam(i->first, *(i->second));
		}
		for ( RealsMap::const_iterator i = reals_.begin(); i != reals_.end(); ++i ) {
			loadParam(i->first, *(i->second));
		}
		for ( StringsMap::const_iterator i = strings_.begin(); i != strings_.end(); ++i ) {
			loadParam(i->first, *(i->second));
		}
		for ( BooleansMap::const_iterator i = booleans_.begin(); i != booleans_.end(); ++i ) {
			loadParam(i->first, *(i->second));
		}
		/*
		for ( VectorsMap::const_iterator i = vectors_.begin(); i != vectors_.end(); ++i ) {
			loadParam(i->first, *(i->second));
		}
		*/		
	}



	void
	mapParam( const std::string& name,
			 int* value )
		throw ( std::runtime_error )
	{
		mapParam( name, value, integers_ ) ;
	}



	void
	mapParam( const std::string& name,
			 double* value )
		throw ( std::runtime_error )
	{
		mapParam( name, value, reals_ ) ;
	}



	void
	mapParam( const std::string& name,
			 std::string* value )
		throw ( std::runtime_error )
	{
		mapParam( name, value, strings_ ) ;
	}



	/*void
	mapParam( const std::string& name,
			 std::vector<double>* value )
		throw ( std::runtime_error )
	{
		mapParam( name, value, vectors_ ) ;
	}*/
	
	

	void
	mapParam( const std::string& name,
			 bool* value )
		throw ( std::runtime_error )
	{
		mapParam( name, value, booleans_ ) ;
	}

private:

	typedef std::map< std::string, int* > IntegersMap ;
	typedef std::map< std::string, double* > RealsMap ;
	typedef std::map< std::string, std::string* > StringsMap ;
	typedef std::map< std::string, bool* > BooleansMap ;
	//typedef std::map< std::string, std::vector<double>* > VectorsMap ;

	IntegersMap integers_ ;
	RealsMap reals_ ;
	StringsMap strings_ ;
	BooleansMap booleans_ ;
	//VectorsMap vectors_ ;

	std::list< std::string > itemNames_ ;

	template< typename ValueType, typename ContainerType >
	void
	mapParam( const std::string& name,
			  ValueType value,
			  ContainerType& container )
		throw ( std::runtime_error )
	{
		if ( std::find( itemNames_.begin(), itemNames_.end(), name ) != itemNames_.end() )
			throw std::runtime_error( "Name already used: " + name ) ;
		itemNames_.push_back( name ) ;
		container.insert( std::make_pair( name, value ) ) ;
	}

	template< typename ParamType >
	void
	loadParam( const std::string& name, ParamType& value)
    {
        if ( ros::param::get(name, value) )
            ROS_INFO("Got param: %s", name.c_str() );
        else
            ROS_FATAL("Couldn't get '%s' param.", name.c_str());
    }
} ;
} ; // namespace ros
} ; // namespace cola2


#endif /* ROS_PARAM_LOADER_H_ */

