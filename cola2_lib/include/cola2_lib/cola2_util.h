/*
 * cola2_util.h
 *
 *  Created on: 2/2/2012
 *      Author: Narcis Palomeras
 */

#ifndef COLA2_UTIL_
#define COLA2_UTIL_

#include <boost/math/constants/constants.hpp>

namespace cola2{
	namespace util{

		double
		normalizeAngle(const double angle);
	
		double
		saturate(const double x, const double min_max);
	
		/*
		 * Distance units
		 */
		double
		decimetersToMeters(const double value);
	
		double
		centimetersToMeters(const double value);
	
		double
		millimetersToMeters(const double value);
	
		/*
		 * Angle units
		 */
		double
		degreesToRadians(const double value);
	
		double
		radiansToDegrees(const double value);
	
		double
		gradiansToRadians(const double value);
	
		/*
		 * Time units
		 */
		double
		microsecondsToSeconds(const double value);
	
		double
		millisecondsToSeconds(const double value);
	}; // namespace util
}; // namespace cola2


double
cola2::util::saturate(const double x, const double min_max) {
	if(x > min_max ) return min_max;
	else if(x < -min_max) return -min_max;
	else return x;
}


double
cola2::util::normalizeAngle( const double angle )
{
	return (angle + ( 2.0* boost::math::constants::pi<double>() * floor( ( boost::math::constants::pi<double>()-angle ) / ( 2.0*boost::math::constants::pi<double>() ) ) ) );
}


/*
 * Distance units
 */
double
cola2::util::decimetersToMeters( const double value ) { return ( value * 0.1 ) ; }

double
cola2::util::centimetersToMeters( const double value  ) { return ( value * 0.01 ) ; }

double
cola2::util::millimetersToMeters( const double value ) { return ( value * 0.001 ); }

/*
 * Angle units
 */
double
cola2::util::degreesToRadians( const double value ) { return value * 0.0174532925 ; }

double
cola2::util::radiansToDegrees( const double value ) { return value * 57.2957795 ; }

double
cola2::util::gradiansToRadians( const double value ) { return value * boost::math::constants::pi<double>() / 200.0 ; }

/*
 * Time units
 */
double
cola2::util::microsecondsToSeconds( const double value ) { return value * 0.000001; }

double
cola2::util::millisecondsToSeconds( const double value ) { return value * 0.001; }

#endif /* COLA2_UTIL_ */
