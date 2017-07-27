
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*
 * cola2_util.h
 *
 *  Created on: 2/2/2012
 *      Author: Narcis Palomeras
 */

#include "cola2_lib/cola2_util.h"

double cola2::util::saturate(const double x, const double min_max)
{
  if (x > min_max) return min_max;
  else if (x < -min_max) return -min_max;
  else return x;
}


double cola2::util::normalizeAngle(const double angle)
{
  return (angle + (2.0* boost::math::constants::pi<double>() * floor((boost::math::constants::pi<double>()-angle) /
                                                                     (2.0*boost::math::constants::pi<double>()))));
}


/*
 * Distance units
 */
double cola2::util::decimetersToMeters(const double value)
{
  return (value * 0.1);
}

double cola2::util::centimetersToMeters(const double value)
{
  return (value * 0.01);
}

double cola2::util::millimetersToMeters(const double value)
{
  return (value * 0.001);
}

/*
 * Angle units
 */
double cola2::util::degreesToRadians(const double value)
{
  return value * 0.0174532925;
}

double cola2::util::radiansToDegrees(const double value)
{
  return value * 57.2957795;
}

double cola2::util::gradiansToRadians(const double value)
{
  return value * boost::math::constants::pi<double>() / 200.0;
}

/*
 * Time units
 */
double cola2::util::microsecondsToSeconds(const double value)
{
  return value * 0.000001;
}

double cola2::util::millisecondsToSeconds(const double value)
{
  return value * 0.001;
}
