
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

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_UTIL_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_UTIL_H_

#include <boost/math/constants/constants.hpp>

namespace cola2 {
namespace util {

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

}  // namespace util
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_UTIL_H_
