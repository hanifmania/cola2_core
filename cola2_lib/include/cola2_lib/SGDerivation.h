
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_SGDERIVATION_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_SGDERIVATION_H_

#include <deque>
#include <vector>

#include "./cola2_util.h"

class SGDerivation
{
 public:
    SGDerivation();
    double addValue(const double data, double dt);

 private:
    double convolve(const std::deque<double> data1, const std::vector<double> data2);

    std::deque<double> _buffer;
    std::vector<double> _savitzky_golay_coeffs;

    double _last_data;
    bool _is_buffer_init;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_SGDERIVATION_H_
