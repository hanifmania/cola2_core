
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ROBUSTIFYINGTERMS_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ROBUSTIFYINGTERMS_H_

#include <vector>

#include "./IController.h"
#include "./Request.h"

class RT
{
 public:
    RT();
    std::vector<double> compute(const std::vector< double > feedback, Request setpoint);

 private:
    // Robustifying Terms
    float nu;
    float nv;
    float nw;
    float nq;
    float nr;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ROBUSTIFYINGTERMS_H_
