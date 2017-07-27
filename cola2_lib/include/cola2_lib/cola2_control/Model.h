
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MODEL_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MODEL_H_

#include <cmath>
#include <vector>

#include "./IController.h"

class Model
{
 public:
    Model();

    std::vector<double> compute(const std::vector<double> twist_feedback, const std::vector<double> pose_feedback);

 private:
    float Mu;
    float Mv;
    float Mw;
    float Mp;
    float Mq;
    float Mr;
    float zB;
    float W;
    float B;
    float Xuu;
    float Yvv;
    float Zww;
    float Kpp;
    float Mqq;
    float Nrr;
    float Xu;
    float Yv;
    float Zw;
    float Kp;
    float Nr;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MODEL_H_
